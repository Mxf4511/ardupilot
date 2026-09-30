-- mode_switch.lua
-- Replaces Copter FLTMODE_CH: a 3-position RC switch selects
-- AltHold / Loiter(GPS) / Loiter(OpticalFlow) and switches EK3 source set.
--
-- Prerequisites:
--   SCR_ENABLE = 1
--   FLTMODE_CH = 0   (disable native flight-mode channel; this script owns mode)
--   Do not assign RCx_OPTION=RTL on RC_RTL_CHANNEL; this script drives RTL from PWM.
--
--   EK3_SRC1_* configured for GPS positioning (Loiter + GPS)
--   EK3_SRC2_* configured for optical-flow positioning (Loiter + Flow)
--   Example:
--     EK3_SRC1_POSXY = 3 (GPS)
--     EK3_SRC1_VELXY = 3 (GPS)
--     EK3_SRC1_VELZ  = 3 (GPS)
--     EK3_SRC1_POSZ  = 1 (Baro) or 2 (RangeFinder)
--     EK3_SRC1_YAW   = 1 (Compass)
--     EK3_SRC2_POSXY = 0 (None)
--     EK3_SRC2_VELXY = 5 (OpticalFlow)
--     EK3_SRC2_VELZ  = 0 (None)
--     EK3_SRC2_POSZ  = 1 (Baro) or 2 (RangeFinder)
--     EK3_SRC2_YAW   = 1 (Compass)
--     EK3_SRC_OPTIONS = 0
--
-- RC_CHANNEL (default CH5) mode switch:
--   Low  (~1000) -> Loiter + EK3_SRC1 (GPS)
--   Mid  (~1500) -> AltHold
--   High (~2000) -> Loiter + EK3_SRC2 (OpticalFlow)
--
-- RC_RTL_CHANNEL (default CH6) RTL override:
--   High (~2000) -> RTL; RC5 ignored
--   Mid/Low      -> leave RTL and immediately apply RC5 mode
--
-- AUTO / Guided (航线): RC5 is ignored entirely; only RC6 RTL override can interrupt.
--
-- luacheck: only 0
---@diagnostic disable: cast-local-type
---@diagnostic disable: need-check-nil

------------------------------------------------------------------------
-- User configuration
------------------------------------------------------------------------
local RC_CHANNEL      = 5      -- mode-switch RC channel (replaces FLTMODE_CH), default CH5
local RC_RTL_CHANNEL  = 6      -- RTL override channel, default CH6 (high = RTL)
local PWM_LOW         = 1000   -- low position PWM
local PWM_MID         = 1500   -- middle position PWM
local PWM_HIGH        = 2000   -- high position PWM
local PWM_DEADZONE    = 100    -- +/- us around each position still counts as that position
local SWAP_LOW_HIGH   = false  -- true: swap low/high (low=Flow Loiter, high=GPS Loiter)
local UPDATE_MS       = 100    -- update period

------------------------------------------------------------------------
-- Constants
------------------------------------------------------------------------
local COPTER_MODE_ALT_HOLD  = 2
local COPTER_MODE_AUTO      = 3
local COPTER_MODE_GUIDED    = 4
local COPTER_MODE_LOITER    = 5
local COPTER_MODE_RTL       = 6
local COPTER_MODE_SMART_RTL = 21
local COPTER_MODE_AUTO_RTL  = 27

local POS_NONE = -1
local POS_LOW  = 0
local POS_MID  = 1
local POS_HIGH = 2

local EK3_SRC1 = 0  -- ahrs source set index for EK3_SRC1
local EK3_SRC2 = 1  -- ahrs source set index for EK3_SRC2

------------------------------------------------------------------------
-- State
------------------------------------------------------------------------
local switch_pos_prev   = POS_NONE  -- last classified RC5 position
local last_applied_pos  = POS_NONE  -- last RC5 position actually applied to vehicle
local rtl_pos_prev      = POS_NONE
local rtl_override      = false  -- true while RC_RTL_CHANNEL is held high
local source_prev       = ahrs:get_posvelyaw_source_set()
local boot_announced    = false
local fltmode_ch_param  = Parameter('FLTMODE_CH')

-- md §7: AUTO/Guided — RC5 must not interrupt mission flight
local function is_mission_mode(current_mode)
  return (current_mode == COPTER_MODE_AUTO) or
         (current_mode == COPTER_MODE_GUIDED)
end

local function is_rtl_mode(current_mode)
  return (current_mode == COPTER_MODE_RTL) or
         (current_mode == COPTER_MODE_SMART_RTL) or
         (current_mode == COPTER_MODE_AUTO_RTL)
end

local function play_source_tune(source)
  if source == EK3_SRC1 then
    notify:play_tune("L8C")    -- one long low tone: SRC1 / GPS
  elseif source == EK3_SRC2 then
    notify:play_tune("L12DD")  -- two medium tones: SRC2 / Flow
  end
end

local function set_ek3_source(source)
  if source ~= source_prev then
    source_prev = source
    ahrs:set_posvelyaw_source_set(source_prev)
    local names = { [EK3_SRC1] = "SRC1(GPS)", [EK3_SRC2] = "SRC2(Flow)" }
    gcs:send_text(4, "mode_switch: EK3 -> " .. (names[source_prev] or tostring(source_prev)))
    play_source_tune(source_prev)
  end
end

-- Classify PWM into low/mid/high using deadzones around each nominal value.
-- Values in the gaps between deadzones keep the previous position (hysteresis).
local function classify_switch(pwm, prev_pos)
  if math.abs(pwm - PWM_LOW) <= PWM_DEADZONE then
    return POS_LOW
  end
  if math.abs(pwm - PWM_MID) <= PWM_DEADZONE then
    return POS_MID
  end
  if math.abs(pwm - PWM_HIGH) <= PWM_DEADZONE then
    return POS_HIGH
  end
  return prev_pos
end

-- Map switch position to flight mode + EK3 source set.
-- Returns mode, ek3_source_or_nil (nil means leave source unchanged for AltHold)
local function position_to_action(pos)
  if pos == POS_MID then
    return COPTER_MODE_ALT_HOLD, nil
  end

  local want_gps = (pos == POS_LOW)
  if SWAP_LOW_HIGH then
    want_gps = (pos == POS_HIGH)
  end

  if want_gps then
    return COPTER_MODE_LOITER, EK3_SRC1
  end
  return COPTER_MODE_LOITER, EK3_SRC2
end

local function apply_position(pos)
  local mode, ek3_src = position_to_action(pos)

  -- Switch EKF source before requesting Loiter so position estimate uses the right sensors.
  if ek3_src ~= nil then
    set_ek3_source(ek3_src)
  end

  local current_mode = vehicle:get_mode()
  if current_mode ~= mode then
    if vehicle:set_mode(mode) then
      last_applied_pos = pos
      local mode_names = {
        [COPTER_MODE_ALT_HOLD] = "AltHold",
        [COPTER_MODE_LOITER]   = "Loiter",
      }
      local pos_names = { [POS_LOW] = "LOW", [POS_MID] = "MID", [POS_HIGH] = "HIGH" }
      local src_txt = ""
      if ek3_src == EK3_SRC1 then
        src_txt = " + GPS"
      elseif ek3_src == EK3_SRC2 then
        src_txt = " + Flow"
      end
      gcs:send_text(4, string.format(
        "mode_switch: %s -> %s%s",
        pos_names[pos] or "?",
        mode_names[mode] or tostring(mode),
        src_txt
      ))
    else
      gcs:send_text(3, "mode_switch: set_mode failed")
    end
  else
    last_applied_pos = pos
    -- Already in target mode; source change (if any) already applied above.
  end
end

local function enter_rtl()
  local current_mode = vehicle:get_mode()
  if not is_rtl_mode(current_mode) then
    if vehicle:set_mode(COPTER_MODE_RTL) then
      gcs:send_text(4, "mode_switch: RC RTL -> RTL")
    else
      gcs:send_text(3, "mode_switch: RTL set_mode failed")
    end
  end
end

function update()
  if not boot_announced then
    boot_announced = true
    local fltmode_ch = fltmode_ch_param:get()
    if (fltmode_ch ~= nil) and (fltmode_ch ~= 0) then
      gcs:send_text(2, string.format(
        "mode_switch: set FLTMODE_CH=0 (now %d), else native mode fights script",
        fltmode_ch
      ))
    end
    gcs:send_text(4, string.format(
      "mode_switch: mode CH%u RTL CH%u dz=%u swap=%s",
      RC_CHANNEL,
      RC_RTL_CHANNEL,
      PWM_DEADZONE,
      SWAP_LOW_HIGH and "Y" or "N"
    ))
  end

  -- Track mode switch (RC5) continuously so RTL exit can restore the latest position
  local pwm = rc:get_pwm(RC_CHANNEL)
  local pos = switch_pos_prev
  if (pwm ~= nil) and (pwm > 0) then
    pos = classify_switch(pwm, switch_pos_prev)
    if pos ~= POS_NONE then
      switch_pos_prev = pos
    end
  end

  -- RTL override (RC6): high = RTL and lock out RC5; mid/low = restore RC5 immediately
  local rtl_pwm = rc:get_pwm(RC_RTL_CHANNEL)
  if (rtl_pwm ~= nil) and (rtl_pwm > 0) then
    local rtl_pos = classify_switch(rtl_pwm, rtl_pos_prev)
    if rtl_pos ~= POS_NONE then
      rtl_pos_prev = rtl_pos
    end
  end

  local rtl_active = (rtl_pos_prev == POS_HIGH)

  if rtl_active then
    if not rtl_override then
      rtl_override = true
      gcs:send_text(4, "mode_switch: RTL override ON (RC5 locked)")
    end
    enter_rtl()
    return update, UPDATE_MS
  end

  -- RC6 left high: immediately restore RC5 mode
  if rtl_override then
    rtl_override = false
    gcs:send_text(4, "mode_switch: RTL override OFF, restore RC5")
    if switch_pos_prev ~= POS_NONE then
      apply_position(switch_pos_prev)
    end
    return update, UPDATE_MS
  end

  if switch_pos_prev == POS_NONE then
    return update, UPDATE_MS
  end

  local current_mode = vehicle:get_mode()

  -- AUTO/Guided: RC5 cannot interrupt; only RC6 RTL can
  if is_mission_mode(current_mode) then
    -- Keep tracking RC5 so leaving mission (via GCS) can sync later if needed,
    -- but never apply RC5 while mission mode is active.
    last_applied_pos = switch_pos_prev
    return update, UPDATE_MS
  end

  -- First apply after boot / after leaving mission
  if last_applied_pos == POS_NONE then
    apply_position(switch_pos_prev)
    return update, UPDATE_MS
  end

  if switch_pos_prev ~= last_applied_pos then
    apply_position(switch_pos_prev)
  end

  return update, UPDATE_MS
end

return update()
