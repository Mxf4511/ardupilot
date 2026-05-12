-- flymode_ek3src_autoswitch.lua
-- Automatic switching between AltHold/Loiter modes and EKF3 GPS/OpticalFlow sources
--
-- Prerequisites:
--   SCR_ENABLE = 1 (enable scripting)
--   Configure a downward facing rangefinder
--
--   EK3_SRC1_POSXY = 3 (GPS)
--   EK3_SRC1_POSZ  = 2 (RangeFinder; script keeps SRC1/2 POSZ fixed to rangefinder)
--   EK3_SRC1_VELXY = 3 (GPS)
--   EK3_SRC1_VELZ  = 3 (GPS)
--   EK3_SRC1_YAW   = 1 (Compass)
--
--   EK3_SRC2_POSXY = 0 (None)
--   EK3_SRC2_POSZ  = 2 (RangeFinder)
--   EK3_SRC2_VELXY = 5 (OpticalFlow)
--   EK3_SRC2_VELZ  = 0 (None)
--   EK3_SRC2_YAW   = 1 (Compass)
--
--   EK3_SRC3_POSXY = 0 (None)
--   EK3_SRC3_POSZ  = 1 (Baro)
--   EK3_SRC3_VELXY = 0 (None)
--   EK3_SRC3_VELZ  = 0 (None)
--   EK3_SRC3_YAW   = 1 (Compass)
--
--   EK3_SRC_OPTIONS = 0 (Do not fuse all velocities)
--
-- md §1: MODE_MD1_AUTO (4) = script auto AltHold<->Loiter; MODE_MD1_MANUAL (6) = fixed SRC3.
--   Stock Copter: 4=GUIDED, 6=RTL — map FLTMODE or change constants to match your airframe doc.
--
-- Tuning parameters (via SCR_USERn):
--   SCR_USER1 = GPS speed accuracy threshold (m/s), default 0.5
--   SCR_USER4 = optical flow quality threshold, default 50
--   SCR_USER5 = optical flow innovation threshold, default 0.15
--
-- GPS hAcc/vAcc: gps:horizontal_accuracy(i) / vertical_accuracy(i) / speed_accuracy(i) each return
-- number|nil (see docs.lua), not (boolean, number). Use a single return value per call.
--
-- SRC1/SRC2 POSZ: fixed RangeFinder (ensure_src12_posz_rangefinder).
--
-- Anti-chatter (noisy sensors / bench): Loiter -> AltHold uses DEGRADE_DWELL_TICKS (no arm / flying / HAGL gate).
--
-- luacheck: only 0
---@diagnostic disable: cast-local-type
---@diagnostic disable: need-check-nil

-- Constants
local COPTER_MODE_ALT_HOLD = 2
local COPTER_MODE_LOITER   = 5
-- flymode_ek3src_autoswitch.md §1 (flight mode numbers from vehicle:get_mode())
local MODE_MD1_AUTO   = 4
local MODE_MD1_MANUAL = 6
local RANGEFINDER_ROTATION = 25  -- downward facing (ROTATION_PITCH_90)
local UPDATE_INTERVAL_MS   = 100 -- 10Hz update rate
local VOTE_COUNTER_MAX     = 10  -- votes needed to switch (2 seconds at 10Hz)
local LOG_INTERVAL_MS      = 2000
-- RangeFinder::Status from rangefinder:status_orient (see AP_RangeFinder.h)
local RF_STATUS_GOOD       = 4
-- Dwell: consecutive update() calls at UPDATE_INTERVAL_MS before auto mode change (reduces ground / RF jitter)
local DEGRADE_DWELL_TICKS  = 30  -- Loiter -> AltHold when both nav bad (~3 s at 100 ms)
local EK3_POSZ_RANGEFINDER = 2
-- GPS OK (for voting): satellites strictly > 15; p_acc = sqrt(h^2+v^2) strictly < 1 m
local GPS_OK_MIN_SATS_EXCLUSIVE = 15
local GPS_OK_MAX_POS_ACC_M      = 1.0

-- State variables
local source_prev          = ahrs:get_posvelyaw_source_set()
local gps_vs_flow_vote     = 0   -- negative = GPS, positive = optical flow
-- Start negative so the first telemetry block can send as soon as we enter AltHold/Loiter
local last_log_ms          = -LOG_INTERVAL_MS
local auto_mode_engaged    = false  -- Loiter GPS/flow voting (false after script degrade until Loiter again)
local md1_auto_armed       = false  -- md §1: true after mode 4 until mode 6 or non-(2|5) flight mode
local manual_to_auto_bridge = false -- md §2: 6->4: stay AltHold+SRC3 until §5 nav ready, then Loiter+SRC
local boot_announced       = false
local bad_nav_streak       = 0   -- Loiter: consecutive cycles with GPS and flow both bad
local posz_prev = -1
local prev_flow_innov_xy = nil
local last_loiter_retry_ms = 0
-- md §1: mode 6 => manual (m:1); mode 4 => auto branch (a:1 after Loiter retry). Script degrade sets a:0 without m:1.
local user_manual_althold = false

-- Initialise parameters
local scr_user1 = Parameter('SCR_USER1')  -- GPS speed accuracy threshold
local scr_user4 = Parameter('SCR_USER4')  -- optical flow quality threshold
local scr_user5 = Parameter('SCR_USER5')  -- optical flow innovation threshold

-- EK3 tertiary set: baro height, compass yaw, no horizontal pos/vel (see flymode_ek3src_autoswitch.md §7)
local ek3_src3_posxy = Parameter('EK3_SRC3_POSXY')
local ek3_src3_posz = Parameter('EK3_SRC3_POSZ')
local ek3_src3_velxy = Parameter('EK3_SRC3_VELXY')
local ek3_src3_velz = Parameter('EK3_SRC3_VELZ')
local ek3_src3_yaw = Parameter('EK3_SRC3_YAW')
local ek3_src1_posz = Parameter('EK3_SRC1_POSZ')
local ek3_src2_posz = Parameter('EK3_SRC2_POSZ')

assert(optical_flow, 'could not access optical flow')
assert(rangefinder, 'could not access rangefinder')

-- Play a buzzer tune to indicate source change
local function play_source_tune(source)
  if source == 0 then
    notify:play_tune("L8C")      -- one long low tone for Source1 (GPS)
  elseif source == 1 then
    notify:play_tune("L12DD")    -- two medium tones for Source2 (OpticalFlow)
  elseif source == 2 then
    notify:play_tune("L16FFF")   -- three fast high tones for Source3 (Baro-only fallback)
  end
end

-- Match spec: SRC3 = none/baro/none/none/compass (uses RAM param set(), not flash)
local function apply_ek3_src3_baro_fallback_params()
  ek3_src3_posxy:set(0)
  ek3_src3_posz:set(1)
  ek3_src3_velxy:set(0)
  ek3_src3_velz:set(0)
  ek3_src3_yaw:set(1)
end

-- Apply SRC3 fallback params at boot so they are correct regardless of flash defaults
apply_ek3_src3_baro_fallback_params()

-- Set EKF source and notify
local function set_source(source)
  if source ~= source_prev then
    source_prev = source
    ahrs:set_posvelyaw_source_set(source_prev)
    local names = { [0] = "1(GPS)", [1] = "2(Flow)", [2] = "3(Baro)" }
    gcs:send_text(0, "EK3 Source -> " .. (names[source_prev] or tostring(source_prev)))
    play_source_tune(source_prev)
  end
end

local function enter_althold_fallback_nav()
  apply_ek3_src3_baro_fallback_params()
  set_source(2)
end

local function set_src12_posz(posz_source)
  if posz_prev ~= posz_source then
    posz_prev = posz_source
    ek3_src1_posz:set(posz_source)
    ek3_src2_posz:set(posz_source)
    local names = { [1] = "Baro", [2] = "RangeFinder", [3] = "GPS" }
    gcs:send_text(0, "EK3 SRC1/2 POSZ -> " .. (names[posz_source] or tostring(posz_source)))
  end
end

-- SRC1/SRC2 vertical position: always rangefinder (no Baro/GPS Z switching).
local function ensure_src12_posz_rangefinder()
  set_src12_posz(EK3_POSZ_RANGEFINDER)
end

-- Get parameter value with a default fallback
local function get_param(param_obj, default_val)
  local v = param_obj:get()
  if (v == nil) or (v <= 0) then
    return default_val
  end
  return v
end

-- Check GPS quality for voting
-- Returns true if GPS meets all thresholds (sats > 15, p_acc < 1 m, plus 3D fix and speed gate)
local function gps_quality_ok(gps_speed_acc_thresh)
  local primary = gps:primary_sensor()

  -- Must have at least 3D fix
  if gps:status(primary) < gps.GPS_OK_FIX_3D then
    return false
  end

  -- Satellite count: strictly more than 15 (>= 16)
  if gps:num_sats(primary) <= GPS_OK_MIN_SATS_EXCLUSIVE then
    return false
  end

  -- Speed accuracy (m/s); nil if driver does not provide it
  local speed_acc = gps:speed_accuracy(primary)
  if (speed_acc == nil) or (speed_acc > gps_speed_acc_thresh) then
    return false
  end

  -- p_acc = sqrt(hacc^2 + vacc^2); must be < 1 m when both components available
  local h_acc = gps:horizontal_accuracy(primary)
  local v_acc = gps:vertical_accuracy(primary)
  if (h_acc == nil) or (v_acc == nil) then
    return false
  end
  local pos_acc = math.sqrt(h_acc * h_acc + v_acc * v_acc)
  if pos_acc >= GPS_OK_MAX_POS_ACC_M then
    return false
  end

  return true
end

-- Check optical flow quality for voting
-- Returns true if optical flow sensor data is usable
local function opticalflow_quality_ok(flow_quality_thresh, flow_innov_thresh)
  if not optical_flow then
    return false
  end

  -- Only check quality; skip enabled()/healthy() because those can reflect
  -- the EKF's willingness to fuse the data (false when EKF is on a source
  -- set that doesn't include optical flow), creating a circular deadlock.
  local quality = optical_flow:quality()
  if quality < flow_quality_thresh then
    return false
  end

  -- Check innovations from EKF for optical flow source.
  -- When EKF is not fusing optical flow (e.g. stuck on SRC3), innovations
  -- freeze at a non-zero constant.  Detect this: if the last two reads are
  -- identical the data is stale and should not fail the check.
  local innov = Vector3f()
  local innov_var = Vector3f()
  innov, innov_var = ahrs:get_vel_innovations_and_variances_for_source(5)
  if innov then
    local xy_innov = math.sqrt(innov:x() * innov:x() + innov:y() * innov:y())
    if xy_innov > flow_innov_thresh and xy_innov > 0.0 then
      -- Possible frozen innovation (EKF not fusing flow) – compare with
      -- previous sample to decide whether it is genuine or stale.
      if not prev_flow_innov_xy then
        -- First sample, store and reject to establish baseline
        prev_flow_innov_xy = xy_innov
        return false
      end
      local prev = prev_flow_innov_xy
      prev_flow_innov_xy = xy_innov
      -- If innovation hasn't changed, EKF is not fusing flow → stale, ignore
      if math.abs(xy_innov - prev) < 1e-6 then
        return true
      end
      -- Innovation is changing → genuine high innovation → reject
      return false
    end
    prev_flow_innov_xy = xy_innov
  end

  return true
end

-- Determine if user manually set mode to AltHold or Loiter
-- We detect "manual" mode changes by checking if the mode differs from what
-- the auto-switch logic expects. A simple approach: when auto logic is NOT
-- engaged and user enters Loiter, we engage. When user leaves Loiter manually,
-- we disengage.

local prev_user_mode = vehicle:get_mode()

local function detect_user_mode_change(current_mode)
  local changed = (current_mode ~= prev_user_mode)
  local old_mode = prev_user_mode
  prev_user_mode = current_mode
  return changed, old_mode
end

local function position_estimate_ready()
  return ahrs:get_position() ~= nil
end

-- Main update function
function update()
  local now_ms = millis()

  if not boot_announced then
    boot_announced = true
    ensure_src12_posz_rangefinder()
    -- MAV_SEVERITY_WARNING (4): default Mission Planner / QGC filters often hide INFO (6)
    gcs:send_text(4, "flymode_ek3src: script running")
    notify:send_text("ek3src run", 0)
  end

  -- Read tuning parameters
  local gps_speed_acc_thresh  = get_param(scr_user1, 0.5)
  local flow_quality_thresh   = get_param(scr_user4, 50)
  local flow_innov_thresh     = get_param(scr_user5, 0.15)

  local current_mode = vehicle:get_mode()
  local mode_changed, old_mode = detect_user_mode_change(current_mode)

  -- md §1: mode 4 = auto (engage + jump to AltHold); mode 6 = manual SRC3 only.
  if mode_changed then
    if current_mode == MODE_MD1_AUTO then
      user_manual_althold = false
      auto_mode_engaged = true
      md1_auto_armed = true
      if old_mode == MODE_MD1_MANUAL then
        manual_to_auto_bridge = true
        gcs:send_text(4, "md2: bridge AH+SRC3 until nav OK")
      else
        manual_to_auto_bridge = false
      end
      gcs:send_text(4, "md1: auto slot -> AltHold")
    elseif current_mode == MODE_MD1_MANUAL then
      user_manual_althold = true
      auto_mode_engaged = false
      md1_auto_armed = false
      manual_to_auto_bridge = false
      enter_althold_fallback_nav()
      gcs:send_text(4, "md1: manual SRC3 (slot6)")
      return update, UPDATE_INTERVAL_MS
    elseif (current_mode ~= COPTER_MODE_ALT_HOLD) and (current_mode ~= COPTER_MODE_LOITER) and
           (current_mode ~= MODE_MD1_AUTO) and (current_mode ~= MODE_MD1_MANUAL) then
      user_manual_althold = false
      auto_mode_engaged = false
      md1_auto_armed = false
      manual_to_auto_bridge = false
      return update, UPDATE_INTERVAL_MS
    end
    if (old_mode == MODE_MD1_MANUAL) and (current_mode ~= MODE_MD1_MANUAL) then
      user_manual_althold = false
    end
  end

  -- Stay in md §1 manual slot (e.g. every frame while mode 6 active)
  if current_mode == MODE_MD1_MANUAL then
    user_manual_althold = true
    auto_mode_engaged = false
    md1_auto_armed = false
    manual_to_auto_bridge = false
    if source_prev ~= 2 then
      enter_althold_fallback_nav()
    end
    return update, UPDATE_INTERVAL_MS
  end

  -- md §1 auto slot: try to leave mode 4 into AltHold so §4–§8 can run in 2/5
  if current_mode == MODE_MD1_AUTO then
    user_manual_althold = false
    auto_mode_engaged = true
    md1_auto_armed = true
    if vehicle:set_mode(COPTER_MODE_ALT_HOLD) then
      prev_user_mode = COPTER_MODE_ALT_HOLD
    end
    return update, UPDATE_INTERVAL_MS
  end

  if (current_mode ~= COPTER_MODE_ALT_HOLD) and (current_mode ~= COPTER_MODE_LOITER) then
    return update, UPDATE_INTERVAL_MS
  end

  -- Diagnostics (compute whenever in AltHold/Loiter so periodic logs still run if auto is off)
  local gps_ok = gps_quality_ok(gps_speed_acc_thresh)

  local rngfnd_dist_m = 0
  local rngfnd_has_data = false
  if rangefinder:has_data_orient(RANGEFINDER_ROTATION) then
    rngfnd_has_data = true
    rngfnd_dist_m = rangefinder:distance_cm_orient(RANGEFINDER_ROTATION) * 0.01
  end

  local rf_status = rangefinder:status_orient(RANGEFINDER_ROTATION)
  local rf_alt_ok = rngfnd_has_data and (rf_status == RF_STATUS_GOOD)
  local flow_ok = opticalflow_quality_ok(flow_quality_thresh, flow_innov_thresh)
  local flow_and_rf_ok = flow_ok and rf_alt_ok

  -- Periodic telemetry: severity 6 (INFO) is often hidden in MP/QGC; use 5 (NOTICE).
  if (now_ms - last_log_ms) >= LOG_INTERVAL_MS then
    last_log_ms = now_ms

    local primary = gps:primary_sensor()
    local h_acc = gps:horizontal_accuracy(primary)
    local v_acc = gps:vertical_accuracy(primary)
    local p_acc = nil
    if (h_acc ~= nil) and (v_acc ~= nil) then
      p_acc = math.sqrt(h_acc * h_acc + v_acc * v_acc)
    end
    local h_str = (h_acc ~= nil) and string.format("%.1f", h_acc) or "-"
    local v_str = (v_acc ~= nil) and string.format("%.1f", v_acc) or "-"
    local p_str = (p_acc ~= nil) and string.format("%.1f", p_acc) or "-"

    local flow_q = 0
    if optical_flow then
      flow_q = optical_flow:quality()
    end

    local innov_xy_str = "-"
    if optical_flow then
      local innov = ahrs:get_vel_innovations_and_variances_for_source(5)
      if innov then
        local xy_innov = math.sqrt(innov:x() * innov:x() + innov:y() * innov:y())
        innov_xy_str = string.format("%.3f", xy_innov)
      end
    end
    local rf_str = rngfnd_has_data and string.format("%.1f", rngfnd_dist_m) or "-"

    -- STATUSTEXT is 50 chars max; keep each line short for GCS display
    -- gcs:send_text(4, "----------------------------------------")
    -- gcs:send_text(4, string.format(
    --   "gps H:%s V:%s P:%s vote:%d %s",
    --   h_str,
    --   v_str,
    --   p_str,
    --   gps_vs_flow_vote,
    --   gps_ok and "OK" or "NO"
    -- ))
    -- gcs:send_text(4, string.format(
    --   "flow Q:%u Inn:%s R:%s vote:%d %s",
    --   flow_q,
    --   innov_xy_str,
    --   rf_str,
    --   gps_vs_flow_vote,
    --   flow_and_rf_ok and "OK" or "NO"
    -- ))
    -- local mode_str = (current_mode == COPTER_MODE_LOITER) and "Loit" or "AltH"
    -- gcs:send_text(4, string.format(
    --   "src/mode S:%u %s a:%u m:%u",
    --   source_prev + 1,
    --   mode_str,
    --   auto_mode_engaged and 1 or 0,
    --   user_manual_althold and 1 or 0
    -- ))
  end

  -- Loiter voting / degrade only when auto is engaged. AltHold->Loiter retry still runs when a:0
  -- so manual AltHold can recover into Loiter and re-engage (see set_mode success below).
  if auto_mode_engaged then
    -- md §4/§5: when both GPS and Flow+Rangefinder meet thresholds, always prefer SRC2 (flow)
    if current_mode == COPTER_MODE_LOITER and flow_and_rf_ok and gps_ok then
      set_source(1)
      gps_vs_flow_vote = VOTE_COUNTER_MAX
    end

    -- When coming from manual AltHold fallback (SRC3), don't stay on SRC3 in auto Loiter path.
    if current_mode == COPTER_MODE_LOITER and source_prev == 2 then
      if flow_and_rf_ok then
        set_source(1)
      elseif gps_ok then
        set_source(0)
      end
    end

    -- md §8: GPS source active but GPS degrades -> prefer flow when available
    if current_mode == COPTER_MODE_LOITER and source_prev == 0 and (not gps_ok) and flow_and_rf_ok then
      set_source(1)
      gps_vs_flow_vote = math.min(gps_vs_flow_vote + 2, VOTE_COUNTER_MAX)
    end

    if flow_and_rf_ok then
      gps_vs_flow_vote = math.min(gps_vs_flow_vote + 1, VOTE_COUNTER_MAX)
    elseif gps_ok then
      gps_vs_flow_vote = math.max(gps_vs_flow_vote - 1, -VOTE_COUNTER_MAX)
    end

    local voted_source = -1
    if gps_vs_flow_vote <= -VOTE_COUNTER_MAX then
      voted_source = 0
    elseif gps_vs_flow_vote >= VOTE_COUNTER_MAX then
      if flow_and_rf_ok then
        voted_source = 1
      end
    end

    if current_mode == COPTER_MODE_LOITER then
      if (not gps_ok) and (not flow_and_rf_ok) then
        bad_nav_streak = bad_nav_streak + 1
      else
        bad_nav_streak = math.max(bad_nav_streak - 1, 0)
      end

      if bad_nav_streak >= DEGRADE_DWELL_TICKS then
        if vehicle:set_mode(COPTER_MODE_ALT_HOLD) then
          enter_althold_fallback_nav()
          gcs:send_text(0, "Auto -> AltHold (GPS+Flow bad, EK3_SRC3+Src3)")
          prev_user_mode = COPTER_MODE_ALT_HOLD
          -- Loiter voting block is for a:1; after script degrade, show a:0 (same as
          -- md: AltHold->Loiter retry still runs when a:0). Do not set m:1 here or
          -- we would block auto Loiter recovery until user selects Loiter again.
          auto_mode_engaged = false
        end
        bad_nav_streak = 0
      end

      if voted_source >= 0 then
        set_source(voted_source)
      end
    end
  else
    bad_nav_streak = 0
  end

  if (current_mode == COPTER_MODE_ALT_HOLD) or (current_mode == COPTER_MODE_LOITER) then
    ensure_src12_posz_rangefinder()
  end

  if current_mode == COPTER_MODE_ALT_HOLD then
    if user_manual_althold then
      -- md §1 mode 6: SRC3 only, no auto Loiter
      return update, UPDATE_INTERVAL_MS
    end
    -- md §1: AltHold<->Loiter automation only after entering auto via mode 4
    if not md1_auto_armed then
      return update, UPDATE_INTERVAL_MS
    end
    -- md §2: manual(6)->auto(4): hold AltHold+SRC3 until §5 nav ready, then Loiter+SRC together
    if manual_to_auto_bridge then
      if source_prev ~= 2 then
        enter_althold_fallback_nav()
        return update, UPDATE_INTERVAL_MS
      end
      if (not flow_and_rf_ok) and (not gps_ok) then
        return update, UPDATE_INTERVAL_MS
      end
      if not position_estimate_ready() then
        return update, UPDATE_INTERVAL_MS
      end
      local bridge_src = -1
      if flow_and_rf_ok then
        bridge_src = 1
      elseif gps_ok then
        bridge_src = 0
      end
      if bridge_src < 0 then
        return update, UPDATE_INTERVAL_MS
      end
      if source_prev ~= bridge_src then
        set_source(bridge_src)
        return update, UPDATE_INTERVAL_MS
      end
      if (now_ms - last_loiter_retry_ms) >= 1000 then
        last_loiter_retry_ms = now_ms
        if vehicle:set_mode(COPTER_MODE_LOITER) then
          manual_to_auto_bridge = false
          auto_mode_engaged = true
          prev_user_mode = COPTER_MODE_LOITER
          gcs:send_text(0, "md2: Loiter+SRC" .. (bridge_src == 0 and "1(GPS)" or "2(Flow)"))
        end
      end
      return update, UPDATE_INTERVAL_MS
    end
    -- md §5: In AltHold with auto engaged, if Flow/GPS is available, switch EKF source then Loiter.
    local desired_source = -1
    if flow_and_rf_ok then
      desired_source = 1
    elseif gps_ok then
      desired_source = 0
    end

    if desired_source < 0 then
      if auto_mode_engaged then
        enter_althold_fallback_nav()
      end
      return update, UPDATE_INTERVAL_MS
    end

    local source_switched = (source_prev ~= desired_source)
    set_source(desired_source)

    -- Give EKF one cycle to apply the new source set before trying Loiter.
    if source_switched then
      return update, UPDATE_INTERVAL_MS
    end

    -- Keep retrying Loiter while source is valid.
    if position_estimate_ready() then
      if (now_ms - last_loiter_retry_ms) >= 1000 then
        last_loiter_retry_ms = now_ms
        if vehicle:set_mode(COPTER_MODE_LOITER) then
          auto_mode_engaged = true
          gcs:send_text(0, "Auto -> Loiter (source " .. (desired_source == 0 and "GPS" or "Flow") .. " ready)")
          prev_user_mode = COPTER_MODE_LOITER
        end
      end
    end
  end

  return update, UPDATE_INTERVAL_MS
end

return update()
