-- flymode_ek3src_autoswitch.lua
-- Automatic switching between AltHold/Loiter modes and EKF3 GPS/OpticalFlow sources
--
-- Prerequisites:
--   SCR_ENABLE = 1 (enable scripting)
--   Configure a downward facing rangefinder
--
--   EK3_SRC1_POSXY = 3 (GPS)
--   EK3_SRC1_POSZ  = 2 (Baro)
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
-- Tuning parameters (via SCR_USERn):
--   SCR_USER1 = GPS speed accuracy threshold (m/s), default 0.5
--   SCR_USER2 = GPS position accuracy threshold (m), sqrt(hacc^2+vacc^2), default 2.0
--   SCR_USER3 = GPS minimum satellite count, default 8
--   SCR_USER4 = optical flow quality threshold, default 50
--   SCR_USER5 = optical flow innovation threshold, default 0.15
--
-- Anti-chatter (noisy sensors / bench): Loiter -> AltHold uses DEGRADE_DWELL_TICKS (no arm / flying / HAGL gate).
--
-- luacheck: only 0
---@diagnostic disable: cast-local-type
---@diagnostic disable: need-check-nil

-- Constants
local COPTER_MODE_ALT_HOLD = 2
local COPTER_MODE_LOITER   = 5
local RANGEFINDER_ROTATION = 25  -- downward facing (ROTATION_PITCH_90)
local UPDATE_INTERVAL_MS   = 100 -- 10Hz update rate
local VOTE_COUNTER_MAX     = 10  -- votes needed to switch (2 seconds at 10Hz)
local LOG_INTERVAL_MS      = 2000
local RF_ALT_RATIO         = 0.9 -- rangefinder max range ratio for alt source switching
local RF_HYST_CENTER       = 0.8 -- new requirement: center of RF hysteresis comparator
local RF_HYST_WIDTH        = 0.1 -- hysteresis half-width around center (0.8 +/- 0.1)
-- Dwell: consecutive update() calls at UPDATE_INTERVAL_MS before auto mode change (reduces ground / RF jitter)
local DEGRADE_DWELL_TICKS  = 30  -- Loiter -> AltHold when both nav bad (~3 s at 100 ms)
local EK3_POSZ_BARO        = 1
local EK3_POSZ_RANGEFINDER = 2
local EK3_POSZ_GPS         = 3

-- State variables
local source_prev          = ahrs:get_posvelyaw_source_set()
local gps_vs_flow_vote     = 0   -- negative = GPS, positive = optical flow
-- Start negative so the first telemetry block can send as soon as we enter AltHold/Loiter
local last_log_ms          = -LOG_INTERVAL_MS
local auto_mode_engaged    = false  -- true when script is managing the mode
local boot_announced       = false
local bad_nav_streak       = 0   -- Loiter: consecutive cycles with GPS and flow both bad
local rf_hyst_use_rangefinder = true -- md §1: boot defaults to rangefinder height source
local posz_prev = -1
local prev_flow_innov_xy = nil
local last_loiter_retry_ms = 0

-- Initialise parameters
local scr_user1 = Parameter('SCR_USER1')  -- GPS speed accuracy threshold
local scr_user2 = Parameter('SCR_USER2')  -- GPS position accuracy threshold (sqrt(hacc^2+vacc^2))
local scr_user3 = Parameter('SCR_USER3')  -- GPS minimum satellite count
local scr_user4 = Parameter('SCR_USER4')  -- optical flow quality threshold
local scr_user5 = Parameter('SCR_USER5')  -- optical flow innovation threshold

-- EK3 tertiary set: baro height, compass yaw, no horizontal pos/vel (see flymode_ek3src_autoswitch.md §5)
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

-- Get parameter value with a default fallback
local function get_param(param_obj, default_val)
  local v = param_obj:get()
  if (v == nil) or (v <= 0) then
    return default_val
  end
  return v
end

-- Check rangefinder altitude and return which EKF alt source set to use
-- Returns true if rangefinder should be used for altitude, false if baro
local function rangefinder_alt_usable()
  if not rangefinder:has_data_orient(RANGEFINDER_ROTATION) then
    return false
  end
  local rngfnd_dist_cm = rangefinder:distance_cm_orient(RANGEFINDER_ROTATION)
  local rngfnd_max_cm  = rangefinder:max_distance_cm_orient(RANGEFINDER_ROTATION)
  if rngfnd_max_cm == 0 then
    return false
  end
  -- usable when distance <= 90% of max range
  return rngfnd_dist_cm <= (rngfnd_max_cm * RF_ALT_RATIO)
end

local function update_height_source_with_hysteresis(current_mode, gps_ok, gps_nav_active, rngfnd_has_data, rngfnd_dist_m, rngfnd_max_m)
  if not rngfnd_has_data or rngfnd_max_m <= 0 then
    rf_hyst_use_rangefinder = false
  else
    local hi = (RF_HYST_CENTER + RF_HYST_WIDTH) * rngfnd_max_m
    local lo = (RF_HYST_CENTER - RF_HYST_WIDTH) * rngfnd_max_m
    if rf_hyst_use_rangefinder then
      if rngfnd_dist_m > hi then
        rf_hyst_use_rangefinder = false
      end
    else
      if rngfnd_dist_m < lo then
        rf_hyst_use_rangefinder = true
      end
    end
  end

  if rf_hyst_use_rangefinder then
    set_src12_posz(EK3_POSZ_RANGEFINDER)
    return
  end

  -- High altitude branch:
  -- AltHold: always Baro
  -- Loiter: GPS height only when GPS quality is good and GPS is active nav source
  if current_mode == COPTER_MODE_ALT_HOLD then
    set_src12_posz(EK3_POSZ_BARO)
  elseif current_mode == COPTER_MODE_LOITER and gps_ok and gps_nav_active then
    set_src12_posz(EK3_POSZ_GPS)
  else
    set_src12_posz(EK3_POSZ_BARO)
  end
end

-- Check GPS quality for voting
-- Returns true if GPS meets all thresholds
local function gps_quality_ok(gps_speed_acc_thresh, gps_pos_acc_thresh, gps_min_sats)
  local primary = gps:primary_sensor()

  -- Must have at least 3D fix
  if gps:status(primary) < gps.GPS_OK_FIX_3D then
    return false
  end

  -- Satellite count
  if gps:num_sats(primary) < gps_min_sats then
    return false
  end

  -- Speed accuracy
  local ok_sa, speed_acc = gps:speed_accuracy(primary)
  if (not ok_sa) or (speed_acc == nil) or (speed_acc > gps_speed_acc_thresh) then
    return false
  end

  -- Position accuracy = sqrt(hacc^2 + vacc^2)
  local ok_ha, h_acc = gps:horizontal_accuracy(primary)
  local ok_va, v_acc = gps:vertical_accuracy(primary)
  if (ok_ha and h_acc) and (ok_va and v_acc) then
    local pos_acc = math.sqrt(h_acc * h_acc + v_acc * v_acc)
    if pos_acc > gps_pos_acc_thresh then
      return false
    end
  else
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
    -- MAV_SEVERITY_WARNING (4): default Mission Planner / QGC filters often hide INFO (6)
    gcs:send_text(4, "flymode_ek3src: script running")
    notify:send_text("ek3src run", 0)
  end

  -- Read tuning parameters
  local gps_speed_acc_thresh  = get_param(scr_user1, 0.5)
  local gps_pos_acc_thresh     = get_param(scr_user2, 2.0)
  local gps_min_sats          = get_param(scr_user3, 8)
  local flow_quality_thresh   = get_param(scr_user4, 50)
  local flow_innov_thresh     = get_param(scr_user5, 0.15)

  local current_mode = vehicle:get_mode()
  local mode_changed, old_mode = detect_user_mode_change(current_mode)

  -- Detect manual mode transitions:
  -- If user manually switches TO Loiter, engage auto logic
  -- If user manually switches FROM Loiter to AltHold, disengage auto logic
  -- If user manually switches to AltHold (not from our auto logic), use Source3 fallback
  if mode_changed then
    if current_mode == COPTER_MODE_LOITER then
      -- User selected Loiter -> engage auto switching
      auto_mode_engaged = true
      gcs:send_text(0, "Auto mode ENGAGED (user set Loiter)")
    elseif current_mode == COPTER_MODE_ALT_HOLD then
      -- User manually selected AltHold -> disengage auto, use Source3
      auto_mode_engaged = false
      enter_althold_fallback_nav()
      gcs:send_text(0, "Auto mode DISENGAGED (user set AltHold, EK3_SRC3+Src3)")
      return update, UPDATE_INTERVAL_MS
    else
      -- User switched to some other mode -> disengage auto
      auto_mode_engaged = false
      return update, UPDATE_INTERVAL_MS
    end
  end

  -- If not in AltHold or Loiter, do nothing
  if (current_mode ~= COPTER_MODE_ALT_HOLD) and (current_mode ~= COPTER_MODE_LOITER) then
    return update, UPDATE_INTERVAL_MS
  end

  -- Diagnostics (compute whenever in AltHold/Loiter so periodic logs still run if auto is off)
  local rf_alt_ok = rangefinder_alt_usable()
  local gps_ok = gps_quality_ok(gps_speed_acc_thresh, gps_pos_acc_thresh, gps_min_sats)
  local flow_ok = opticalflow_quality_ok(flow_quality_thresh, flow_innov_thresh)
  local flow_and_rf_ok = flow_ok and rf_alt_ok

  local rngfnd_dist_m = 0
  local rngfnd_max_m = 0
  local rngfnd_has_data = false
  if rangefinder:has_data_orient(RANGEFINDER_ROTATION) then
    rngfnd_has_data = true
    rngfnd_dist_m = rangefinder:distance_cm_orient(RANGEFINDER_ROTATION) * 0.01
    rngfnd_max_m = rangefinder:max_distance_cm_orient(RANGEFINDER_ROTATION) * 0.01
  end

  -- md §1: in Loiter at high altitude, GPS height requires GPS-quality and active GPS nav source.
  local gps_nav_active = (source_prev == 0)
  update_height_source_with_hysteresis(current_mode, gps_ok, gps_nav_active, rngfnd_has_data, rngfnd_dist_m, rngfnd_max_m)

  -- Periodic telemetry: severity 6 (INFO) is often hidden in MP/QGC; use 5 (NOTICE).
  if (now_ms - last_log_ms) >= LOG_INTERVAL_MS then
    last_log_ms = now_ms

    local primary = gps:primary_sensor()
    local ok_sa, s_acc = gps:speed_accuracy(primary)
    local ok_ha, h_acc = gps:horizontal_accuracy(primary)
    local ok_va, v_acc = gps:vertical_accuracy(primary)
    local n_sats = gps:num_sats(primary)
    local pos_acc_str = "nil"
    if (ok_ha and h_acc) and (ok_va and v_acc) then
      pos_acc_str = string.format("%.2f", math.sqrt(h_acc * h_acc + v_acc * v_acc))
    end

    local flow_q = 0
    if optical_flow then
      flow_q = optical_flow:quality()
    end

    -- gcs:send_text(4, string.format(
    --   "ek3as Src:%u Auto:%u Rng:%.1f/%.1f RfAlt:%s GPS:%s Flow:%s V:%d",
    --   source_prev + 1,
    --   auto_mode_engaged and 1 or 0,
    --   rngfnd_dist_m,
    --   rngfnd_max_m * RF_ALT_RATIO,
    --   rf_alt_ok and "OK" or "NO",
    --   gps_ok and "OK" or "BAD",
    --   flow_and_rf_ok and "OK" or "BAD",
    --   gps_vs_flow_vote
    -- ))
    -- gcs:send_text(4, string.format(
    --   "ek3as Sats:%u SAcc:%s PAcc:%s FlowQ:%u",
    --   n_sats,
    --   ok_sa and s_acc and string.format("%.2f", s_acc) or "nil",
    --   pos_acc_str,
    --   flow_q
    -- ))
    -- 在这里打印光流的创新值
    if optical_flow then
      local innov, innov_var = ahrs:get_vel_innovations_and_variances_for_source(5)
      if innov then
        local xy_innov = math.sqrt(innov:x() * innov:x() + innov:y() * innov:y())
        gcs:send_text(4, string.format("OptFlow Innovation: %.4f", xy_innov))
      else
        gcs:send_text(4, "OptFlow Innovation: nil")
      end
    else
      gcs:send_text(4, "OptFlow Not Available")
    end
  end

  -- Only run voting / mode changes when user has engaged auto (switched to Loiter once)
  if not auto_mode_engaged then
    bad_nav_streak = 0
    return update, UPDATE_INTERVAL_MS
  end

  -- md §2/§4: when both GPS and Flow+Rangefinder meet thresholds, always prefer SRC2 (flow)
  if current_mode == COPTER_MODE_LOITER and flow_and_rf_ok and gps_ok then
    set_source(1)
    gps_vs_flow_vote = VOTE_COUNTER_MAX
  end

  -- When coming from manual AltHold fallback (SRC3), don't stay on SRC3 in auto Loiter path.
  -- This also serves as an emergency escape: if voting can't accumulate because both sensors
  -- were previously judged BAD, force a switch now using strict quality gates.
  if current_mode == COPTER_MODE_LOITER and source_prev == 2 then
    if flow_and_rf_ok then
      set_source(1)
    elseif gps_ok then
      set_source(0)
    end
  end

  -- md §7: GPS source active but GPS degrades -> prefer flow when available
  if current_mode == COPTER_MODE_LOITER and source_prev == 0 and (not gps_ok) and flow_and_rf_ok then
    set_source(1)
    -- Bias vote toward flow so we don't bounce back to GPS on the next frame.
    gps_vs_flow_vote = math.min(gps_vs_flow_vote + 2, VOTE_COUNTER_MAX)
  end

  -- === AUTO MODE LOGIC (works disarmed on bench for tuning; dwell reduces chatter) ===
  -- Vote: if both are good, flow branch runs first (md §2 dual-good -> flow). Dual-good also
  -- forces SRC2 above; this updates vote for when only one sensor is good.
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
      -- don't hard-reset on one good sample, otherwise noisy transitions can delay degrade indefinitely
      bad_nav_streak = math.max(bad_nav_streak - 1, 0)
    end

    if bad_nav_streak >= DEGRADE_DWELL_TICKS then
      if vehicle:set_mode(COPTER_MODE_ALT_HOLD) then
        enter_althold_fallback_nav()
        gcs:send_text(0, "Auto -> AltHold (GPS+Flow bad, EK3_SRC3+Src3)")
        prev_user_mode = COPTER_MODE_ALT_HOLD
      end
      bad_nav_streak = 0
    end

    if voted_source >= 0 then
      set_source(voted_source)
      gps_nav_active = (source_prev == 0)
      update_height_source_with_hysteresis(current_mode, gps_ok, gps_nav_active, rngfnd_has_data, rngfnd_dist_m, rngfnd_max_m)
    end
  end

  if current_mode == COPTER_MODE_ALT_HOLD then
    -- Requirement:
    -- In AltHold, if Flow/GPS is available, switch EKF source first and keep
    -- trying to enter Loiter. If both become unavailable, stay in AltHold SRC3.
    local desired_source = -1
    if flow_and_rf_ok then
      desired_source = 1
    elseif gps_ok then
      desired_source = 0
    end

    if desired_source < 0 then
      enter_althold_fallback_nav()
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
          gcs:send_text(0, "Auto -> Loiter (source " .. (desired_source == 0 and "GPS" or "Flow") .. " ready)")
          prev_user_mode = COPTER_MODE_LOITER
        end
      end
    end
  end

  return update, UPDATE_INTERVAL_MS
end

return update()
