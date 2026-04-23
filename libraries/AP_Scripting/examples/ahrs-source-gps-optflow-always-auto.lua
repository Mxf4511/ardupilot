-- automatically switches AHRS/EKF sources between GPS and optical flow
-- this variant does not use RCx_OPTION=90 or RCx_OPTION=300
-- and keeps auto source selection logic always enabled
--
-- configure a forward or downward facing lidar with a range of at least 5m
-- SCR_ENABLE = 1 (enable scripting)
-- setup EK3_SRCn_ parameters so that GPS is the primary source, opticalflow is secondary.
--     EK3_SRC1_POSXY = 3 (GPS)
--     EK3_SRC1_VELXY = 3 (GPS)
--     EK3_SRC1_VELZ  = 3 (GPS)
--     EK3_SRC1_POSZ  = 2 (RangeFinder)
--     EK3_SRC1_YAW   = 1 (Compass)
--     EK3_SRC2_POSXY = 0 (None)
--     EK3_SRC2_POSZ  = 1 (Baro)
--     EK3_SRC2_VELXY = 5 (OpticalFlow)
--     EK3_SRC2_VELZ  = 0 (None)
--     EK3_SRC2_YAW   = 1 (Compass)
--     EK3_SRC_OPTIONS = 0 (Do not fuse all velocities)
--
-- SCR_USER1 holds the threshold (in meters) for rangefinder altitude (around 15 is a good choice)
--     if rangefinder distance >= SCR_USER1, source1 (GPS) will be used
--     if rangefinder distance < SCR_USER1, source2 (optical flow) will be used if innovations are below SCR_USER4 value
-- SCR_USER2 holds the threshold for GPS speed accuracy (around 0.3 is a good choice)
-- SCR_USER3 holds the threshold for optical flow quality (about 50 is a good choice)
-- SCR_USER4 holds the threshold for optical flow innovations (about 0.15 is a good choice)
--
-- luacheck: only 0
---@diagnostic disable: cast-local-type
---@diagnostic disable: need-check-nil

local rangefinder_rotation = 25     -- check downward (25) facing lidar
local source_prev = ahrs:get_posvelyaw_source_set()
local gps_usable_accuracy = 1.0     -- GPS is usable if speed accuracy is at or below this value
local opticalflow_innov_thresh_loose = 8.0 -- relaxed re-entry threshold when currently on GPS
local vote_counter_max = 20         -- when a vote counter reaches this number (i.e. 2sec) source may be switched
local gps_vs_opticalflow_vote = 0   -- vote counter for GPS vs optical (-20 = GPS, +20 = optical flow)
local param_log_interval_ms = 2000  -- parameter print period
local last_param_log_ms = 0
local copter_mode_althold = 2
local copter_mode_loiter = 5

-- initialise parameters
local scr_user1_param = Parameter('SCR_USER1') -- user1 param (rangefinder altitude threshold)
local scr_user2_param = Parameter('SCR_USER2') -- user2 param (GPS speed accuracy threshold)
local scr_user3_param = Parameter('SCR_USER3') -- user3 param (optical flow quality threshold)
local scr_user4_param = Parameter('SCR_USER4') -- user4 param (optical flow innovation threshold)

assert(optical_flow, 'could not access optical flow')
assert(rangefinder, 'could not access rangefinder')

-- play tune on buzzer to alert user to change in active source set
function play_source_tune(source)
  if (source) then
    if (source == 0) then
      notify:play_tune("L8C")       -- one long lower tone
    elseif (source == 1) then
      notify:play_tune("L12DD")     -- two fast medium tones
    elseif (source == 2) then
      notify:play_tune("L16FFF")    -- three very fast, high tones
    end
  end
end

local function set_source(source)
  if source ~= source_prev then
    source_prev = source
    ahrs:set_posvelyaw_source_set(source_prev)
    gcs:send_text(0, "Auto switched to Source " .. string.format("%d", source_prev + 1))
    play_source_tune(source_prev)
  end
end

-- the main update function
function update()
  local now_ms = millis()

  -- check rangefinder distance threshold has been set
  local rangefinder_thresh_dist = scr_user1_param:get()     -- SCR_USER1 holds rangefinder threshold
  if (rangefinder_thresh_dist <= 0) then
    gcs:send_text(0, "ahrs-source-gps-optflow-always-auto.lua: set SCR_USER1 to rangefinder threshold")
    return update, 1000
  end

  -- check GPS speed accuracy threshold has been set
  local gps_speedaccuracy_thresh = scr_user2_param:get()    -- SCR_USER2 holds GPS speed accuracy threshold
  if (gps_speedaccuracy_thresh <= 0) then
    gcs:send_text(0, "ahrs-source-gps-optflow-always-auto.lua: set SCR_USER2 to GPS speed accuracy threshold")
    return update, 1000
  end

  -- check optical flow quality threshold has been set
  local opticalflow_quality_thresh = scr_user3_param:get()  -- SCR_USER3 holds opticalflow quality
  if (opticalflow_quality_thresh <= 0) then
    gcs:send_text(0, "ahrs-source-gps-optflow-always-auto.lua: set SCR_USER3 to OpticalFlow quality threshold")
    return update, 1000
  end

  -- check optical flow innovation threshold has been set
  local opticalflow_innov_thresh = scr_user4_param:get()    -- SCR_USER4 holds opticalflow innovation
  if (opticalflow_innov_thresh <= 0) then
    gcs:send_text(0, "ahrs-source-gps-optflow-always-auto.lua: set SCR_USER4 to OpticalFlow innovation threshold")
    return update, 1000
  end

  -- check if GPS speed accuracy is over threshold
  local gps_speed_accuracy = gps:speed_accuracy(gps:primary_sensor())
  local gps_over_threshold = (gps_speed_accuracy == nil) or (gps:speed_accuracy(gps:primary_sensor()) > gps_speedaccuracy_thresh)
  local gps_usable = (gps_speed_accuracy ~= nil) and (gps_speed_accuracy <= gps_usable_accuracy)

  -- check optical flow quality
  local opticalflow_quality = 0
  local opticalflow_quality_good = false
  if (optical_flow) then
    opticalflow_quality = optical_flow:quality()
    opticalflow_quality_good = (optical_flow:enabled() and optical_flow:healthy() and opticalflow_quality >= opticalflow_quality_thresh)
  end

  -- get opticalflow innovations from ahrs (only x and y values are valid)
  local opticalflow_over_threshold = true
  local opticalflow_innov_thresh_active = opticalflow_innov_thresh
  local opticalflow_xy_innov = 0
  local opticalflow_innov = Vector3f()
  local opticalflow_var = Vector3f()
  if (source_prev == 0) then
    opticalflow_innov_thresh_active = opticalflow_innov_thresh_loose
  end
  opticalflow_innov, opticalflow_var = ahrs:get_vel_innovations_and_variances_for_source(5)
  if (opticalflow_innov) then
    opticalflow_xy_innov = math.sqrt(opticalflow_innov:x() * opticalflow_innov:x() + opticalflow_innov:y() * opticalflow_innov:y())
    opticalflow_over_threshold = (opticalflow_xy_innov == 0.0) or (opticalflow_xy_innov > opticalflow_innov_thresh_active)
  end

  -- get rangefinder distance (4.6 API: distance_cm_orient)
  local rngfnd_distance_m = 0
  if rangefinder:has_data_orient(rangefinder_rotation) then
    rngfnd_distance_m = rangefinder:distance_cm_orient(rangefinder_rotation) * 0.01
  end
  local rngfnd_over_threshold = (rngfnd_distance_m == 0) or (rngfnd_distance_m > rangefinder_thresh_dist)

  -- opticalflow is usable if quality and innovations are good and rangefinder is in range
  local opticalflow_usable = opticalflow_quality_good and (not opticalflow_over_threshold) and (not rngfnd_over_threshold)

  -- GPS vs opticalflow vote. "-1" to move towards GPS, "+1" to move to opticalflow
  if (not gps_over_threshold) then
    -- only vote for GPS when GPS speed accuracy passes SCR_USER2
    gps_vs_opticalflow_vote = math.max(gps_vs_opticalflow_vote - 1, -vote_counter_max)
  elseif opticalflow_usable then
    -- vote for opticalflow if usable
    gps_vs_opticalflow_vote = math.min(gps_vs_opticalflow_vote + 1, vote_counter_max)
  end

  -- auto source vote collation
  local auto_source = -1                         -- auto source undecided if -1
  if gps_vs_opticalflow_vote <= -vote_counter_max then
    auto_source = 0                              -- GPS
  elseif gps_vs_opticalflow_vote >= vote_counter_max then
    auto_source = 1                              -- opticalflow
  end

  local current_mode = vehicle:get_mode()
  local mode_is_loiter = (current_mode == copter_mode_loiter)
  local mode_is_althold = (current_mode == copter_mode_althold)
  local mode_managed = mode_is_loiter or mode_is_althold

  -- if both navigation sources are bad, drop to AltHold
  if mode_is_loiter and gps_over_threshold and (not opticalflow_usable) then
    if vehicle:set_mode(copter_mode_althold) then
      gcs:send_text(0, "Auto switched to AltHold: GPS and optical flow unusable")
      current_mode = copter_mode_althold
      mode_is_loiter = false
      mode_is_althold = true
    end
  end

  -- recover back to Loiter only after a source has passed voting
  if mode_is_althold and (auto_source >= 0) then
    set_source(auto_source)
    if vehicle:set_mode(copter_mode_loiter) then
      gcs:send_text(0, "Auto switched to Loiter")
      current_mode = copter_mode_loiter
      mode_is_loiter = true
      mode_is_althold = false
    end
  elseif mode_managed and (auto_source >= 0) then
    -- when staying in Loiter, still keep the active source aligned with the vote
    set_source(auto_source)
  end

  if (now_ms - last_param_log_ms) >= param_log_interval_ms then
    last_param_log_ms = now_ms
    gcs:send_text(
      6,
      string.format(
        "\nSrc:%u Rng:%.2f/%.2f GPSsAcc:%s/%.2f FlowQ:%u/%.2f FlowInnv:%.2f/%.2f",
        source_prev + 1,
        rngfnd_distance_m,
        rangefinder_thresh_dist,
        gps_speed_accuracy and string.format("%.2f", gps_speed_accuracy) or "nil",
        gps_speedaccuracy_thresh,
        opticalflow_quality,
        opticalflow_quality_thresh,
        opticalflow_xy_innov,
        opticalflow_innov_thresh_active
      )
    )
  end

  return update, 100
end

return update()
