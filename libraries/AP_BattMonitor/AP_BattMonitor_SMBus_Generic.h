#pragma once

#include "AP_BattMonitor_SMBus.h"

#if AP_BATTERY_SMBUS_GENERIC_ENABLED

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define BATTMONITOR_SMBUS_NUM_CELLS_MAX 14
#else
#define BATTMONITOR_SMBUS_NUM_CELLS_MAX 12
#endif

class AP_BattMonitor_SMBus_Generic : public AP_BattMonitor_SMBus
{
public:

    // Constructor
    AP_BattMonitor_SMBus_Generic(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params);

    // returns true if battery monitor instance provides time remaining info
    bool has_time_remaining() const override { return true; }

    // write BTR debug log message
    void Log_Write_BTR(const uint8_t instance, const uint64_t time_us) const override;

private:

    void timer(void) override;

    // override capacity scaler to use the _scaler parameter
    uint16_t get_capacity_scaler() const override { return _scaler.get(); }

    // check if PEC supported with the version value in SpecificationInfo() function
    // returns true once PEC has been confirmed as working or not working
    bool check_pec_support();

    // update time remaining estimation using LTD filter and dual-branch fusion
    void update_time_remaining();

    // reset flight state (called when landing detected)
    void reset_flight_state();

    uint8_t _pec_confirmed; // count of the number of times PEC has been confirmed as working
    uint32_t _last_cell_update_us[BATTMONITOR_SMBUS_NUM_CELLS_MAX]; // system time of last successful read of cell voltage
    uint32_t _cell_count_check_start_us;  // system time we started attempting to count the number of cells
    uint8_t _cell_count;    // number of cells returning voltages
    bool _cell_count_fixed; // true when cell count check is complete

    // --- Time remaining estimation state ---

    // LTD (Linear Tracking Differentiator) filter state
    float _ltd_v1;                          // LTD tracking output (filtered current, A)
    float _ltd_v2;                          // LTD tracking derivative
    bool _ltd_initialized;                  // true after first LTD update
    bool _ltd_converged;                    // true when v1 has converged to actual current
    uint32_t _last_ltd_update_us;           // timestamp of last LTD update (microseconds)

    // Flight state tracking
    bool _in_flight;                        // true when in flight (current > threshold)
    uint32_t _takeoff_time_ms;              // time of takeoff detection (milliseconds)
    float _takeoff_consumed_mah;            // consumed_mah at takeoff
    float _takeoff_remaining_mah;           // remaining capacity (mAh) at takeoff
    uint32_t _low_current_start_ms;         // time when current dropped below landing threshold

    // Branch 1 (experience-based) state
    bool _branch1_active;                   // true after 3 minutes of flight
    uint32_t _branch1_activate_time_ms;     // time when branch 1 was activated
    float _branch1_prev_estimate;           // previous branch 1 estimate for smooth transition

    // Debug data for logging
    float _dbg_raw_current;                 // current before scaling (A)
    float _dbg_branch1_est;                 // branch 1 time estimate (sec)
    float _dbg_branch2_est;                 // branch 2 time estimate (sec)
    float _dbg_w1;                          // weight for branch 1
    float _dbg_remaining_cap_mah;           // remaining capacity (mAh)

    // Constants
    static constexpr float TAKEOFF_CURRENT_THRESH_A = 2.5f;   // current threshold for takeoff detection (A)
    static constexpr float LANDING_CURRENT_THRESH_A = 2.5f;   // current threshold for landing detection (A)
    static constexpr uint32_t LANDING_TIMEOUT_MS = 3000;       // current must stay below threshold for this duration to declare landing (ms)
    static constexpr uint32_t BRANCH1_DELAY_MS = 40000;       // branch 1 activates after 1.5 minutes (ms)
    static constexpr uint32_t BRANCH1_RAMP_MS = 30000;         // branch 1 weight ramp-in period (ms)
    static constexpr float BRANCH1_FULL_WEIGHT_MAH_FRAC = 0.1f; // branch 1 reaches full weight after consuming this fraction of pack capacity
    static constexpr float LTD_CONVERGE_THRESH_A = 0.5f;       // LTD convergence threshold (A)
};

#endif  // AP_BATTERY_SMBUS_GENERIC_ENABLED
