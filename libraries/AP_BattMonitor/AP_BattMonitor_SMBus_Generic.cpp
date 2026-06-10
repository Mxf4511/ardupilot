#include "AP_BattMonitor_config.h"

#if AP_BATTERY_SMBUS_GENERIC_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include "LogStructure.h"
#include "AP_BattMonitor.h"

#include "AP_BattMonitor_SMBus_Generic.h"

uint8_t smbus_cell_ids[] = { 0x3f,  // cell 1
                             0x3e,  // cell 2
                             0x3d,  // cell 3
                             0x3c,  // cell 4
                             0x3b,  // cell 5
                             0x3a,  // cell 6
                             0x39,  // cell 7
                             0x38,  // cell 8
                             0x37,  // cell 9
                             0x36,  // cell 10
                             0x35,  // cell 11
                             0x34,  // cell 12
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                             0x33,  // cell 13
                             0x32   // cell 14
#endif
};

#define SMBUS_CELL_COUNT_CHECK_TIMEOUT       15     // check cell count for up to 15 seconds

/*
 * Other potentially useful registers, listed here for future use
 * #define BATTMONITOR_SMBUS_MAXELL_CHARGE_STATUS         0x0d    // relative state of charge
 * #define BATTMONITOR_SMBUS_MAXELL_BATTERY_STATUS        0x16    // battery status register including alarms
 * #define BATTMONITOR_SMBUS_MAXELL_BATTERY_CYCLE_COUNT   0x17    // cycle count
 * #define BATTMONITOR_SMBUS_MAXELL_DESIGN_VOLTAGE        0x19    // design voltage register
 * #define BATTMONITOR_SMBUS_MAXELL_MANUFACTURE_DATE      0x1b    // manufacturer date
 * #define BATTMONITOR_SMBUS_MAXELL_SERIALNUM             0x1c    // serial number register
 * #define BATTMONITOR_SMBUS_MAXELL_HEALTH_STATUS         0x4f    // state of health
 * #define BATTMONITOR_SMBUS_MAXELL_SAFETY_ALERT          0x50    // safety alert
 * #define BATTMONITOR_SMBUS_MAXELL_SAFETY_STATUS         0x51    // safety status
 * #define BATTMONITOR_SMBUS_MAXELL_PF_ALERT              0x52    // safety status
 * #define BATTMONITOR_SMBUS_MAXELL_PF_STATUS             0x53    // safety status
*/

// Constructor
AP_BattMonitor_SMBus_Generic::AP_BattMonitor_SMBus_Generic(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params)
    : AP_BattMonitor_SMBus(mon, mon_state, params, AP_BATTMONITOR_SMBUS_BUS_EXTERNAL),
      _ltd_v1(0),
      _ltd_v2(0),
      _ltd_initialized(false),
      _ltd_converged(false),
      _last_ltd_update_us(0),
      _in_flight(false),
      _takeoff_time_ms(0),
      _takeoff_consumed_mah(0),
      _takeoff_remaining_mah(0),
      _low_current_start_ms(0),
      _branch1_active(false),
      _branch1_activate_time_ms(0),
      _branch1_prev_estimate(0),
      _dbg_raw_current(0),
      _dbg_branch1_est(0),
      _dbg_branch2_est(0),
      _dbg_w1(0),
      _dbg_remaining_cap_mah(0)
{}

void AP_BattMonitor_SMBus_Generic::timer()
{
	// check if PEC is supported
    if (!check_pec_support()) {
        return;
    }

    uint16_t data;
    uint32_t tnow = AP_HAL::micros();

    // read voltage (V)
    if (read_word(BATTMONITOR_SMBUS_VOLTAGE, data)) {
        _state.voltage = (float)data * 0.001f;
        _state.last_time_micros = tnow;
        _state.healthy = true;
    }

    // assert that BATTMONITOR_SMBUS_NUM_CELLS_MAX must be no more than smbus_cell_ids
    static_assert(BATTMONITOR_SMBUS_NUM_CELLS_MAX <= ARRAY_SIZE(smbus_cell_ids), "BATTMONITOR_SMBUS_NUM_CELLS_MAX must be no more than smbus_cell_ids");

    // check cell count
    if (!_cell_count_fixed) {
        if (_state.healthy) {
            // when battery first becomes healthy, start check of cell count
            if (_cell_count_check_start_us == 0) {
                _cell_count_check_start_us = tnow;
            }
            if (tnow - _cell_count_check_start_us > (SMBUS_CELL_COUNT_CHECK_TIMEOUT * 1e6)) {
                // give up checking cell count after 15sec of continuous healthy battery reads
                _cell_count_fixed = true;
            }
        } else {
            // if battery becomes unhealthy restart cell count check
            _cell_count_check_start_us = 0;
        }
    }

    // we loop over something limited by
    // BATTMONITOR_SMBUS_NUM_CELLS_MAX but assign into something
    // limited by AP_BATT_MONITOR_CELLS_MAX - so make sure we won't
    // over-write:
    static_assert(BATTMONITOR_SMBUS_NUM_CELLS_MAX <= ARRAY_SIZE(_state.cell_voltages.cells), "BATTMONITOR_SMBUS_NUM_CELLS_MAX must be <= number of cells in state voltages");

    // read cell voltages
    for (uint8_t i = 0; i < (_cell_count_fixed ? _cell_count : BATTMONITOR_SMBUS_NUM_CELLS_MAX); i++) {
        if (read_word(smbus_cell_ids[i], data) && (data > 0) && (data < UINT16_MAX)) {
            _has_cell_voltages = true;
            _state.cell_voltages.cells[i] = data;
            _last_cell_update_us[i] = tnow;
            if (!_cell_count_fixed) {
                _cell_count = MAX(_cell_count, i + 1);
            }
        } else if ((tnow - _last_cell_update_us[i]) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
            _state.cell_voltages.cells[i] = UINT16_MAX;
        }
    }

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
        return;
    }

    // read current (A)
    if (read_word(BATTMONITOR_SMBUS_CURRENT, data)) {
        _dbg_raw_current = -(float)((int16_t)data) * 0.001f;  // store raw value before scaling
        _state.current_amps = _dbg_raw_current * get_current_scaler();
        _state.last_time_micros = tnow;
    }

    read_full_charge_capacity();

    // FIXME: Perform current integration if the remaining capacity can't be requested
    read_remaining_capacity();

    // update time remaining estimation
    update_time_remaining();

    // print remaining time every 1 second
    if (_state.has_time_remaining) {
        static uint32_t _last_print_ms = 0;
        const uint32_t now_ms = AP_HAL::millis();
        if (now_ms - _last_print_ms >= 1000) {
            _last_print_ms = now_ms;
            const uint32_t rem_sec = _state.time_remaining;
            const uint32_t rem_min = rem_sec / 60;
            const uint32_t rem_sec_mod = rem_sec % 60;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Remaining: %u:%02u", (unsigned)rem_min, (unsigned)rem_sec_mod);
        }
    }

    read_temp();

    read_serial_number();

    read_cycle_count();
}

// reset flight state when landing is detected
void AP_BattMonitor_SMBus_Generic::reset_flight_state()
{
    _in_flight = false;
    _ltd_initialized = false;
    _ltd_converged = false;
    _ltd_v1 = 0;
    _ltd_v2 = 0;
    _branch1_active = false;
    _branch1_prev_estimate = 0;
    _state.has_time_remaining = false;
    _state.time_remaining = 0;
    _low_current_start_ms = 0;
}

// update time remaining estimation using LTD filter and dual-branch fusion
void AP_BattMonitor_SMBus_Generic::update_time_remaining()
{
    if (!_state.healthy || _params._pack_capacity <= 0) {
        _state.has_time_remaining = false;
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    const float abs_current = fabsf(_state.current_amps);
    const uint32_t pack_capacity = _params._pack_capacity;

    // --- Landing / Takeoff detection ---
    if (!_in_flight) {
        // on ground: wait for current to exceed takeoff threshold
        if (abs_current > TAKEOFF_CURRENT_THRESH_A) {
            _in_flight = true;
            _takeoff_time_ms = now_ms;
            _takeoff_consumed_mah = _state.consumed_mah;
            _takeoff_remaining_mah = (float)pack_capacity - _state.consumed_mah;
            _low_current_start_ms = 0;
            // initialize LTD filter with current value to speed up convergence
            _ltd_v1 = abs_current;
            _ltd_v2 = 0;
            _ltd_initialized = true;
            _last_ltd_update_us = AP_HAL::micros();
        }
        _state.has_time_remaining = false;
        return;
    }

    // check for landing: current below threshold for LANDING_TIMEOUT_MS
    if (abs_current < LANDING_CURRENT_THRESH_A) {
        if (_low_current_start_ms == 0) {
            _low_current_start_ms = now_ms;
        }
        if ((now_ms - _low_current_start_ms) > LANDING_TIMEOUT_MS) {
            reset_flight_state();
            return;
        }
    } else {
        _low_current_start_ms = 0;
    }

    // --- LTD Filter Update ---
    const uint32_t now_us = AP_HAL::micros();
    float h = (now_us - _last_ltd_update_us) * 1e-6f;  // time step in seconds
    _last_ltd_update_us = now_us;

    // clamp h to reasonable range (1ms - 1s)
    h = constrain_float(h, 0.001f, 1.0f);

    const float r = (float)_ltd_r.get();
    const float sqrt_r = sqrtf(r);

    // LTD update equations
    float fh = -r * (_ltd_v1 - abs_current) - 2.0f * sqrt_r * _ltd_v2;
    _ltd_v1 = _ltd_v1 + h * _ltd_v2;
    _ltd_v2 = _ltd_v2 + h * fh;

    // after takeoff, wait until LTD v1 is close to the sampled current
    // before allowing branch2 time remaining estimation
    if (!_ltd_converged) {
        if (fabsf(_ltd_v1 - abs_current) < LTD_CONVERGE_THRESH_A) {
            _ltd_converged = true;
        }
    }

    // don't output time remaining until LTD has converged
    if (!_ltd_converged) {
        _state.has_time_remaining = false;
        return;
    }

    // --- Remaining capacity ---
    const float remaining_cap_mah = (float)pack_capacity - _state.consumed_mah;
    _dbg_remaining_cap_mah = remaining_cap_mah;

    if (remaining_cap_mah <= 0) {
        _state.time_remaining = 0;
        return;
    }

    // --- Branch 2: LTD filtered current ---
    float branch2_est = 0;
    if (_ltd_v1 > 0.1f) {  // avoid division by near-zero
        branch2_est = (remaining_cap_mah / _ltd_v1) * (1.0f / 60.0f) * 3600.0f;  // mAh / A * 3600/1 = seconds
        // simplify: (mAh / A) = (mAh / A) * 3600 / 1000 ... wait
        // remaining_cap_mah is in mAh, _ltd_v1 is in A
        // time = (mAh / 1000) / A * 3600 = mAh / A * 3.6
        branch2_est = remaining_cap_mah / _ltd_v1 * 3.6f;
    }
    _dbg_branch2_est = branch2_est;

    // --- Branch 1: experience-based estimation (activates after 3 minutes) ---
    float branch1_est = 0;
    const uint32_t flight_time_ms = now_ms - _takeoff_time_ms;
    const float flight_mah = _state.consumed_mah - _takeoff_consumed_mah;

    // check if branch 1 should activate (after 3 min of flight)
    if (!_branch1_active && flight_time_ms > BRANCH1_DELAY_MS && flight_mah > 0) {
        _branch1_active = true;
        _branch1_activate_time_ms = now_ms;
    }

    if (_branch1_active && flight_mah > 0) {
        const float flight_time_sec = flight_time_ms * 0.001f;
        // branch1: remain = flight_time * (remaining_mah / flight_mah)
        const float remaining_mah = _takeoff_remaining_mah - flight_mah;
        if (remaining_mah > 0) {
            branch1_est = flight_time_sec * remaining_mah / flight_mah;
        }
        if (branch1_est < 0) {
            branch1_est = 0;
        }
    }
    _dbg_branch1_est = branch1_est;


    /* 方法1或方法2任意一个有效,则认为有剩余时间 */
    if(_ltd_converged == true || flight_time_ms > BRANCH1_DELAY_MS)
    {
        _state.has_time_remaining = true;
    }


    // --- Dual-branch fusion with smooth weighting ---
    float w1 = 0;
    float w2 = 1.0f;
    float final_time_remaining = branch2_est;

    if (_branch1_active) {
        // compute target weight based on how much capacity has been consumed
        float target_w1 = constrain_float(flight_mah / (BRANCH1_FULL_WEIGHT_MAH_FRAC * (float)pack_capacity), 0.0f, 1.0f);

        // smooth ramp-in over BRANCH1_RAMP_MS to avoid data jump
        const uint32_t ramp_elapsed = now_ms - _branch1_activate_time_ms;
        if (ramp_elapsed < BRANCH1_RAMP_MS) {
            float ramp_factor = (float)ramp_elapsed / (float)BRANCH1_RAMP_MS;
            w1 = target_w1 * ramp_factor;
        } else {
            w1 = target_w1;
        }
        w2 = 1.0f - w1;

        final_time_remaining = w1 * branch1_est + w2 * branch2_est;

        // smooth transition: if this is the first frame with branch1 active,
        // ensure the output doesn't jump by blending with previous estimate
        if (_branch1_prev_estimate > 0) {
            // exponential smoothing to prevent discontinuity
            final_time_remaining = 0.5f * final_time_remaining + 0.5f * _branch1_prev_estimate;
        }

        /* 最后1s直接清零 */
        if(final_time_remaining <= 1)
        {
            final_time_remaining = 0;
        }
    }
    _branch1_prev_estimate = final_time_remaining;

    _dbg_w1 = w1;
    _dbg_raw_current = abs_current;  // reuse field for debug: store abs current

    _state.time_remaining = (uint32_t)MAX(0, final_time_remaining);
    _state.has_time_remaining = true;
}

// check if PEC supported with the version value in SpecificationInfo() function
// returns true once PEC is confirmed as working or not working
bool AP_BattMonitor_SMBus_Generic::check_pec_support()
{
    // exit immediately if we have already confirmed pec support
    if (_pec_confirmed) {
        return true;
    }

    // specification info
    uint16_t data;
    if (!read_word(BATTMONITOR_SMBUS_SPECIFICATION_INFO, data)) {
        return false;
    }

    // extract version
    uint8_t version = (data & 0xF0) >> 4;

    // version less than 0011b (i.e. 3) do not support PEC
    if (version < 3) {
        _pec_supported = false;
        _pec_confirmed = true;
        return true;
    }

    // check manufacturer name
    uint8_t buff[AP_BATTMONITOR_SMBUS_READ_BLOCK_MAXIMUM_TRANSFER + 1] {};
    if (read_block(BATTMONITOR_SMBUS_MANUFACTURE_NAME, buff, sizeof(buff))) {
        // Hitachi maxell batteries do not support PEC
        if (strcmp((char*)buff, "Hitachi maxell") == 0) {
            _pec_supported = false;
            _pec_confirmed = true;
            return true;
        }
    }

    // assume all other batteries support PEC
	_pec_supported = true;
	_pec_confirmed = true;
	return true;
}

#if HAL_LOGGING_ENABLED
// write BTR debug log message
void AP_BattMonitor_SMBus_Generic::Log_Write_BTR(const uint8_t instance, const uint64_t time_us) const
{
    const struct log_BTR pkt{
        LOG_PACKET_HEADER_INIT(LOG_BTR_MSG),
        time_us         : time_us,
        instance        : instance,
        raw_current     : _dbg_raw_current,
        scaled_current  : _state.current_amps,
        ltd_v1          : _ltd_v1,
        remaining_cap_mah : _dbg_remaining_cap_mah,
        branch1_est     : _dbg_branch1_est,
        branch2_est     : _dbg_branch2_est,
        w1              : _dbg_w1,
        time_remaining  : _state.time_remaining,
        in_flight       : _in_flight,
        ltd_converged   : _ltd_converged
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}
#endif  // HAL_LOGGING_ENABLED

#endif  // AP_BATTERY_SMBUS_GENERIC_ENABLED
