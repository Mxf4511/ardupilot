#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_BATTMONITOR \
    LOG_BAT_MSG, \
    LOG_BCL_MSG, \
    LOG_BTR_MSG

// @LoggerMessage: BAT
// @Description: Gathered battery data
// @Field: TimeUS: Time since system startup
// @Field: Inst: battery instance number
// @Field: Volt: measured voltage
// @Field: VoltR: estimated resting voltage
// @Field: Curr: measured current
// @Field: CurrTot: consumed Ah, current * time
// @Field: EnrgTot: consumed Wh, energy this battery has expended
// @Field: Temp: measured temperature
// @Field: Res: estimated battery resistance
// @Field: RemPct: remaining percentage
// @Field: H: health
// @Field: SH: state of health percentage.  0 if unknown
struct PACKED log_BAT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    voltage;
    float    voltage_resting;
    float    current_amps;
    float    current_total;
    float    consumed_wh;
    int16_t  temperature; // degrees C * 100
    float    resistance;
    uint8_t  rem_percent;
    uint8_t  health;
    uint8_t  state_of_health_pct;
};

// @LoggerMessage: BCL
// @Description: Battery cell voltage information
// @Field: TimeUS: Time since system startup
// @Field: Instance: battery instance number
// @Field: Volt: battery voltage
// @Field: V1: first cell voltage
// @Field: V2: second cell voltage
// @Field: V3: third cell voltage
// @Field: V4: fourth cell voltage
// @Field: V5: fifth cell voltage
// @Field: V6: sixth cell voltage
// @Field: V7: seventh cell voltage
// @Field: V8: eighth cell voltage
// @Field: V9: ninth cell voltage
// @Field: V10: tenth cell voltage
// @Field: V11: eleventh cell voltage
// @Field: V12: twelfth cell voltage
struct PACKED log_BCL {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    voltage;
    uint16_t cell_voltages[12]; // the format does not support more than 12 cells, the remaining cells are reported in the BCL2 message
};

// @LoggerMessage: BTR
// @Description: Battery time remaining estimation debug data
// @Field: TimeUS: Time since system startup
// @Field: Inst: battery instance number
// @Field: RCurr: raw current before scaling
// @Field: SCurr: scaled current
// @Field: FiltC: LTD filtered current (v1)
// @Field: RemCap: remaining capacity (mAh)
// @Field: B1: branch 1 time estimate (sec)
// @Field: B2: branch 2 time estimate (sec)
// @Field: W1: branch 1 weight
// @Field: TRem: estimated time remaining (sec)
// @Field: Fl: in flight flag
// @Field: Conv: LTD converged flag
struct PACKED log_BTR {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    raw_current;
    float    scaled_current;
    float    ltd_v1;
    float    remaining_cap_mah;
    float    branch1_est;
    float    branch2_est;
    float    w1;
    uint32_t time_remaining;
    uint8_t  in_flight;
    uint8_t  ltd_converged;
};

#define LOG_STRUCTURE_FROM_BATTMONITOR        \
    { LOG_BAT_MSG, sizeof(log_BAT), \
        "BAT", "QBfffffcfBBB", "TimeUS,Inst,Volt,VoltR,Curr,CurrTot,EnrgTot,Temp,Res,RemPct,H,SH", "s#vvAaXOw%-%", "F-000C0?0000" , true },  \
    { LOG_BCL_MSG, sizeof(log_BCL), \
        "BCL", "QBfHHHHHHHHHHHH", "TimeUS,Instance,Volt,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12", "s#vvvvvvvvvvvvv", "F-0CCCCCCCCCCCC" , true },  \
    { LOG_BTR_MSG, sizeof(log_BTR), \
        "BTR", "QBfffffffIBB", "TimeUS,Inst,RCurr,SCurr,FiltC,RemCap,B1,B2,W1,TRem,Fl,Conv", "s#AAAOssss--%", "F-0000000C--" , true },
