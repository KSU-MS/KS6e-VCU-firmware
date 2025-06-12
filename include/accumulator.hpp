#ifndef ACCUMULATOR_HPP
#define ACCUMULATOR_HPP
#include <Arduino.h>
#include "Metro.h"
#include "FlexCAN_util.hpp"

typedef struct precharge_status_t
{
    int16_t accVoltageReading;
    int16_t invVoltageReading;
    uint8_t statusEnum;
    elapsedMillis timeSinceLastHeartbeat;
} precharge_status_t;

class Accumulator
{

private:
    Metro *pchgTimeout;
    can_obj_ksu_ev_can_h_t *acc_ksu_can;
    bool preChargeAttempted_;
    bool imdstate;
    bool bmsstate;
    int pchgState;
    unsigned long pchgAliveTimer;
    uint8_t BMS_packInfo[8];
    can_0x6b1_MSGID_0X6B1_t bms_curr_lim_info;
    can_0x6b2_MSGID_0X6B2_t bms_voltage_info;
    precharge_status_t precharge_status;
    bool acc_ok;

public:
    Accumulator(Metro *pch_timeout,can_obj_ksu_ev_can_h_t *ksu_can_) : pchgTimeout(pch_timeout),acc_ksu_can(ksu_can_){};    
    void update_acc_state();
    void updateAccumulatorCAN();
    void sendPrechargeStartMsg();
    int get_precharge_state();
    bool check_precharge_success();
    bool check_precharge_timeout();
    bool GetIfPrechargeAttempted();
    void resetPchgState();
    int16_t get_acc_current();
    double get_acc_voltage();
    bool get_imd_state();
    bool get_bms_state();
    bool get_acc_state();
    void acc_debug_print();
};

#endif