// library includes
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>
#include <string.h>
#include <stdint.h>
#include <FreqMeasureMulti.h>
#include <cmath>
// our includes
#include "MCU_status.hpp"
#include "KS2eCAN.hpp"
#include "KS2eVCUgpios.hpp"
#include "FlexCAN_util.hpp"
#include "parameters.hpp"
#include "inverter.hpp"
#include "accumulator.hpp"
#include "state_machine.hpp"
#include "ksu_device_status.h"

#define NEBUG
static can_obj_ksu_ev_can_h_t ksu_can;
// Metro timers for inverter:
Metro timer_mc_kick_timer = Metro(50, 1); // Motor controller heartbeat timer
Metro timer_inverter_enable = Metro(2000, 1); // Timeout for inverter enabling
Metro timer_motor_controller_send = Metro(5, 1); // Motor controller torque command timer
Metro timer_current_limit_send = Metro(10, 1); // Motor controller power limiting timer

// timers for the accumulator:
// precharge_timeout is the time allowed for no precharge message to be received until the car goes out of RTD
Metro precharge_timeout = Metro(500, 0);

// timers for the pedals:

Metro timer_debug_pedals_raw = Metro(100, 1);
Metro pedal_out_20hz = Metro(50, 1); // CAN sending at 20hz
Metro pedal_out_1hz = Metro(1000,1); // CAN sending at 1hz
Metro pedal_check = Metro(40, 1);

// timers for the dashboard:
// there are no timers for the dash
// timers for the state machine:
Metro timer_ready_sound = Metro(1000); // Time to play RTD sound
Metro debug_tim = Metro(200, 1);

// PID shit
double current_rpm, set_rpm, throttle_out;
const double KP = D_KP;
const double KI = D_KI;
const double KD = D_KD;
const double OUTPUT_MIN = D_OUTPUT_MIN;
const double OUTPUT_MAX = D_OUTPUT_MAX;

AutoPID speedPID(&current_rpm, &set_rpm, &throttle_out, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// timers for VCU state out:
Metro timer_can_update = Metro(1000, 1);

// Wheel speed shit
FreqMeasureMulti wsfl;
FreqMeasureMulti wsfr;
#define ENABLED_LC_TYPES {launchControlTypes_e::LC_DRIVERCONTROL,launchControlTypes_e::LC_LINEAR,launchControlTypes_e::LC_PID}
#define ENABLED_TC_TYPES {torque_control_types_e::TC_DRIVERCONTROL,torque_control_types_e::TC_PID,torque_control_types_e::TC_SlipTime}
// objects
Dashboard dash;
Inverter pm100(&timer_mc_kick_timer, &timer_inverter_enable, &timer_motor_controller_send, &timer_current_limit_send, &dash);
Accumulator accum(&precharge_timeout,&ksu_can);
PedalHandler pedals(&timer_debug_pedals_raw, &pedal_out_20hz, &pedal_out_1hz, &speedPID, &current_rpm, &set_rpm, &throttle_out, &wsfl, &wsfr);
launchControlSystem launchSystem(ENABLED_LC_TYPES); // THIS WILL INCLUDE *ALL* LAUNCH MODES BY DEFAULT
torque_control_system tractionControlSystem(ENABLED_TC_TYPES);
StateMachine state_machine(&pm100, &accum, &timer_ready_sound, &dash, &debug_tim, &pedals, &launchSystem, &tractionControlSystem, &pedal_check, &ksu_can);
MCU_status mcu_status = MCU_status();

static CAN_message_t mcu_status_msg;
static CAN_message_t fw_hash_msg;
static CAN_message_t pedal_thresholds_msg;
device_status_t vcu_status_t;
pedal_thresholds_t vcu_pedal_thresholds_t;

void gpio_init();
// Use this to emulate a changing signal from the ADC
#if DEBUG
uint16_t generateSineValue(double time_ms, double amplitude, double frequency, double phase_shift, double offset) {
    // Convert time from milliseconds to seconds
    double time_s = time_ms / 1000.0;

    // Calculate the angle (in radians) based on time, frequency, and phase shift
    double angle = 2 * M_PI * frequency * time_s + phase_shift;

    // Calculate the sine value
    double value = amplitude * sin(angle) + offset;
    value+=amplitude;
    // Ensure the value is within the 12-bit unsigned integer range
    value = fmax(0, fmin(value, 4095));
    // Serial.println(value);
    // double value = 1000;
    return static_cast<uint16_t>(value);
}
#endif
int dummy_max_torque = 1000;
int16_t dummy_motor_speed = 1000;
//----------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
    Serial.begin(57600);
    Serial.setTimeout(0);
    delay(100);

    InitCAN();
    gpio_init();
    mcu_status.set_max_torque(0); // no torque on startup
    mcu_status.set_torque_mode(0);
    // build up fw git hash message
    fw_hash_msg.id = ID_VCU_FW_VERSION;
    fw_hash_msg.len = sizeof(vcu_status_t) / sizeof(uint8_t);
#if DEBUG
    Serial.printf("FW git hash: %lu, IS_DIRTY: %d IS_MAIN: %d\n", AUTO_VERSION, FW_PROJECT_IS_DIRTY, FW_PROJECT_IS_MAIN_OR_MASTER);
    Serial.printf("FW git hash in the struct: %lu\n", vcu_status_t.firmware_version);
#endif
    vcu_status_t.on_time_seconds = millis() / 1000;
    memcpy(fw_hash_msg.buf, &vcu_status_t, sizeof(vcu_status_t));

    mcu_status.set_inverter_powered(true); // note VCU does not control inverter power on rev3
    uint8_t last_mode = 1;
    EEPROM.get(TORQUE_MODE_EEPROM_ADDR,last_mode);
    if (last_mode > 4 || last_mode < 1)
    {
        last_mode = 3;
    }
    Serial.printf("Last torque mode (from EEPROM: %d)\n",last_mode);
    mcu_status.set_torque_mode(last_mode);         // TODO torque modes should be an enum
    mcu_status.set_max_torque(torque_mode_list[last_mode-1]);   // TORQUE_1=10nm, 2=54nm, 3=180nm, 4=240nm
    state_machine.init_state_machine(mcu_status);
}

void loop()
{
    // delay(10);
    // uint16_t value = generateSineValue(millis(),1500/2,0.5,0,0);
    // pedals.read_pedal_values_debug(value);
    // pedals.run_pedals();
    // pedals.calculate_torque(dummy_motor_speed,dummy_max_torque);
    state_machine.handle_state_machine(mcu_status);

    if (timer_can_update.check())
    {
        // Send Main Control Unit status message
        mcu_status.write(mcu_status_msg.buf);
        mcu_status_msg.id = ID_VCU_STATUS;
        mcu_status_msg.len = sizeof(mcu_status);
        WriteCANToInverter(mcu_status_msg);

        // broadcast firmware git hash
        vcu_status_t.on_time_seconds = millis() / 1000;
        memcpy(fw_hash_msg.buf, &vcu_status_t, sizeof(vcu_status_t));
        WriteCANToInverter(fw_hash_msg);

        sendStructOnCan(state_machine.get_lifetime_data(),ID_VCU_LIFETIME_DATA);

        // broadcast pedal thresholds information
        pedal_thresholds_msg.id = ID_VCU_PEDAL_THRESHOLD_SETTINGS;
        pedal_thresholds_msg.len = 7;
        pedal_thresholds_msg.buf[0]=0;
        memcpy(&pedal_thresholds_msg.buf[1], &vcu_pedal_thresholds_t.pedal_thresholds_0, sizeof(vcu_pedal_thresholds_t.pedal_thresholds_0));
        WriteCANToInverter(pedal_thresholds_msg);

        pedal_thresholds_msg.buf[0]=1;
        memcpy(&pedal_thresholds_msg.buf[1], &vcu_pedal_thresholds_t.pedal_thresholds_1, sizeof(vcu_pedal_thresholds_t.pedal_thresholds_1));
        WriteCANToInverter(pedal_thresholds_msg);

        pedal_thresholds_msg.buf[0]=2;
        memcpy(&pedal_thresholds_msg.buf[1], &vcu_pedal_thresholds_t.pedal_thresholds_2, sizeof(vcu_pedal_thresholds_t.pedal_thresholds_2));
        WriteCANToInverter(pedal_thresholds_msg);
    }
}

void gpio_init()
{
    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);
    pinMode(LOWSIDE1, OUTPUT);
    pinMode(LOWSIDE2, OUTPUT);
    pinMode(WSFL, INPUT_PULLUP);
    pinMode(WSFR, INPUT_PULLUP);
}