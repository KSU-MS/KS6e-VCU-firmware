#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H
#include "parameters.hpp"
typedef struct wheelSpeeds_s
{
    float fl;
    float fr;
    float rl;
    float rr;
    wheelSpeeds_s(float fl, float fr, float rl, float rr)
        : fl(fl), fr(fr), rl(rl), rr(rr) {}
    float calc_slip()
    {
        // Calculate front and rear wheel speeds - take average of left and right
        float frontRpmAvg = ((fl + fr) / 2);
        float rearRpmAvg = ((rl + rr) / 2);
        // avoid zero division
        if (frontRpmAvg <= 10 || rearRpmAvg <= 10)
        {
            return 0;
        }
        else
        {
            // Slip = (rear / front) - 1
            // ie. 1000rpm/900rpm = 1.111..
            // 1.111 - 1 = 0.111 slip ratio 
            return (rearRpmAvg / frontRpmAvg) - 1;
        }
    }
    void load_diag_data(uint16_t joe[3])
    {
        float frontRpmAvg = ((fl + fr) / 2);
        float rearRpmAvg = ((rl + rr) / 2);
        joe[0] = static_cast<uint16_t>(frontRpmAvg*10);
        joe[1] = static_cast<uint16_t>(rearRpmAvg*10);
        joe[2] = static_cast<uint16_t>(calc_slip()*100);
    }

} wheelSpeeds_s;

typedef struct lc_countdown_t
{
    unsigned long release_countdown;
    const unsigned long release_delay = LAUNCHCONTROL_RELEASE_DELAY;
} lc_countdown_t;

typedef struct time_and_distance_tracker_t
{
    unsigned long vcu_lifetime_ontime;
    unsigned long vcu_lifetime_distance;
} time_and_distance_tracker_t;

typedef struct coulomb_count_t
{
    uint16_t capacity_ah;
} coulomb_count_t;
typedef struct vn_data_t
{
    double yaw = 0;
    double pitch = 0;
    double roll = 0;

    double w_x = 0;
    double w_y = 0;
    double w_z = 0;

    double velocity_north = 0;
    double velocity_east = 0;
    double velocity_down = 0;
    double velocity_magnitude = 0;

    double accel_x = 0;
    double accel_y = 0;
    double accel_z = 0;
    float mock_ws_rpm()
    {
        float vn_mock_ws = velocity_magnitude / WHEEL_CIRCUMFERENCE; // Divide meters per second by circumference to get Revs per Second
        vn_mock_ws *= 60;
        return vn_mock_ws;
    }
    unsigned long last_update_time = 0;

} vn_data_t;
#endif