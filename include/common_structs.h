#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H
#include "parameters.hpp"
typedef struct wheelSpeeds_s {
    float fl;
    float fr;
    float rl;
    float rr;
    wheelSpeeds_s(float fl, float fr, float rl, float rr)
        : fl(fl), fr(fr), rl(rl), rr(rr){}
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
#endif