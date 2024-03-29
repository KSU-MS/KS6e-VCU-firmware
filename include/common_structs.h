#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H
typedef struct wheelSpeeds_s {
    float fl;
    float fr;
    float rl;
    float rr;
    wheelSpeeds_s(float fl, float fr, float rl, float rr)
        : fl(fl), fr(fr), rl(rl), rr(rr){}
} wheelSpeeds_s;
#endif