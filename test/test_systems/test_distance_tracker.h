#ifndef DISTANCE_TRACKER_TEST
#define DISTANCE_TRACKER_TEST
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "distance_tracker.h"
#include "parameters.hpp"
/*
distance_tracker works like this:
    void update(float amps, float volts, float rpm, float circumference, unsigned long newtime);


*/
/*
get_data() returns this:
struct energy_data_t
{
    int16_t energy_wh;
    int16_t eff_inst;
    int16_t distance_m;
    int16_t efficiency_kmkwh;
    energy_data_t(int16_t wh, int16_t eff, int16_t m, int16_t kmkwh) : energy_wh(wh), eff_inst(eff), distance_m(m), efficiency_kmkwh(kmkwh) {}
};
*/
TEST(distance_tracker_tests,init_zero)
{
    distance_tracker_s distance_tracker;
    EXPECT_EQ(distance_tracker.get_data().energy_wh,0);
    EXPECT_EQ(distance_tracker.get_data().eff_inst,0);
    EXPECT_EQ(distance_tracker.get_data().distance_m,0);
    EXPECT_EQ(distance_tracker.get_data().efficiency_kmkwh,0);
}
TEST(distance_tracker_tests,drive_onekm)
{
    distance_tracker_s distance_tracker;
    float amps = 10;
    float volts = 100;
    const float rpm = 11.58;

    distance_tracker.tick(0);
    for (unsigned long i = 0; i <= 1000*7200; i+= 100)
    {
        distance_tracker.update(amps,volts,rpm,WHEEL_CIRCUMFERENCE,i);
    }
    // EXPECT_EQ(distance_tracker.get_data().energy_wh,1000);
    // EXPECT_EQ(distance_tracker.get_data().eff_inst,0);
    // EXPECT_EQ(distance_tracker.get_data().distance_m,1000);
    // EXPECT_EQ(distance_tracker.get_data().efficiency_kmkwh,1);
    EXPECT_EQ(distance_tracker.capacity_ah,10);
    EXPECT_EQ(distance_tracker.energy_wh,1000);
    EXPECT_EQ(distance_tracker.distance_km,1000);
    EXPECT_EQ(distance_tracker.efficiency_kmkwh,1);
    EXPECT_EQ(distance_tracker.efficiency_instantaneous,1);
    printf("%fmeters %fah %fwh %fkm/kwh %fkm/kwh\n",distance_tracker.distance_km,distance_tracker.capacity_ah,distance_tracker.energy_wh,distance_tracker.efficiency_kmkwh,distance_tracker.efficiency_instantaneous);
}
/*
    float capacity_ah;
    float energy_wh = 0;
    float distance_km = 0;
    float efficiency_kmkwh = 0;
    float efficiency_instantaneous = 0;*/



#endif