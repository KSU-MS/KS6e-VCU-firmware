#ifndef LAUNCH_CONTROL_TEST
#define LAUNCH_CONTROL_TEST
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <iostream>
#include "launch_controller.hpp"
#include "launch_controller.cpp"
#include "AutoPID.h"

TEST(lcTesting, test_launch_control_exists)
{
    lc_lut launchControl;
    unsigned long systime = 0;
    launchControl.initLaunchController(systime);
    EXPECT_EQ(launchControl.getState(), launchState::IDLE);
    launchControl.setState(launchState::WAITING_TO_LAUNCH,0);
    EXPECT_EQ(launchControl.getState(), launchState::WAITING_TO_LAUNCH);
    launchControl.setState(launchState::LAUNCHING,0);
    EXPECT_EQ(launchControl.getState(), launchState::LAUNCHING);
    launchControl.setState(launchState::FINISHED,0);
    EXPECT_EQ(launchControl.getState(), launchState::FINISHED);
}

TEST(lcTesting, test_lc_notorque)
{
    wheelSpeeds_s wheelSpeedData = {0,0,0,0};
    lc_lut lc;
    unsigned long t = 0;
    lc.initLaunchController(t);
    int torque = 2400;
    lc.run(t,torque,wheelSpeedData);
    lc.getTorqueOutput();
    EXPECT_EQ(lc.getTorqueOutput(),0);

    torque= 2400;
    t++;
    launchState nextState = lc.setState(launchState::WAITING_TO_LAUNCH,t);
    lc.run(t,torque,wheelSpeedData);
    EXPECT_EQ(lc.getTorqueOutput(),0);

    torque= 2400;
    t++;
    nextState = lc.setState(launchState::LAUNCHING,t);
    lc.run(t,torque,wheelSpeedData);
    EXPECT_NE(lc.getTorqueOutput(),0);

}

TEST(lcTesting, test_lc_ramp)
{
    wheelSpeeds_s wheelSpeedData = {0,0,0,0};
    lc_lut lc;
    unsigned long t = 0;
    int nm = 2400;
    lc.initLaunchController(t);
    lc.setState(launchState::LAUNCHING,t);
    std::cout << "time" << "," << "torqueOut" << "\n";

    for (int i = 0; i < 1400; i += 10)
    {
        lc.run(i,nm,wheelSpeedData);
        std::cout << i << "," << lc.getTorqueOutput() << "\n";

    }

}

#endif