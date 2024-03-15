#ifndef LAUNCH_CONTROLLER_HPP
#define LAUNCH_CONTROLLER_HPP
#include <cmath>

enum class launchState
{
    IDLE = 0,
    WAITING_TO_LAUNCH = 1,
    LAUNCHING = 2,
    FINISHED = 3
};

class launchController
{

private:
    const unsigned long launchControlMaxDuration = 1200; // milliseconds that the launching period will run for
    unsigned long launchStartTime = 0;   // ms the starting time of the launch
    unsigned long launchCurrentTime = 0; // ms the current time of the launch
    unsigned long launchElapsedTime = 0; // ms time elapsed from start of launch
    int driverTorqueRequest = 0;         // Driver torque command if using pedal travel
    int lcTorqueRequest = 0;             // launch control system torque command
    int outputTorqueCommand = 0;         // The ffinal torque command that will be output from the launch controller
    bool disableTorqueCommanding = true; // to disable torque commanding in IDLE and WAIT states
    launchState launchControlState; // state enum to control LC actions
public:
    void initLaunchController(unsigned long sysTime);
    launchState getState();
    launchState setState(const launchState nextState, unsigned long sysTime);
    int calculateTorque(unsigned long elapsedTime, int maxTorque);
    void run(unsigned long sysTime, int &torqueRequest);
    int getTorqueOutput() const { return outputTorqueCommand;};
};
#endif