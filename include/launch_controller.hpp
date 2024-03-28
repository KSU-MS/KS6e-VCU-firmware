#ifndef LAUNCH_CONTROLLER_HPP
#define LAUNCH_CONTROLLER_HPP
#include <cmath>
#include <cstring>
#include <common_structs.h>
#include <AutoPID.h>

enum class launchState
{
    IDLE = 0,
    WAITING_TO_LAUNCH = 1,
    LAUNCHING = 2,
    FINISHED = 3
};

enum launchControlTypes_e
{
    LC_DRIVERCONTROL = 0, // No firmware limiting, driver throttle directly
    LC_LOOKUP = 1, // Polynomial/ lookup table
    LC_PID = 2, // PID slip control
    LC_NUM_CONTROLLERS
};

struct diagData_s {
    unsigned long launchElapsedTime;
    int16_t outputTorqueCommand;
    uint8_t launchState;
    uint8_t launchType;
    diagData_s(unsigned long time, int16_t outputTorque, uint8_t state, uint8_t type)
        : launchElapsedTime(time), outputTorqueCommand(outputTorque), launchState(state), launchType(type) {}
};
class launchController
{

private:
    const unsigned long launchControlMaxDuration = 1200; // milliseconds that the launching period will run for
    unsigned long launchStartTime = 0;                   // ms the starting time of the launch
    unsigned long launchCurrentTime = 0;                 // ms the current time of the launch
    unsigned long launchElapsedTime = 0;                 // ms time elapsed from start of launch
    int driverTorqueRequest = 0;                         // Driver torque command if using pedal travel
    int lcTorqueRequest = 0;                             // launch control system torque command
    int outputTorqueCommand = 0;                         // The final torque command that will be output from the launch controller
    bool disableTorqueCommanding = true;                 // to disable torque commanding in IDLE and WAIT states
    launchState launchControlState;                      // state enum to control LC actions
public:
    void initLaunchController(unsigned long sysTime);                         // general to all launchControllers
    launchState getState();                                                   // general
    launchState setState(const launchState nextState, unsigned long sysTime); // general
    virtual int calculateTorque(unsigned long elapsedTime, int maxTorque, wheelSpeeds_s &wheelSpeedData);
    virtual void run(unsigned long sysTime, int &torqueRequest, wheelSpeeds_s &wheelSpeedData); // instance specific
    virtual launchControlTypes_e getType() {return launchControlTypes_e::LC_DRIVERCONTROL;}
    int getTorqueOutput() const { return outputTorqueCommand; };                                // general
    diagData_s getDiagData(); // 8 byte info on torque controller
};

class launchControllerLookup : public launchController
{
private:
    const float cal5 = -0.000000000000085; // Fifth
    const float cal4 = 0.000000000114956;  // Fourth
    const float cal3 = 0.000000015913376;  // Third
    const float cal2 = 0.000011808754927;  // Second
    const float cal1 = 0.093415288604319;  // First order of poly
    const float calIntercept = 10.361085973494500;

public:
    int calculateTorque(unsigned long elapsedTime, int maxTorque, wheelSpeeds_s &wheelSpeedData);
    launchControlTypes_e getType() {return launchControlTypes_e::LC_LOOKUP;}
};
class launchControllerPID : public launchController
{
private:
    const double tireSlipLow = 0.05;
    const double tireSlipHigh = 0.2;
    double d_kp = 1.5;
    double d_ki = 0;
    double d_kd = 0.5;
    const double output_min = 0.6; // Minimum output of the PID controller
    const double output_max = 1.0; // Max output of the PID controller
    double input, setpoint, output;
    AutoPID pid;

public:
    launchControllerPID() : pid(&input, &setpoint, &output, output_min, output_max, d_kp, d_ki, d_kd)
    {
        pid.setBangBang(tireSlipLow);
        pid.setTimeStep(1);
    }
    int calculateTorque(unsigned long elapsedTime, int maxTorque, wheelSpeeds_s &wheelSpeedData);
    launchControlTypes_e getType() {return launchControlTypes_e::LC_PID;}


};
#endif