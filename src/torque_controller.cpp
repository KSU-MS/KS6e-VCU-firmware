#include "torque_controller.hpp"

void torque_controller::inittorque_controller(unsigned long sysTime)
{
    driverTorqueRequest = 0;
    lcTorqueRequest = 0;
    outputTorqueCommand = 0;
    this->launchElapsedTime = 0;
}


tc_diag_data_t torque_controller::getDiagData()
{
    tc_diag_data_t diagData = tc_diag_data_t{this->launchElapsedTime,this->outputTorqueCommand,static_cast<uint8_t>(this->getType())};
    return diagData;
}

// PID to get optimal slip
int16_t torque_controllerPID::calculate_torque(unsigned long elapsedTime, int16_t maxTorque, wheelSpeeds_s &wheelSpeedData)
{
    driverTorqueRequest = maxTorque;
    float torqueOut = 0;

    // Calculate front and rear wheel speeds - take average of left and right
    float frontRpmAvg = ((wheelSpeedData.fl+wheelSpeedData.fr)/2);
    float rearRpmAvg =  ((wheelSpeedData.rl+wheelSpeedData.rr)/2);
    printf("TC Front avg: %f Rear avg: %f\n",frontRpmAvg,rearRpmAvg);
    printf("TC WHEELSPEEDS \nFL: %f FR: %f\nRL: %f RR: %f\n",wheelSpeedData.fl,wheelSpeedData.fr,wheelSpeedData.rl,wheelSpeedData.rr);
    // avoid zero division
    if (frontRpmAvg || rearRpmAvg <= 0.001)
    {
        this->input = 0; // treat it like 0 slip (maybe this is bad)
    }
    else
    {
        // Slip = (rear / front) - 1
        // ie. 1000rpm/900rpm = 1.111..
        // 1.111 - 1 = 0.111 slip ratio
        this->input = (rearRpmAvg / frontRpmAvg) - 1;
    }
 
    pid.run(elapsedTime);
    torqueOut = maxTorque + (output * maxTorque);
    lcTorqueRequest = static_cast<int16_t>(torqueOut); // Pre clamping
    if (torqueOut > maxTorque)
    {
        torqueOut = maxTorque;
    }
    if (torqueOut < 0)
    {
        torqueOut = 0;
    }
    tc_pid_t._curr_slip = static_cast<int16_t>(input*100);
    outputTorqueCommand = static_cast<int16_t>(torqueOut); // Post-clamping
    return outputTorqueCommand;  
}
