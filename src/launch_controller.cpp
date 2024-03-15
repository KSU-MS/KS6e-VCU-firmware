#include "launch_controller.hpp"

const float cal5 = -0.000000000000085; // Fifth
const float cal4 = 0.000000000114956;  // Fourth
const float cal3 = 0.000000015913376;  // Third
const float cal2 = 0.000011808754927;  // Second
const float cal1 = 0.093415288604319;  // First order of poly
const float calIntercept = 10.361085973494500;

void launchController::initLaunchController(unsigned long sysTime)
{
    driverTorqueRequest = 0;
    lcTorqueRequest = 0;
    outputTorqueCommand = 0;
    this->setState(launchState::IDLE,sysTime);
}
launchState launchController::getState()
{
    return this->launchControlState;
}
launchState launchController::setState(const launchState nextState, unsigned long sysTime)
{
    launchState currentState = this->getState();
    if (currentState == nextState)
    {
        return currentState;
    }
    // State exit logic
    switch (currentState)
    {
    case launchState::IDLE:
    {
        break;
    }
    case launchState::WAITING_TO_LAUNCH:
    {
        break;
    }
    case launchState::LAUNCHING:
    {
        break;
    }
    case launchState::FINISHED:
    {
        break;
    }
    }

    launchControlState = nextState;
    // State ENTRY logic
    switch (nextState)
    {
    case launchState::IDLE:
    {
        this->disableTorqueCommanding = true;
        break;
    }
    case launchState::WAITING_TO_LAUNCH:
    {
        this->disableTorqueCommanding = true;
        break;
    }
    case launchState::LAUNCHING:
    {
        this->launchStartTime = sysTime;
        this->disableTorqueCommanding = false;
        break;
    }
    case launchState::FINISHED:
    {
        this->disableTorqueCommanding = false;
        break;
    }
    }
    return this->getState();
}

void launchController::run(unsigned long sysTime, int &torqueRequest)
{
    launchCurrentTime = sysTime;
    launchElapsedTime = launchCurrentTime - launchStartTime;
    driverTorqueRequest = torqueRequest;

    if (disableTorqueCommanding)
    {
        outputTorqueCommand = 0;
    }

    switch (this->getState())
    {
    case launchState::IDLE:
    {
        // Do nothing
        this->disableTorqueCommanding = true;
        break;
    }
    case launchState::WAITING_TO_LAUNCH:
    {
        // Do nothing?
        this->disableTorqueCommanding = true;
        break;
    }
    case launchState::LAUNCHING:
    {
        if (launchElapsedTime >= launchControlMaxDuration)
        {
            this->setState(launchState::FINISHED,sysTime);
            break;
        }
        outputTorqueCommand = this->calculateTorque(launchElapsedTime, driverTorqueRequest);

        break;
    }
    case launchState::FINISHED:
    {
        outputTorqueCommand = driverTorqueRequest;
        this->disableTorqueCommanding = false;
        break;
    }
    }
}

int launchController::calculateTorque(unsigned long elapsedTime, int maxTorque)
{
    int torqueOut = 0;
    // @jstri114 peep dis
    lcTorqueRequest = (cal5 * pow(elapsedTime, 5)) + (cal4 * pow(elapsedTime, 4)) + (cal3 * pow(elapsedTime, 3)) + (cal2 * pow(elapsedTime, 2)) + (cal1 * (elapsedTime)) + calIntercept;
    lcTorqueRequest *= 10; // Scale for inverter
    // this linear equation goes nowhere
    // lcTorqueRequest = (1.0/25.0) * (elapsedTime) + calIntercept; // Performs the calibration curve math, shrimple linear for now
    if (lcTorqueRequest > maxTorque)
    {
        torqueOut = maxTorque;
    }
    else
    {
        torqueOut = lcTorqueRequest;
    }
    return torqueOut;
}