#ifndef LAUNCH_SYSTEM_H
#define LAUNCH_SYSTEM_H
#include <unordered_map>
#include "launch_controller.hpp"
#include <parameters.hpp>
class launchControlSystem
{
    public:
        void setActiveSystem(launchControlTypes_e systype) {lcType = systype;}
        launchControlTypes_e getActiveSystem() {return lcType;}
        launchController* getController()
        {
            return static_cast<launchController*>(lc_map[lcType]);
        }
        void toggleController(unsigned long systime);
        void printControllerType(char buf[60]);
        
    private:
        launchControlTypes_e lcType = launchControlTypes_e::LC_DRIVERCONTROL;
        launchController lc_base;
        launchControllerLookup lc_lookup;
        launchControllerPID lc_pid;
        launchControllerLinear lc_linear = launchControllerLinear(0,TORQUE_2,200,TORQUE_4);
        std::unordered_map<launchControlTypes_e, void*> lc_map = {
            {launchControlTypes_e::LC_DRIVERCONTROL, &lc_base},
            {launchControlTypes_e::LC_LOOKUP, &lc_lookup},
            {launchControlTypes_e::LC_PID, &lc_pid},
            {launchControlTypes_e::LC_LINEAR, &lc_linear}
        };



};
#endif