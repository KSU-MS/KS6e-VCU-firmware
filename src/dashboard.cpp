
#include "FlexCAN_util.hpp"
#include "dashboard.hpp"


const uint8_t NUM_OF_DIGITS = 4;


void Dashboard::updateDashCAN()
{
    CAN_message_t rxMsg;

    if (ReadInverterCAN(rxMsg))
    {
        if (rxMsg.id == ID_DASH_BUTTONS)
        {
            button_states = rxMsg.buf[0];
        }
    }


}

bool Dashboard::updatePRECHG_RELIABLE()
{
    CAN_message_t rxMsg;

    if (ReadAccumulatorCAN(rxMsg)){
        if(rxMsg.id == ID_PRECHG_INFO){
            bool RELIABLE_state_PRECHG = rxMsg.buf[7]; 
            return RELIABLE_state_PRECHG;            
        }
    }
}

bool Dashboard::updateACU_RELIABLE()
{
    CAN_message_t rxMsg;

    if (ReadAccumulatorCAN(rxMsg)){
        if(rxMsg.id == ID_ACU_RELAY){
            bool RELIABLE_state_ACU = rxMsg.buf[7]; 
            return RELIABLE_state_ACU;            
        }
    }
}

void Dashboard::ByteEachDigit(int num)
{
   for (int i = NUM_OF_DIGITS - 1; i >= 0; i--)
   {
        this->BusVolt_ByteEachDigit[i] = num % 10;
        num = num / 10;
   }
}


uint8_t *Dashboard::getBusVoltage()
{
    return this->BusVolt_ByteEachDigit;
}

