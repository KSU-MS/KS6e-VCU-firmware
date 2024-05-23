#include "state_machine.hpp"
#define ACCDEBUG
//  initializes the mcu status and pedal handler
void StateMachine::init_state_machine(MCU_status &mcu_status)
{
  EEPROM.get(ODOMETER_EEPROM_ADDR,_lifetime_distance);
  EEPROM.get(10,_lifetime_on_time);
  Serial.printf("Loaded lifetime distance: %d meters (%f km)\n",_lifetime_distance,static_cast<float>(_lifetime_distance)/1000);
  Serial.printf("Loaded lifetime on_time: %d seconds (%f hours)\n",_lifetime_on_time,static_cast<float>(_lifetime_on_time)/3600);
  set_state(mcu_status, MCU_STATE::STARTUP);
  pedals->init_pedal_handler();
  distance_tracker_motor.tick(millis());
  distance_tracker_fl.tick(millis());
}

// Send a state message on every state transition so we don't miss any
void StateMachine::send_state_msg(MCU_status &mcu_status)
{
  CAN_message_t tx_msg;
  mcu_status.write(tx_msg.buf);
  tx_msg.id = ID_VCU_STATUS;
  tx_msg.len = sizeof(mcu_status);
  WriteCANToInverter(tx_msg);
}
/* Handle changes in state */
void StateMachine::set_state(MCU_status &mcu_status, MCU_STATE new_state)
{
  // Send current mcu_status before state transition
  send_state_msg(mcu_status);
  // If current state is the same as new state, exit
  if (mcu_status.get_state() == new_state)
  {
    return;
  }

  // exit logic ----------------------------------------------------------------------------------------------------------------------------------------------------
  switch (mcu_status.get_state())
  {
  case MCU_STATE::STARTUP: // ----------
  {
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE: // ----------
  {
#if USE_INVERTER
    pm100->tryToClearMcFault();
#endif
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE: // ----------
  {
    // Reset precharge status when exit of TS ACTIVE
    accumulator->resetPchgState(); // precharge will time out but stay "ready" if we don't reset it here
    break;
  }
  case MCU_STATE::ENABLING_INVERTER: // ----------
  {
    break;
  }
  case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND: // ----------
  {
    // make dashboard stop buzzer
    mcu_status.set_activate_buzzer(false);
    digitalWrite(BUZZER, LOW);
    break;
  }
  case MCU_STATE::READY_TO_DRIVE: // ----------
  {
    // reset "state" of precharge in memory
    accumulator->resetPchgState();
    // disable lowside outputs (pump, etc.)
    digitalWrite(LOWSIDE1, LOW);
    digitalWrite(LOWSIDE2, LOW);
    mcu_status.set_launch_ctrl_active(0); // disable launch control if it was active
    break;
  }
  }

  mcu_status.set_state(new_state);
  // Send new mcu_status after transition
  send_state_msg(mcu_status);
  // entry logic ----------------------------------------------------------------------------------------------------------------------------------------------------
  switch (new_state)
  {
  case MCU_STATE::STARTUP: // ----------
  {
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE: // ----------
  {
#if USE_INVERTER
    pm100->forceMCdischarge();
#endif
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE: // ----------
  {
    break;
  }
  case MCU_STATE::ENABLING_INVERTER: // ----------
  {
#if USE_INVERTER
    pm100->tryToClearMcFault();

    pm100->doStartup();
#endif
    break;
  }
  case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND: // ----------
  {
    // make dashboard sound buzzer
    mcu_status.set_activate_buzzer(true);
    digitalWrite(BUZZER, HIGH);
    timer_ready_sound->reset();
    break;
  }
  case MCU_STATE::READY_TO_DRIVE: // ----------
  {
    // enable low-side outputs
    digitalWrite(LOWSIDE1, HIGH);
    digitalWrite(LOWSIDE2, HIGH);
    break;
  }
  }
}

// Constant logic ----------------------------------------------------------------------------------------------------------------------------------------------------
void StateMachine::handle_state_machine(MCU_status &mcu_status)
{
  bool _10hz_send = can_10hz_timer.check();
  bool _20hz_send = can_20hz_timer.check();
  bool _100hz_send = can_100hz_timer.check();
  // things that are done every loop go here:

#if USE_INVERTER
  pm100->updateInverterCAN();
#endif
  accumulator->updateAccumulatorCAN();
  // dash_->updateDashCAN(); Reading inverter CAN and "dash can" at the same time
  // causes a lot of delay. DOn't do it

  mcu_status.set_brake_pedal_active(pedals->read_pedal_values());
  pedals->run_pedals(); // Maybe wrap all of this in a "pedals superloop" function
  pedals->ws_run();
  mcu_status.set_imd_ok_high(accumulator->get_imd_state());
  mcu_status.set_bms_ok_high(accumulator->get_bms_state());
  mcu_status.set_bspd_ok_high(pedals->get_board_sensor_readings());
  mcu_status.set_bspd_current_high((accumulator->get_acc_current() > (bspd_current_high_threshold * 10)));
  pedals->send_readings();
  if (_20hz_send)
  {
    sendStructOnCan(lcSystem->getController()->getDiagData(), ID_VCU_BASE_LAUNCH_CONTROLLER_INFO);
    sendStructOnCan(tcSystem->getController()->getDiagData(), ID_VCU_TRACTION_CONTROLLER_INFO);
  }

  if (_100hz_send)
  {
    handle_distance_trackers(mcu_status);
  }

  if (_10hz_send)
  {
    sendStructOnCan(distance_tracker_fl.get_data(),ID_VCU_DISTANCE_TRACKER_WHEELSPEED);
    sendStructOnCan(distance_tracker_motor.get_data(),ID_VCU_DISTANCE_TRACKER_MOTOR);

  }

// DASH BUTTON INTERACTIONS
  // If dash button is on and has been on for 750ms
  // AND the motor is not spinning!!
  if ((!dash_->get_button(2)))
  {
    if (dash_->get_button_held_duration(6, 750) && (pm100->getmcMotorRPM() <= 300) && !mcu_status.get_launch_ctrl_active())
    {
      dash_->set_button_last_pressed_time(0, 6);
      mcu_status.toggle_max_torque(mcu_status.get_torque_mode());
      mcu_status.set_max_torque(torque_mode_list[mcu_status.get_torque_mode() - 1]);
      send_state_msg(mcu_status);
    }
  }

// TODO test this
  if (dash_->get_button_held_duration(4,500) && (pm100->getmcMotorRPM() <= 300))
  {
    tcSystem->toggleController(millis());
    tcSystem->getController()->inittorque_controller(millis());
    sendStructOnCan(tcSystem->getController()->getDiagData(),ID_VCU_TRACTION_CONTROLLER_INFO);
  }
  // If dash button held and LC not active
  if (!dash_->get_button(6))
  {
    if (dash_->get_button_held_duration(2, 500) && lcSystem->getController()->getState() == launchState::IDLE)
    {
      lcSystem->toggleController(millis());
      // init new system
      lcSystem->getController()->initLaunchController(millis());
      sendStructOnCan(lcSystem->getController()->getDiagData(), ID_VCU_BASE_LAUNCH_CONTROLLER_INFO);
    }
  }
  // End of (most) dash thingies
  // Do Torque Calcs here
  int16_t calculated_torque = 0;
  bool accel_is_plausible = false;
  bool brake_is_plausible = false;
  bool accel_and_brake_plausible = false;
  bool impl_occ = true;

  // FSAE EV.5.5
  // FSAE T.4.2.10
  pedals->verify_pedals(accel_is_plausible, brake_is_plausible, accel_and_brake_plausible, impl_occ);
  mcu_status.set_accel_implausible(!accel_is_plausible);
  mcu_status.set_brake_implausible(!brake_is_plausible);
  mcu_status.set_accel_brake_implausible(!accel_and_brake_plausible);

  if (accel_is_plausible && brake_is_plausible && accel_and_brake_plausible && (!impl_occ))
  {

    int max_t_actual = mcu_status.get_max_torque() * 10;

    int16_t motor_speed = 0;
#if USE_INVERTER
    motor_speed = pm100->getmcMotorRPM();
#endif
// TODO test TCs
    calculated_torque = pedals->calculate_torque(motor_speed, max_t_actual);
    wheelSpeeds_s wheelSpeedData = {pedals->get_wsfl(), pedals->get_wsfr(), pm100->getmcMotorRPM(), pm100->getmcMotorRPM()};
    calculated_torque = tcSystem->getController()->calculate_torque(millis(),calculated_torque,wheelSpeedData);
    // REGEN
    if (mcu_status.get_brake_pedal_active() && dash_->get_button2() && calculated_torque < 5)
    {
      calculated_torque = pedals->calculate_regen(motor_speed, REGEN_NM);
    }
    else
    {
      // Reset regen to 0 when not commanding it
      pedals->reset_regen();
    }
  }
  // joe_mock_lc(&mcu_status,calculated_torque,impl_occ);
  // end of functions that run every loop unconditionally

  // start of state machine conditional functionality
  switch (mcu_status.get_state())
  {
  case MCU_STATE::STARTUP: // --------------------
  {
    set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE: // --------------------
  {
#if USE_INVERTER
    pm100->inverter_kick(0);
#endif
    if (!accumulator->GetIfPrechargeAttempted())
    {
      accumulator->sendPrechargeStartMsg(); // we dont actually need to send this-precharge is automatic
    }

    bool accumulator_ready = false;

    if (accumulator->check_precharge_success() && (!accumulator->check_precharge_timeout()))
    {
      accumulator_ready = true;
    }
    else if ((!accumulator->check_precharge_timeout()) && (!accumulator->check_precharge_success()))
    {
      // if the accumulator hasnt finished precharge and it hasnt timed out yet, break

      break;
    }
    else if (accumulator->check_precharge_timeout() && (!accumulator->check_precharge_success()))
    {
      // attempt pre-charge again if the pre-charge has timed out
      // TODO add in re-try limitation number
      accumulator->sendPrechargeStartMsg();
      break;
    }
    // if TS is above HV threshold, move to Tractive System Active
#if USE_INVERTER

    if (pm100->check_TS_active() && accumulator_ready)
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
    }
#else
    if (accumulator_ready)
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
    }
#endif

    break;
  }
  // TRACTIVE SYSTEM ACTIVE, HV ON
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE: // --------------------
  {

    // TODO (Disabled to test error 3/27/23)
#if USE_INVERTER
    if (!pm100->check_TS_active())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
    }
#else
    if (false)
    {
    } // dummy
#endif
    else if (accumulator->check_precharge_timeout())
    {

      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
    }
#if USE_INVERTER
    pm100->inverter_kick(0);
#endif
    // Serial.println(dash_->get_button3());
    // if start button has been pressed and brake pedal is held down, transition to the next state
    if (dash_->get_button3() && mcu_status.get_brake_pedal_active())
    {
      set_state(mcu_status, MCU_STATE::ENABLING_INVERTER);
    }

    break;
  }
  // ATTEMPTING TO ENABLE INVERTER
  case MCU_STATE::ENABLING_INVERTER: // --------------------
  {
#if USE_INVERTER
    pm100->inverter_kick(1);
    if (!pm100->check_TS_active())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }
#else
    if (false)
    {
    }
#endif
    else if (accumulator->check_precharge_timeout())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }

    // inverter enabling timed out
#if USE_INVERTER
    bool tuff = pm100->check_inverter_enable_timeout();

    if (tuff) // if inverter times out , go from ENABLING_INVERTER back to TRACTIVE_SYSTEM_ACTIVE
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
      break;
    }

    // motor controller indicates that inverter has enabled within timeout period
    if (pm100->check_inverter_ready())

    {
      set_state(mcu_status, MCU_STATE::WAITING_READY_TO_DRIVE_SOUND);
      break;
    }
#else
    set_state(mcu_status, MCU_STATE::WAITING_READY_TO_DRIVE_SOUND);
#endif

    break;
  }
  // WAITING TO ENTER READY TO DRIVE (BUZZER ACTIVE)
  case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND: // --------------------
  {
#if USE_INVERTER
    pm100->inverter_kick(1);
    if (!pm100->check_TS_active())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }
    if (pm100->check_inverter_disabled())
    {

      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
      break;
    }
#endif
    // if the ready to drive sound has been playing for long enough, move to ready to drive mode
    if (timer_ready_sound->check())
    {

      set_state(mcu_status, MCU_STATE::READY_TO_DRIVE);
    }
    break;
  }
  // READY TO DRIVE
  case MCU_STATE::READY_TO_DRIVE: // --------------------
  {
// #ifdef CHEESE_RTD //teehee
#if USE_INVERTER

    if (!pm100->check_TS_active())
    {
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }

    if (pm100->check_inverter_disabled())
    {

      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_ACTIVE);
      break; // TODO idk if we should break here or not but it sure seems like it
    }
#endif
    if (accumulator->check_precharge_timeout())
    { // if the precharge hearbeat has timed out, we know it is no longer enabled-> the SDC is open
      set_state(mcu_status, MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE);
      break;
    }
    // #endif
    // Torque calc always runs in the superloop
    // Toggle launch control if button 5 held for 1 second, while brake is pressed
    if (dash_->get_button_held_duration(LAUNCH_CONTROL_BUTTON, 1000))
    { //   && mcu_status.get_brake_pedal_active()
      {
        // Toggle launch control (allows deactivating if sitting in it)
        mcu_status.set_launch_ctrl_active(!(mcu_status.get_launch_ctrl_active()));
        // Auto-set to max torque mode
        if (mcu_status.get_launch_ctrl_active())
        {
          mcu_status.set_max_torque(TORQUE_4);
          mcu_status.set_torque_mode(4);
        }
        send_state_msg(mcu_status);

        // Reset the launch controller each time we toggle it
        lcSystem->getController()->initLaunchController(millis());
      }
#if DEBUG
      Serial.printf("DEBUG: Set launch control to %d", mcu_status.get_launch_ctrl_active());
#endif
    }
    if (mcu_status.get_launch_ctrl_active())
    {
      // Do launch control things
      switch (lcSystem->getController()->getState())
      {
      case launchState::IDLE:
      {
        calculated_torque = 0; // Set torque to zero in IDLE
        // If button is held, APPS is floored (90%), brake is not active, and impl_occ is false
        // THEN: go to WAITING_TO_LAUNCH
        if (dash_->get_button(6) && (pedals->getAppsTravel() > 0.9) && !(mcu_status.get_brake_pedal_active()) && !impl_occ)
        {
          lcSystem->getController()->setState(launchState::WAITING_TO_LAUNCH, millis());
          break;
        }
        break;
      }
      case launchState::WAITING_TO_LAUNCH:
      {
        calculated_torque = 0; // Set torque to zero in WAITING_TO_LAUNCH
        // If gas is released, return to IDLE
        if ((pedals->getAppsTravel() < 0.5))
        {
          lcSystem->getController()->setState(launchState::IDLE, millis());
          break;
        }
        // If gas is still pinned and button has been released for 1000ms, start LAUNCHING
        if ((pedals->getAppsTravel() > 0.9) && !dash_->get_button6() && dash_->get_button_released_duration(6, LAUNCHCONTROL_RELEASE_DELAY))
        {
          lcSystem->getController()->setState(launchState::LAUNCHING, millis());
          break;
        }
        else if (dash_->get_button6())
        {
          dash_->set_button_last_pressed_time(0, 6);
        }
        if (_20hz_send)
        {
          lc_countdown_t countdown_t;
          countdown_t.release_countdown = dash_->get_button_last_pressed_time(6);
          sendStructOnCan(countdown_t, ID_VCU_LAUNCH_CONTROL_COUNTDOWN);
        }
        break;
      }
      case launchState::LAUNCHING:
      {
        if ((mcu_status.get_brake_pedal_active()) || impl_occ)
        {
          // Terminate launch control early if the brake is pressed or there was a pedal fault
          lcSystem->getController()->setState(launchState::FINISHED, millis());
          break;
        }
        wheelSpeeds_s wheelSpeedData = {pedals->get_wsfl(), pedals->get_wsfr(), pm100->getmcMotorRPM(), pm100->getmcMotorRPM()};
        lcSystem->getController()->run(millis(), calculated_torque, wheelSpeedData);
        calculated_torque = lcSystem->getController()->getTorqueOutput();
        // Yeet data fast when running
        // GO FASTER THAN 20HZ TODO
        if (_100hz_send)
        {
          (lcSystem->getController()->getDiagData(), ID_VCU_BASE_LAUNCH_CONTROLLER_INFO);
        }
        break;
      }
      case launchState::FINISHED:
      {
        // Finished! exit launch control active!
        mcu_status.set_launch_ctrl_active(0);
        break;
      }
      }
    }
#if USE_INVERTER
    pm100->calc_and_send_current_limit(pm100->getmcBusVoltage(), DISCHARGE_POWER_LIM, CHARGE_POWER_LIM);
    pm100->command_torque(calculated_torque);
#endif
    break;
  }
  }
#if DEBUG
  if (debug_->check())
  {

    // int16_t motor_speed = 1000;
    // int max_torque = 1000;
    // pedals->calculate_torque(motor_speed, max_torque); // Just to print the calced value and confirm it's right
    // Put debug prints here if/when needed
    pm100->debug_print();
    accumulator->acc_debug_print();
    Serial.printf("\tDASH BUTTONS \nONE: %d TWO: %d THREE: %d FOUR: %d FIVE: %d SIX: %d\n", dash_->get_button1(), dash_->get_button2(), dash_->get_button3(), dash_->get_button4(), dash_->get_button5(), dash_->get_button6());
  }
#endif
}

// this sucks
void StateMachine::handle_distance_trackers(MCU_status &mcu_status)
{
#if DEBUG
  Serial.printf("==Handled distance tracker!==\n");
#endif
  bool _10s_timer_fired = _log_distance_timer_10s.check();
  // TODO uncomment the eeprom writes
  if (_10s_timer_fired)
  {
    unsigned long temporary_total_time = _lifetime_on_time + millis()/1000;
    // EEPROM.put(10,temporary_total_time);
    time_and_distance_t.vcu_lifetime_ontime = temporary_total_time;
    Serial.printf("Wrote total time: initial: %d millis: %d total: %d\n",_lifetime_on_time,millis()/1000,temporary_total_time);
  }

  if (mcu_status.get_state() == MCU_STATE::READY_TO_DRIVE)
  {
    distance_tracker_fl.update(accumulator->get_acc_current(), accumulator->get_acc_voltage(), pedals->get_wsfl(), WHEEL_CIRCUMFERENCE, millis());
    distance_tracker_motor.update(accumulator->get_acc_current(), accumulator->get_acc_voltage(), pm100->getmcMotorRPM() * FINAL_DRIVE, WHEEL_CIRCUMFERENCE, millis());
    mcu_status.set_distance_travelled(distance_tracker_motor.get_data().distance_m);
    if (_10s_timer_fired)
    {
      unsigned long temporary_total_distance = _lifetime_distance + distance_tracker_motor.get_data().distance_m;
      // EEPROM.put(ODOMETER_EEPROM_ADDR,temporary_total_distance);
      time_and_distance_t.vcu_lifetime_distance = temporary_total_distance;
      Serial.printf("Wrote total distance: initial: %d millis: %d total: %d",_lifetime_distance,distance_tracker_motor.get_data().distance_m,temporary_total_distance);
    }
  }
  else
  {
    distance_tracker_fl.tick(millis());
    distance_tracker_motor.tick(millis());
  }

}

// void StateMachine::joe_mock_lc(MCU_status* mcu_status, int torq, bool implaus)
// {
//   bool impl_occ = implaus;
//   int16_t calculated_torque = torq;
//       // Toggle launch control if button 2 & 6 are held for 1 second, while brake is pressed));
//     if (dash_->get_button_held_duration(LAUNCH_CONTROL_BUTTON, 1000)){
//       if (true)
//     {
//       // Toggle launch control (allows deactivating if sitting in it)
//       mcu_status->set_launch_ctrl_active(!(mcu_status->get_launch_ctrl_active()));
//       // Reset the launch controller each time we toggle it
//       lcSystem->getController()->initLaunchController(millis());
//       this->send_state_msg(*mcu_status);
//       Serial.printf("DEBUG: Set launch control to %d\n", mcu_status->get_launch_ctrl_active());
//     }
//     }
//     if (mcu_status->get_launch_ctrl_active())
//     {
//       // Do launch control things
//       switch (lcSystem->getController()->getState())
//       {
//       case launchState::IDLE:
//       {
//         calculated_torque=0; // Set torque to zero in IDLE
//         // If button is held for 1 second, APPS is floored (90%), brake is not active, and impl_occ is false
//         // THEN: go to WAITING_TO_LAUNCH
//         float travel = pedals->getAppsTravel();
//         Serial.printf("travel: %f brake %d impl %d\n",travel,mcu_status->get_brake_pedal_active(),impl_occ);
//         if (!dash_->get_button(2))
//         {
//           Serial.println("stage1");
//           bool dashbutton = dash_->get_button(6);
//           bool travelinrange = (travel > 0.7);
//           bool brakeactive = !(mcu_status->get_brake_pedal_active());
//           bool impl = !impl_occ;
//           if (dashbutton && travelinrange && brakeactive && impl)
//         {
//           Serial.println("JOE");
//           lcSystem->getController()->setState(launchState::WAITING_TO_LAUNCH,millis());
//           break;
//         }
//         }
//         break;
//       }
//       case launchState::WAITING_TO_LAUNCH:
//       {
//         calculated_torque=0; // Set torque to zero in WAITING_TO_LAUNCH
//         // If gas is released, return to IDLE
//         if ((pedals->getAppsTravel() < 0.5)) // TODO be less redundant?
//         {
//           lcSystem->getController()->setState(launchState::IDLE,millis());
//           break;
//         }
//         // If gas is still pinned and button has been released for 1000ms, start LAUNCHING
//         if ((pedals->getAppsTravel()> 0.9) && !dash_->get_button6() && dash_->get_button_released_duration(6,1000))
//         {
//           lcSystem->getController()->setState(launchState::LAUNCHING,millis());
//         }
//         break;
//       }
//       case launchState::LAUNCHING:
//       {
//         if ((mcu_status->get_brake_pedal_active()) || impl_occ)
//         {
//           // Terminate launch control early if the brake is pressed or there was a pedal fault
//           lcSystem->getController()->setState(launchState::FINISHED,millis());
//           break;
//         }
//         wheelSpeeds_s wheelSpeedData = {pedals->get_wsfl(),pedals->get_wsfr(),pm100->getmcMotorRPM(),pm100->getmcMotorRPM()};
//         lcSystem->getController()->run(millis(), calculated_torque, wheelSpeedData);
//         calculated_torque = lcSystem->getController()->getTorqueOutput();
//         break;
//       }
//       case launchState::FINISHED:
//       {
//         // Finished! exit launch control active!
//         mcu_status->set_launch_ctrl_active(0);
//         break;
//       }
//       }
//     Serial.printf("State: %d Resultant torque: %d\n",static_cast<int>(lcSystem->getController()->getState()),calculated_torque);
//     }
// }