/*
 * @Author: LabIoT Team
 * @Date: 2022-06-17 11:35
 * @Description: MQTT Conveyor
 */
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

DeviceDriverSet_Motor AppMotor;

enum SmartRobotConveyorControl
{
  Forward,       //(1)
  Backward,      //(2)
  stop_it        //(9)
};

void ApplicationFunctionSet_SmartRobotConveyorControl(SmartRobotConveyorControl direction, uint8_t is_speed);

static void ApplicationFunctionSet_SmartRobotConveyorControl(SmartRobotConveyorControl direction, uint8_t is_speed)
{
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;
  Kp = 10;
  UpperLimit = 255;

  switch (direction)
  {
    case Forward:
        AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
            /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
      break;
    case Backward:
        AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
            /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
      break;
      case stop_it:
      /* code */
      directionRecord = 9;
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
          /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
      break;
  }
}

void ApplicationFunctionSet::conveyor_control(String control, int car_speed)
{
  if (control == "Forward") {
    ApplicationFunctionSet_SmartRobotConveyorControl(Forward, car_speed);
  }
  else if (control == "Backward") {
    ApplicationFunctionSet_SmartRobotConveyorControl(Backward, car_speed);
  }
  else if (control == "stop_it") {
    ApplicationFunctionSet_SmartRobotConveyorControl(stop_it, car_speed);
  }
}
