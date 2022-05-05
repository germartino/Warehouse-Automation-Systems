/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-19 15:46:13
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <arduino.h>

class ApplicationFunctionSet
{
public:
  void ApplicationFunctionSet_Init(void);
  void ApplicationFunctionSet_Tracking(void);           //循迹
  void ApplicationFunctionSet_SensorDataUpdate(void);   //传感器数据更新
  void ApplicationFunctionSet_SerialPortDataAnalysis(void);
  void ApplicationFunctionSet::car_stop(void);
  void ApplicationFunctionSet::car_round(void);
  void ApplicationFunctionSet::car_goes(void);
  void ApplicationFunctionSet::car_control(char val,int car_speed);
  

  
private:
  volatile float TrackingData_L;       //循迹数据
  volatile float TrackingData_M;       //循迹数据
  volatile float TrackingData_R;

  boolean TrackingDetectionStatus_R = false;
  boolean TrackingDetectionStatus_M = false;
  boolean TrackingDetectionStatus_L = false;

public:
  boolean Car_LeaveTheGround = true;

public:
  uint16_t TrackingDetection_S = 250;
  uint16_t TrackingDetection_E = 850;
  uint16_t TrackingDetection_V = 950;
};
extern ApplicationFunctionSet Application_FunctionSet;
#endif
