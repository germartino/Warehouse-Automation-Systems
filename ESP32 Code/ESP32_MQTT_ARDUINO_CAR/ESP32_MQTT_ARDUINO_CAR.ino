// ARDUINO UNO RECEIVER
//COM 3

#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

char control = '0';
int car_speed = 50;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Application_FunctionSet.ApplicationFunctionSet_Init();
}
void loop() { // run over and over
  delay(10);
  while (Serial.available() <= 0) {
    Application_FunctionSet.ApplicationFunctionSet_Tracking();
    Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
  }

  while (Serial.available() > 0) {
    control = Serial.read();
    Serial.print("Control:");
    Serial.println(control);
    Serial.flush();
    
    Application_FunctionSet.car_control(control, car_speed);
  }
}
