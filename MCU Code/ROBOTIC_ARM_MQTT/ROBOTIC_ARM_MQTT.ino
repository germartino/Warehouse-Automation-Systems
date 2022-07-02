/*
   @Author: LabIoT Team
   @Date: 2022-06-17 11:55
   @Description: ESP32 MQTT ROBOTIC ARM
*/
//=============================================================================================================================
#include <Wire.h> // Include Wire Library for I2C Communications
#include <WiFiClient.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_PWMServoDriver.h> // Include Adafruit PWM Library
//=============================================================================================================================
#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50
//=============================================================================================================================
const char* ssid = ""; //Provide your SSID
const char* password = ""; // Provide Password
const char* mqtt_server = ""; // Provide localhost name of the broker devices
const char* mqtt_user = ""; // Provide the username of the broker devices
const char* mqtt_pass = ""; // Provide the password of the broker devices
const int mqtt_port = 1883; // Provide the port number of the broker devices (1883, 8883)
//=============================================================================================================================
long lastMsg = 0;
String arm_controller = "Automatic";
// Define Motor Outputs on PCA9685 board
int motorBase = 0;
int motorShoulder = 1;
int motorElbow = 2;
int motorWristPitch = 3;
int motorWristRoll = 4;
int motorGripper = 5;
int servoAngle[6] = {10, 30, 190, 190, 90, 105};
int currentPos[6] = {152, 191, 500, 500, 307, 336};
int delayTime = 10;
//=============================================================================================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
WiFiClient espClient;
PubSubClient client(espClient);
//=============================================================================================================================

void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  Serial.println();

  if (String(topic) == "arm/controller" && messageTemp == "Manual") {
    arm_controller = "Manual";
  }
  else if (String(topic) == "arm/controller" &&messageTemp == "Automatic") {
    arm_controller = "Automatic";
  }

  if (arm_controller == "Manual") {
    if (String(topic) == "arm/baseArm") {
      servoAngle[0] = messageTemp.toInt();
      moveMotorDeg(servoAngle[0], motorBase, delayTime);
    } else if (String(topic) == "arm/shoulderArm") {
      servoAngle[1] = messageTemp.toInt();
      moveMotorDeg(servoAngle[1], motorShoulder, delayTime);
    } else if (String(topic) == "arm/elbowArm") {
      servoAngle[2] = messageTemp.toInt();
      moveMotorDeg(servoAngle[2], motorElbow, delayTime);
    } else if (String(topic) == "arm/wristPitchArm") {
      servoAngle[3] = messageTemp.toInt();
      moveMotorDeg(servoAngle[3], motorWristPitch, delayTime);
    } else if (String(topic) == "arm/wristRollArm") {
      servoAngle[4] = messageTemp.toInt();
      moveMotorDeg(servoAngle[4], motorWristRoll, delayTime);
    } else if (String(topic) == "arm/gripperArm") {
      servoAngle[5] = messageTemp.toInt();
      moveMotorDeg(servoAngle[5], motorGripper, delayTime);
    }
  }
  else if (arm_controller == "Automatic") {
    if (String(topic) == "arm/conveyor" && messageTemp == "pick") {
      pick();
    } else if (String(topic) == "arm/car" && messageTemp == "place") {
      place();
    }
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Arduino Robotic Arm")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("arm/baseArm");
      client.subscribe("arm/shoulderArm");
      client.subscribe("arm/elbowArm");
      client.subscribe("arm/wristPitchArm");
      client.subscribe("arm/wristRollArm");
      client.subscribe("arm/gripperArm");
      client.subscribe("arm/controller");
      client.subscribe("arm/conveyor");
      client.subscribe("arm/car");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Setup PWM Controller object
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
}

// Function to move motor to specific position
void moveMotorDeg(int moveDegree, int motorOut, int delayTime)
{
  int pulse_wide, pulse_width;

  // Convert to pulse width
  pulse_wide = map(moveDegree, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);

  //Control Motor
  if (currentPos[motorOut] == pulse_width) {
    return;
  }
  else if (pulse_width > currentPos[motorOut]) {
    for (int pos = currentPos[motorOut]; pos < pulse_width; pos++) {
      pwm.setPWM(motorOut, 0, pos);
      delay(delayTime);
    }
  }
  else if (pulse_width < currentPos[motorOut]) {
    for (int pos = currentPos[motorOut]; pos > pulse_width; pos--) {
      pwm.setPWM(motorOut, 0, pos);
      delay(delayTime);
    }
  }

  currentPos[motorOut] = pulse_width;
  servoAngle[motorOut] = moveDegree;

  Serial.println("Analogue Servo Position: ");
  Serial.println(pulse_width);

}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
  }
}

void pick()
{
  home_arm();

  moveMotorDeg(150, motorBase, delayTime);
  moveMotorDeg(190, motorElbow, delayTime);
  moveMotorDeg(110, motorGripper, delayTime);
  moveMotorDeg(100, motorWristPitch, delayTime);
  moveMotorDeg(90, motorWristRoll, delayTime);
  moveMotorDeg(85, motorShoulder, delayTime);
  moveMotorDeg(165, motorGripper, delayTime);
  moveMotorDeg(30, motorShoulder, delayTime);
  moveMotorDeg(60, motorBase, delayTime);
  moveMotorDeg(125, motorWristPitch, delayTime);
  moveMotorDeg(63, motorShoulder, delayTime);
  moveMotorDeg(90, motorBase, delayTime);
  delay(5000);
  client.publish("arm/camera", "take");
}

void place()
{
  moveMotorDeg(60, motorBase, delayTime);
  moveMotorDeg(30, motorShoulder, delayTime);
  moveMotorDeg(115, motorBase, delayTime);
  moveMotorDeg(50, motorShoulder, delayTime);
  moveMotorDeg(170, motorElbow, delayTime);
  moveMotorDeg(75, motorShoulder, delayTime);
  moveMotorDeg(150, motorElbow, delayTime);
  moveMotorDeg(100, motorShoulder, delayTime);
  moveMotorDeg(110, motorGripper, delayTime);
  moveMotorDeg(60, motorShoulder, delayTime);
  moveMotorDeg(190, motorElbow, delayTime);
  moveMotorDeg(40, motorShoulder, delayTime);
  moveMotorDeg(190, motorWristPitch, delayTime);
  moveMotorDeg(30, motorShoulder, delayTime);
  moveMotorDeg(10, motorBase, delayTime);
  home_arm();
  client.publish("arm/car", "car_on");
}



void home_arm()
{
  moveMotorDeg(10, motorBase, delayTime);
  moveMotorDeg(30, motorShoulder, delayTime);
  moveMotorDeg(190, motorElbow, delayTime);
  moveMotorDeg(190, motorWristPitch, delayTime);
  moveMotorDeg(90, motorWristRoll, delayTime);
  moveMotorDeg(110, motorGripper, delayTime);
}
