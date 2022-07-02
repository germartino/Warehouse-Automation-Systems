/*
 * @Author: LabIoT Team
 * @Date: 2022-06-17 11:35
 * @Description: MQTT Car
 */
//=============================================================================================================================
#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"
#include <string.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Servo.h>
//============================================================================================================================
const char* ssid = ""; //Provide your SSID
const char* password = ""; // Provide Password
const char* mqtt_server = ""; // Provide localhost name of the broker devices
const char* mqtt_user = ""; // Provide the username of the broker devices
const char* mqtt_pass = ""; // Provide the password of the broker devices
const int mqtt_port = 1883; // Provide the port number of the broker devices (1883, 8883)
//=============================================================================================================================
WiFiClient wifiClient;
PubSubClient client(wifiClient);
Servo servo_cam;
//=============================================================================================================================
long lastMsg = 0;
String activate_robotic_car_parameters = "param_off";
String activate_robotic_car_control = "Automatic";
char tempDirection = 'i';
String qr_code;
int car_speed = 100;
int car_distance = 0;
int car_direction = 0;
int car_speed_hi = 0;
int car_speed_low = 0;
const char* car_area = "";
int servo_val = 0;
boolean stopAfterReturnPath = false;
boolean cameraSleep = false;
boolean objectPlaced = false;
boolean carStarts = false;
int path_finder = 0;
int servo_pin = 11;
int pos = 0;
String placeObject = "place_on";

//=============================================================================================================================
void setup() {
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  Serial.print("Client Ready! Use '");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  path_finder = 0;
  qr_code = "";
  servo_cam.attach(servo_pin);
  Application_FunctionSet.ApplicationFunctionSet_Init();
}

void setup_wifi() {
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

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  if (String(topic) == "car/control" && messageTemp == "Manual") {
    activate_robotic_car_control = "Manual";
  }
  else if (String(topic) == "car/control" && messageTemp == "Automatic") {
    activate_robotic_car_control = "Automatic";
  }
  if (String(topic) == "car/parameters" && messageTemp == "param_on") {
    activate_robotic_car_parameters = "param_on";
  }
  else if (String(topic) == "car/parameters" && messageTemp == "param_off") {
    activate_robotic_car_parameters = "param_off";
  }
  Serial.println();

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (activate_robotic_car_control == "Manual") {
    if (messageTemp == "Forward") {
      //Serial.println(messageTemp);
      Application_FunctionSet.car_control("Forward", car_speed);
    }
    else if (messageTemp == "Backward") {
      //Serial.println(messageTemp);
      Application_FunctionSet.car_control("Backward", car_speed);
    }
    else if (messageTemp == "Left") {
      Serial.println(messageTemp);
      Application_FunctionSet.car_control("Left", car_speed);
    }
    else if (messageTemp == "Right") {
      Serial.println(messageTemp);
      Application_FunctionSet.car_control("Right", car_speed);
    }
    else if (messageTemp == "LeftForward") {
      Serial.println(messageTemp);
      Application_FunctionSet.car_control("LeftForward", car_speed);
    }
    else if (messageTemp == "LeftBackward") {
      Serial.println(messageTemp);
      Application_FunctionSet.car_control("LeftBackward", car_speed);
    }
    else if (messageTemp == "RightForward") {
      Serial.println(messageTemp);
      Application_FunctionSet.car_control("RightForward", car_speed);
    }
    else if (messageTemp == "RightBackward") {
      Serial.println(messageTemp);
      Application_FunctionSet.car_control("RightBackward", car_speed);
    }
    else if (messageTemp == "stop_it") {
      Serial.println(messageTemp);
      Application_FunctionSet.car_control("stop_it", car_speed);
    }
  }
  else if (activate_robotic_car_control == "Automatic") {
    if (String(topic) == "car/camera") {
      Serial.print("messagetemp: ");
      Serial.println(messageTemp);
      qr_code = messageTemp;
      placeObject = "place_on";
      objectPlaced = false;
      Serial.println(qr_code);
    }
    else if (String(topic) == "arm/car" && messageTemp == "car_on") {
      Serial.print("messagetemp: ");
      Serial.println(messageTemp);
      objectPlaced = true;
    }
  }
  if (activate_robotic_car_parameters == "param_on") {
    if (stringStartsWith(messageTemp, "{") == 1) { //if it's a JSON...
      const char* json = messageTemp.c_str();
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, json);

      car_direction = doc["direction"];
      car_distance = doc["distance"];
      car_speed_hi = doc["speed_hi"];
      car_speed_low = doc["speed_low"];
      car_area = doc["area"];
      servo_val = doc["servo"];
      servo_cam.write(servo_val);

      Serial.println(car_direction);
      Serial.println(car_distance);
      Serial.println(car_speed_hi);
      Serial.println(car_speed_low);
      Serial.println(car_area);
      Serial.println(servo_val);
    }
  }
  else if (activate_robotic_car_parameters == "param_off") {
    return;
  }
}

boolean stringStartsWith(String string, const char *starter) {
  const char* converted_string = string.c_str();
  if (strncmp(converted_string, starter, strlen(starter)) == 0) return 1;
  return 0;
}

boolean strEquals(String string1, const char *string2) {
  const char* converted_string = string1.c_str();
  if (strcmp(converted_string, string2) == 0) return 1;
  return 0;
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Arduino Robotic Car")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("car/control");
      client.subscribe("car/camera");
      client.subscribe("car/parameters");
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

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //Serial.println(car_direction);
  //if initialization parameters arrived, the car can starts
  if (car_direction != 0) {
    carStarts = true;
  }

  if (carStarts && qr_code != "") {
    Serial.println("Start!");
    //    cameraSleep = true;
    //    if(cameraSleep) {
    //      servo_cam.write(180);
    //      cameraSleep = false;
    //    }

    if (placeObject == "place_on") {
      client.publish("arm/car", "place");
      placeObject = "place_off";
    }
    else if (objectPlaced) {
      //here we must put the code that will let the car search and then follow the line
      while (path_finder == 0) {
        path_finder = Application_FunctionSet.find_path(qr_code[0], path_finder);
      }

      while (path_finder == 1) {
        path_finder = Application_FunctionSet.ApplicationFunctionSet_Tracking();
      }

      if (path_finder == 2 && stopAfterReturnPath == false) {
        delay(5000);
        path_finder = Application_FunctionSet.moveIt('A', car_direction, car_distance, car_speed_hi);
        stopAfterReturnPath = true;
        //qr_code = '0';
      }

      else if (stopAfterReturnPath == true) {
        char tempValue = "";
        tempValue = qr_code.c_str(); 
        path_finder = 0;
        stopAfterReturnPath = false;
        client.publish("car/conveyor", "pick_on");
        client.publish("conveyor/object", "red");
        client.publish("conveyor", "active");
        client.publish("car/object", tempValue);
        qr_code = ""; 
        objectPlaced = false;
      }
    }
  }

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
  }
}
