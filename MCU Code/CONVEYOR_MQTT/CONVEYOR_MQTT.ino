/*
 * @Author: LabIoT Team
 * @Date: 2022-06-17 11:35
 * @Description: MQTT Conveyor
 */
//=============================================================================================================================
#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"
#include <string.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
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
//=============================================================================================================================
long lastMsg = 0;
int conveyor_speed = 150;
int IR = 10;
int objectDetectIR;
String conveyor_controller = "deactivated";
String pickObject = "pick_on";
//=============================================================================================================================
void setup() 
{
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  Serial.print("Client Ready! Use '");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  pinMode(IR, INPUT);
}

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

  if (String(topic) == "conveyor" && messageTemp == "active") {
    //conveyor_controller = "active";
    Application_FunctionSet.conveyor_control("Backward", conveyor_speed);
  }
  else if (String(topic) == "conveyor" && messageTemp == "deactivate") {
    //conveyor_controller = "deactivated";
    Application_FunctionSet.conveyor_control("stop_it", 0);
  }
  else if (String(topic) == "car/conveyor" && messageTemp == "pick_on") {
    pickObject = "pick_on";
  }
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Arduino Robotic Conveyor")) {
      Serial.println("connected");
      client.subscribe("conveyor");
      client.subscribe("car/conveyor");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
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

  objectDetectIR = digitalRead(IR);

  if (objectDetectIR == LOW && pickObject == "pick_on") {
    Application_FunctionSet.conveyor_control("stop_it", 0);
    client.publish("conveyor/object", "green");
    client.publish("arm/conveyor", "pick");
    pickObject = "pick_off";
  }
}
