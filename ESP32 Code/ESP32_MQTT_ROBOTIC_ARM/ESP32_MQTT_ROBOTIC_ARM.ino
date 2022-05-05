//=============================================================================================================================
#include "SPIFFS.h"
#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
//=============================================================================================================================
// Define the number of servos
#define SERVOS 4
#define SPEED 0.3
//=============================================================================================================================
const char* ssid = "***"; //Provide your SSID
const char* password = "***"; // Provide Password
const char* mqtt_server = "***"; // Provide localhost name of the broker devices
const char* mqtt_user = "***"; // Provide the username of the broker devices
const char* mqtt_pass = "***"; // Provide the password of the broker devices
const int mqtt_port = ***; // Provide the port number of the broker devices
//=============================================================================================================================
long lastMsg = 0;
byte mac[6];
char mac_Id[18];
//=============================================================================================================================
// Create the servo objects.
Servo servo[SERVOS];
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Attach servos to digital pins on the ESP32
int servo_pins[SERVOS] = {4, 17, 5, 18};
int servoAngle[4] = {0, 90, 120, 110};
int robot_controller = 0;
//=============================================================================================================================
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }

  payload[length] = '\0';
  int x = atoi((char *)payload);
  getTopicValue(x, topic);

  Serial.println();
  Serial.println("-----------------------");
}

void getTopicValue(int x, char* topic) {

  if (strcmp(topic, "robotics/baseArm") == 0) {
    servoAngle[0] = x;
  } else if (strcmp(topic, "robotics/shoulderArm") == 0) {
    servoAngle[1] = x;
  } else if (strcmp(topic, "robotics/elbowArm") == 0) {
    servoAngle[2] = x;
  } else if (strcmp(topic, "robotics/gripperArm") == 0) {
    servoAngle[3] = x;
  } else if (strcmp(topic, "robotics/controller") == 0) {
    robot_controller = x;
  } else if (strcmp(topic, "robotics/value") == 0 && robot_controller == 0) {
    if (x == 1.00) {
      pick_conveyor_one();
    }
    else if (x == 4) 
    {
      release_object();
    }
//    else if (x == 2.00) {
//      pick_conveyor_two();
//    } else if (x == 3.00) {
//      pick_conveyor_three();
//    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      Serial.println("Subscribe to the topic");
      client.subscribe("robotics/baseArm");
      client.subscribe("robotics/shoulderArm");
      client.subscribe("robotics/elbowArm");
      client.subscribe("robotics/gripperArm");
      client.subscribe("robotics/controller");
      client.subscribe("robotics/value");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(2000);
    }
  }
}
void setup() {
  Serial.begin(9600);
  setup_wifi();
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  for (int i = 0; i < SERVOS; i++) {
    // Attach the servos to the servo object
    servo[i].attach(servo_pins[i]);
  }
  home_arm();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;

    if (robot_controller == 1) {
      control_arm();
    }

  }
}

/* Function to control manually the 4DOF robot arm */
void control_arm() {
  base_arm(servoAngle[0]);
  shoulder_arm(servoAngle[1]);
  elbow_arm(servoAngle[2]);
  gripper_arm(servoAngle[3]);

}

/* Home position and message publish to capture picture when the object is in front of the ESP32 CAM */
void home_arm() {
  //client.publish("camera/capture", "Capture");
  delay(3000);
  shoulder_arm(90);
  elbow_arm(120);
  base_arm(0);
  gripper_arm(120);
}

/* Pick and Place function - pick object from conveyor three after radar scan (45 degree) and place in fron of the camera ESP32)
   publish mqtt message in order to capture a picture for object detection
*/
void pick_conveyor_three() {
  // Position of the Servo Motors for the first movement of the Robot Arm
  shoulder_arm(120);
  base_arm(45);
  elbow_arm(50);
  shoulder_arm(150);
  gripper_arm(120);
  elbow_arm(110);
  shoulder_arm(170);
  gripper_arm(170);
  elbow_arm(90);
  shoulder_arm(150);
  base_arm(170);
  shoulder_arm(170);
  elbow_arm(110);
  gripper_arm(120);
  shoulder_arm(90);
  home_arm();
}

void pick_conveyor_two() { // Position of the Servo Motors for the first movement of the Robot Arm
  shoulder_arm(120);
  base_arm(90);
  elbow_arm(50);
  shoulder_arm(150);
  gripper_arm(120);
  elbow_arm(110);
  shoulder_arm(170);
  gripper_arm(170);
  elbow_arm(90);
  shoulder_arm(150);
  base_arm(170);
  shoulder_arm(170);
  elbow_arm(110);
  gripper_arm(120);
  shoulder_arm(90);
  home_arm();
}

void pick_conveyor_one() { // Position of the Servo Motors for the first movement of the Robot Arm
  shoulder_arm(120);
  base_arm(120);
  elbow_arm(50);
  shoulder_arm(150);
  gripper_arm(120);
  elbow_arm(110);
  shoulder_arm(170);
  gripper_arm(170);
  elbow_arm(90);
  shoulder_arm(150);
  base_arm(170);
  shoulder_arm(170);
  client.publish("camera/capture", "Capture");
//  elbow_arm(110);
//  gripper_arm(120);
//  shoulder_arm(90);
//  home_arm();
}

void release_object()
{
  elbow_arm(110);
  gripper_arm(120);
  shoulder_arm(90);
  home_arm();
}

//=============================================================================================================================

/*
   Base (joint 0) function to move the arm form his current position to the input angle received
*/
void base_arm(int baseValue) {

  if (servoAngle[0] <= baseValue) {
    for (float pos = servoAngle[0]; pos <= baseValue; pos += SPEED) {
      servo[0].write(pos);
      delay(5);
      servoAngle[0] = pos;
    }
    return;
  }
  else if (servoAngle[0] > baseValue) {
    for (float pos = servoAngle[0]; pos >= baseValue; pos -= SPEED) {
      servo[0].write(pos);
      delay(5);
      servoAngle[0] = pos;
    }
    return;
  }
}

/*
   Shoulder (joint 1) function to move the arm form his current position to the input angle received
*/
void shoulder_arm(int shoulderValue) {

  if (servoAngle[1] <= shoulderValue) {
    for (float pos = servoAngle[1]; pos <= shoulderValue; pos += SPEED) {
      servo[1].write(pos);
      delay(5);
      servoAngle[1] = pos;
    }
    return;
  }
  else if (servoAngle[1] > shoulderValue) {
    for (float pos = servoAngle[1]; pos >= shoulderValue; pos -= SPEED) {
      servo[1].write(pos);
      delay(5);
      servoAngle[1] = pos;
    }
    return;
  }

}

/*
   Elbow (joint 2) function to move the arm form his current position to the input angle received
*/
void elbow_arm(int elbowValue) {

  if (servoAngle[2] <= elbowValue) {
    for (float pos = servoAngle[2]; pos <= elbowValue; pos += SPEED) {
      servo[2].write(pos);
      delay(5);
      servoAngle[2] = pos;
    }
    return;
  }
  else if (servoAngle[2] > elbowValue) {
    for (float pos = servoAngle[2]; pos >= elbowValue; pos -= SPEED) {
      servo[2].write(pos);
      delay(5);
      servoAngle[2] = pos;
    }
    return;
  }
}


/*
   Gripper (joint 3) function to move the arm form his current position to the input angle received
*/
void gripper_arm(int gripperValue) {

  if (servoAngle[3] <= gripperValue) {
    for (float pos = servoAngle[3]; pos <= gripperValue; pos += SPEED) {
      servo[3].write(pos);
      delay(5);
      servoAngle[3] = pos;
    }
    return;
  }
  else if (servoAngle[3] > gripperValue) {
    for (float pos = servoAngle[3]; pos >= gripperValue; pos -= SPEED) {
      servo[3].write(pos);
      delay(5);
      servoAngle[3] = pos;
    }
    return;
  }

}
