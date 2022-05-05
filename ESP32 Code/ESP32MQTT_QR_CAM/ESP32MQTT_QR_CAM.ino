#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>
#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
#include "camera_pins.h"
#define RXP2 33
#define TXP2 4
//=============================================================================================================================
const char* ssid = "***"; //Provide your SSID
const char* password = "***"; // Provide Password
const char* mqtt_server = "***"; // Provide localhost name of the broker devices
const char* mqtt_user = "***"; // Provide the username of the broker devices
const char* mqtt_pass = "***"; // Provide the password of the broker devices
const int mqtt_port = ***; // Provide the port number of the broker devices (1883, 8883)
//=============================================================================================================================
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
char tempDirection = '0';
//=============================================================================================================================
void startCameraServer();
//=============================================================================================================================
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXP2, TXP2);
  camera_setup();
  setup_wifi();
  
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  
  startCameraServer();
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
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
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == "car/control") {
    if (messageTemp == "Forward") {
      tempDirection = '1';
      Serial2.write(tempDirection);
    }
    else if (messageTemp == "Backward") {
      tempDirection = '2';
      Serial2.write(tempDirection);
    }
    else if (messageTemp == "Left") {
      tempDirection = '3';
      Serial2.write(tempDirection);
    }
    else if (messageTemp == "Right") {
      tempDirection = '4';
      Serial2.write(tempDirection);
    }
    else if (messageTemp == "LeftForward") {
      tempDirection = '5';
      Serial2.write(tempDirection);
    }
    else if (messageTemp == "LeftBackward") {
      tempDirection = '6';
      Serial2.write(tempDirection);
    }
    else if (messageTemp == "RightForward") {
      tempDirection = '7';
      Serial2.write(tempDirection);
    }
    else if (messageTemp == "RightBackward") {
      tempDirection = '8';
      Serial2.write(tempDirection);
    }
    else if (messageTemp == "stop_it") {
      tempDirection = '9';
      Serial2.write(tempDirection);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32CAM")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("car/control");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void camera_setup()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
  }
}
