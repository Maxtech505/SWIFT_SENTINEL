

#include <WiFi.h>
#include "esp_camera.h"
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"
#include "credentials.h"
#include <PubSubClient.h>
#include <base64.h>

/******************************** Declare Variables ********************************/
// Adafruit feeds for publishing
const char* mqttTopicSnap = "MREGALADO2/feeds/SentinelSnap";
const char* mqttTopicSnap2 = "MREGALADO2/feeds/SentinelSnap2";
const int MAX_PUBLISH = 100 * 1024;    // Adafruit's limit is 100 kB

// Publishing cycle delay
const int PUBLISH_DELAY = 30 * 1000; // 30 seconds between publishing cycles

// Image storage
char strOut[1024];
char strOut2[1024];
String buffer, buffer2;

/******************************** Create Objects ********************************/
WiFiClient espClient;
PubSubClient client(espClient);

/******************************** Create Prototypes ********************************/
void mqtt_setup();
void callback(char* topic, byte* payload, unsigned int length);

void setup() {
  Serial.begin(115200);
  pinMode(14,INPUT);
  Serial.setDebugOutput(true);
  Serial.println("Serial is up!");
  
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
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 45;
  config.fb_count = 1;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }
  
  sensor_t * s = esp_camera_sensor_get();
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setSleep(false);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  client.setServer(AIO_SERVER, AIO_SERVERPORT);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    mqtt_setup();
  }
  client.loop();

  // Snaps the first picture
  camera_fb_t *fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  
  buffer = base64::encode((uint8_t *)fb->buf, fb->len);
  sprintf(strOut, "Got frame: %d x %d (%d/%d)", fb->width, fb->height, fb->len, buffer.length());
  Serial.printf("%s\n", strOut);
  
  if (buffer.length() < MAX_PUBLISH) {
    if (client.publish(mqttTopicSnap, buffer.c_str())) {
      Serial.print("Published Image 1 to ");
      Serial.print(mqttTopicSnap);
      Serial.printf("\n");
      Serial.printf("Published %d bytes (from %d)\n", buffer.length(), fb->len);
    } else {
      Serial.println("Error Publishing Image");
    }
  } else {
    Serial.println("Over limit on image 1 - We'll try to publish the next pic");
  }

  esp_camera_fb_return(fb);
  buffer.clear();
  
  // A short delay between the two pictures is good practice
  delay(100);

  // Snaps the second picture
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed for image 2");
    return;
  }

  buffer2 = base64::encode((uint8_t *)fb->buf, fb->len);
  sprintf(strOut2, "Got frame2: %d x %d (%d/%d)", fb->width, fb->height, fb->len, buffer2.length());
  Serial.printf("%s\n", strOut2);

  if (buffer2.length() < MAX_PUBLISH) {
    if (client.publish(mqttTopicSnap2, buffer2.c_str())) {
      Serial.print("Published Image 2 to ");
      Serial.print(mqttTopicSnap2);
      Serial.printf("\n");
      Serial.printf("Published2 %d bytes (from %d)\n", buffer2.length(), fb->len);
    } else {
      Serial.println("Error Publishing Image2");
    }
  } else {
    Serial.println("Over limit on image 2 - We'll try to publish the next pic");
  }

  esp_camera_fb_return(fb);
  buffer2.clear();

  // Wait for the full 30 seconds before the loop starts over
  delay(PUBLISH_DELAY);
}

/******************************** Define Functions *********************************/
void mqtt_setup() {
  const int MQTT_BUFFERSIZE = 50 * 1024;
  client.setBufferSize((uint16_t)MQTT_BUFFERSIZE);
  client.setServer(AIO_SERVER, AIO_SERVERPORT);
  client.setCallback(callback);
  Serial.println("Connecting to MQTT…");

  while (!client.connected()) {
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), AIO_USERNAME, AIO_KEY )) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state  ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  String byteRead = "";
  Serial.print("Message: ");
  for (int i = 0; i < length; i++) {
    byteRead += (char)payload[i];
  }
  Serial.println(byteRead);
}