#include <M5StickC.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "hx711.h"
#include "secrets.h"

#define FRONT 1

const char* ssid = SECRET_SSID;                     // WiFi name
const char* password = SECRET_PASSWORD;             // WiFi password
const char* mqtt_server = "mqtt.cloud.kaaiot.com";
const String TOKEN = SECRET_TOKEN;                  // Endpoint token - you get (or specify) it during device provisioning
const String APP_VERSION = SECRET_APP_VERSION;      // Application version - you specify it during device provisioning

const unsigned long fiveSeconds = 1 * 5 * 1000UL;
static unsigned long lastPublish = 0 - fiveSeconds;

// Initialize communications
WiFiClient espClient;
PubSubClient client(espClient);

HX711 scale(33, 32);    // initialize scale

void setup() {
  M5.begin();
  
  Serial.begin(115200);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  //M5.IMU.Init();
  M5.MPU6886.Init();
  M5.MPU6886.SetGyroFsr(M5.MPU6886.GFS_500DPS);
  M5.MPU6886.SetAccelFsr(M5.MPU6886.AFS_4G);
  
  M5.Lcd.setRotation(3);
  M5.Lcd.setCursor(2,2,FRONT);
  M5.Lcd.print("Welcome to the ");
  M5.Lcd.setCursor(2,11,FRONT);
  M5.Lcd.print("Guess the Weight Game!");
  M5.Lcd.setCursor(2,21,FRONT);
  M5.Lcd.print("Designed by Peter So");
  M5.Lcd.setCursor(2,31,FRONT);
  M5.Lcd.printf("deviceIP: ");
  M5.Lcd.printf("%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
  M5.Lcd.setCursor(2,41,FRONT);
  M5.Lcd.print("Btn_M5 to tare");
  delay(1000); // temp delay to see how m5 might clog publish cmd
}

float weight = 0.0F;
float temp = 0.0F;
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;
int gameOver = -1;

float target_weight = 500.0F;
float guessedWeight = 0.0F;
float difference = 0.0F;

void loop() {   // loops forever
  
  setup_wifi();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Get device accel data
  M5.MPU6886.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.MPU6886.getAccelData(&accX, &accY, &accZ);
  M5.MPU6886.getTempData(&temp);

  unsigned long now = millis();
  if (now - lastPublish >= fiveSeconds) {   // send a report every 5 secs
    lastPublish += fiveSeconds;
    if (gameOver == 1) {
      sendReport();
    }
  }

  M5.update();

  if (M5.BtnA.wasReleased()) {
    scale.setOffset(scale.averageValue());
    gameOver == 0;
  }

  if (M5.BtnB.wasReleased()) {
    M5.Lcd.setCursor(2,31, FRONT);
    M5.Lcd.printf("Checking weight............");
    delay(1000);
    guessedWeight = weight;
    M5.Lcd.setCursor(2,41, FRONT);
    M5.Lcd.printf("Guessed weight was: %0.2f", guessedWeight);
    gameOver = 1;
    delay(3000);
  }

  weight =scale.getGram();
  Serial.print("curr weight: ");
  Serial.println(weight);
  difference = target_weight - weight;
  Serial.print("weight diff: ");
  Serial.println(difference);

  updateDisplay();

}

// user-defined functions
void updateDisplay() {
  M5.Lcd.setCursor(2,2,FRONT);
  //Serial.println(WiFi.localIP());
  M5.Lcd.printf("deviceIP: ");
  M5.Lcd.printf("%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
  M5.Lcd.setCursor(2,11,FRONT);
  M5.Lcd.printf("Match the weight: %0.1f g", target_weight);
  M5.Lcd.setCursor(2,21,FRONT);
  if (gameOver == -1){
    M5.Lcd.printf("Press M5 btn to start");
  } 
  if (gameOver == 0){
    M5.Lcd.printf("Press B btn to make guess");
  } else {
    M5.Lcd.printf("Last guess was: %0.2f", guessedWeight);
  }
  M5.Lcd.setCursor(2,31,FRONT);
  M5.Lcd.printf("current weight: %0.2f g", weight);
  M5.Lcd.setCursor(2,41,FRONT);
  M5.Lcd.printf("you are off by: %0.2f g", difference);
  M5.Lcd.setCursor(2,51,FRONT);
  M5.Lcd.printf("temp: %0.2f degC    ", temp);
  M5.Lcd.setCursor(2,61,FRONT);
  M5.Lcd.printf("accZ: %5.2f G    ", accZ);

  M5.Rtc.GetBm8563Time();
  M5.Lcd.setCursor(2, 71,FRONT);
  M5.Lcd.printf("%02d : %02d : %02d", M5.Rtc.Hour, M5.Rtc.Minute, M5.Rtc.Second);
}

void sendReport() {
  DynamicJsonDocument telemetry(1023);
  telemetry.createNestedObject();
  telemetry[0]["temperature"] = temp;
  telemetry[0]["humidity"] = random(40, 60);
  telemetry[0]["co2"] = random(900, 1200);
  telemetry[0]["weight"] = weight;
  telemetry[0]["accX"] = accX;
  telemetry[0]["accY"] = accY;
  telemetry[0]["accZ"] = accZ;
  telemetry[0]["gyroX"] = gyroX;
  telemetry[0]["gyroY"] = gyroY;
  telemetry[0]["gyroZ"] = gyroZ;
  telemetry[0]["roll"] = roll;
  telemetry[0]["pitch"] = pitch;
  telemetry[0]["yaw"] = yaw;

  String topic = "kp1/" + APP_VERSION + "/dcx/" + TOKEN + "/json";
  client.publish(topic.c_str(), telemetry.as<String>().c_str());
  //Serial.println("Published on topic: " + topic);
}

// boilerplate functions
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("\nHandling command message on topic: %s\n", topic);

  DynamicJsonDocument doc(1023);
  deserializeJson(doc, payload, length);
  JsonVariant json_var = doc.as<JsonVariant>();

  DynamicJsonDocument commandResponse(1023);
  for (int i = 0; i < json_var.size(); i++) {
    unsigned int command_id = json_var[i]["id"].as<unsigned int>();
    commandResponse.createNestedObject();
    commandResponse[i]["id"] = command_id;
    commandResponse[i]["statusCode"] = 200;
    commandResponse[i]["payload"] = "done";
  }

  String responseTopic = "kp1/" + APP_VERSION + "/cex/" + TOKEN + "/result/SWITCH";
  client.publish(responseTopic.c_str(), commandResponse.as<String>().c_str());
  Serial.println("Published response to SWITCH command on topic: " + responseTopic);
}

void setup_wifi() {
  if (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.println();
    Serial.printf("Connecting to [%s]", ssid);
    WiFi.begin(ssid, password);
    connectWiFi();
  }
}

void connectWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    char *client_id = "client-id-123ab";
    if (client.connect(client_id)) {
      Serial.println("Connected to WiFi");
      // ... and resubscribe
      subscribeToCommand();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void subscribeToCommand() {
  String topic = "kp1/" + APP_VERSION + "/cex/" + TOKEN + "/command/SWITCH/status";
  client.subscribe(topic.c_str());
  Serial.println("Subscribed on topic: " + topic);
}
