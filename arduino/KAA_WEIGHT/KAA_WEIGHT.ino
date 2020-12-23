//#include <ESP8266WiFi.h>
#include <M5StickC.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "hx711.h"
#include "secrets.h"

#define FRONT 1

const char* ssid = SECRET_SSID;        // WiFi name
const char* password = SECRET_PASSWORD;    // WiFi password
const char* mqtt_server = "mqtt.cloud.kaaiot.com";
const String TOKEN = SECRET_TOKEN;        // Endpoint token - you get (or specify) it during device provisioning
const String APP_VERSION = SECRET_APP_VERSION;  // Application version - you specify it during device provisioning

const unsigned long fiveSeconds = 1 * 5 * 1000UL;
static unsigned long lastPublish = 0 - fiveSeconds;

WiFiClient espClient;
PubSubClient client(espClient);

HX711 scale(33, 32);

void setup() {
  M5.begin();
  Serial.begin(115200);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  M5.Lcd.setRotation(3);
  M5.Lcd.setCursor(2,11,FRONT);
  M5.Lcd.print("Btn_A to tare");
}

float weight;

void loop() {
  
  setup_wifi();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastPublish >= fiveSeconds) {
    lastPublish += fiveSeconds;
    DynamicJsonDocument telemetry(1023);
    telemetry.createNestedObject();
    telemetry[0]["temperature"] = random(18, 23);
    telemetry[0]["humidity"] = random(40, 60);
    telemetry[0]["co2"] = random(900, 1200);
    telemetry[0]["weight"] = weight;

    String topic = "kp1/" + APP_VERSION + "/dcx/" + TOKEN + "/json";
    client.publish(topic.c_str(), telemetry.as<String>().c_str());
    Serial.println("Published on topic: " + topic);
  }

  M5.update();
  if (M5.BtnA.wasReleased()) {
    scale.setOffset(scale.averageValue());
  }
  weight =scale.getGram();
  Serial.println(weight);
  M5.Lcd.setCursor(2,21,FRONT);
  M5.Lcd.printf("                             ");
  M5.Lcd.setCursor(2,31,FRONT);
  M5.Lcd.printf("weight:%0.2f g     ",weight);
  M5.Lcd.setCursor(2,41,FRONT);
  Serial.println(WiFi.localIP());
  
  M5.Lcd.printf("deviceIP: ");
  M5.Lcd.printf("%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );

}

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
