// This code is copy paste from https://docs.kaaiot.io/KAA/docs/current/Tutorials/device-integration/hardware-guides/esp32-ota-updates/
// Minor edits to run on M5StickCPlus and update screen
//
// This works with the over the air feature with kaa.
// Take EXTREME CARE when copying the .bin link from the Kaa UI File 
// you will need to trim off the extra strings after .bin and remove the "s"
// to make "http" instead of "https". Verify against the suggested text in 
// the Software OTA add new version Download Link field.
//
// Test Procedure: Go to Kaa UI/Devices/Software OTA and find version 0.0.91
// Verify the targets to upgrade include this version which is 0.0.2.
// Also verify that the update is enabled for "All" devices. Click Update.
// Flash this file to your M5StickPlus device and keep the Serial Monitor open
// Device will (1) power up and report its current version, device will display a mostly black screen and show FW:0.0.2 (2) request new versions (3) Attempt to download the newer version and you should see the task board screen with FW version 0.0.91.
// Congrats! You got the OTA feature to work!

#include <M5StickCPlus.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

const String FW_VERSION = "0.0.2"; // Firmware Version for OTA management

const char* ssid = "FRITZ!Box 7530 WE";        // WiFi name
const char* password = "70348511462386919316";    // WiFi password

const char* mqttServer = "mqtt.cloud.kaaiot.com";

const String TOKEN = "task-board-peter";        // Endpoint token - you get (or specify) it during device provisioning
const String APP_VERSION = "c1v9jqmgul2l1s47m6bg-v0";  // Application version - you specify it during device provisioning

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  M5.begin(true, true, true); //screen, batt, serial
  
  Serial.begin(115200);
  client.setServer(mqttServer, 1883);
  client.setCallback(handleOtaUpdate);
  initServerConnection();

  if (client.setBufferSize(1023)) {
    Serial.println("Successfully reallocated internal buffer size");
  } else {
    Serial.println("Failed to reallocated internal buffer size");
  }

  delay(1000);
  reportCurrentFirmwareVersion();
  requestNewFirmware();
}

void loop() {
  // Do work here
  initServerConnection();
  delay(1000);

  // Lcd display setup
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  M5.Lcd.print("FW:" + FW_VERSION);
}

void reportCurrentFirmwareVersion() {
  String reportTopic = "kp1/" + APP_VERSION + "/cmx_ota/" + TOKEN + "/applied/json";
//  String reportPayload = "{\"configId\":\"1.0.0\"}";
  String reportPayload = "{\"configId\":\"" + FW_VERSION + "\"}"; //UPDATE this to match the OTA upgradeable from field on Kaa
  Serial.println("Reporting current firmware version on topic: " + reportTopic + " and payload: " + reportPayload);
  client.publish(reportTopic.c_str(), reportPayload.c_str());
}

void requestNewFirmware() {
  int requestID = random(0, 99);
  String firmwareRequestTopic = "kp1/" + APP_VERSION + "/cmx_ota/" + TOKEN + "/config/json/" + requestID;
  Serial.println("Requesting firmware using topic: " + firmwareRequestTopic);
  client.publish(firmwareRequestTopic.c_str(), "{\"observe\":true}"); // observe is used to specify whether the client wants to accept server pushes
}

void initServerConnection() {
  setupWifi();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void handleOtaUpdate(char* topic, byte* payload, unsigned int length) {
  Serial.printf("\nHandling firmware update message on topic: %s and payload: ", topic);

  DynamicJsonDocument doc(1023);
  deserializeJson(doc, payload, length);
  JsonVariant json_var = doc.as<JsonVariant>();
  Serial.println(json_var.as<String>());
  if (json_var.isNull()) {
    Serial.println("No new firmware version is available");
    return;
  }

  unsigned int statusCode = json_var["statusCode"].as<unsigned int>();
  if (statusCode != 200) {
    Serial.printf("Firmware message's status code is not 200, but: %d\n", statusCode);
    return;
  }

  String firmwareLink = json_var["config"]["link"].as<String>();

  t_httpUpdate_return ret = httpUpdate.update(espClient, firmwareLink.c_str());

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  }
}

void setupWifi() {
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
    char *client_id = "bqf1uai03p4cop6jr3u0";
    if (client.connect(client_id)) {
      Serial.println("Connected to WiFi");
      subscribeToFirmwareUpdates();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void subscribeToFirmwareUpdates() {
  String serverPushOnConnect = "kp1/" + APP_VERSION + "/cmx_ota/" + TOKEN + "/config/json/#";
  client.subscribe(serverPushOnConnect.c_str());
  Serial.println("Subscribed to server firmware push on topic: " + serverPushOnConnect);

  String serverFirmwareResponse = "kp1/" + APP_VERSION + "/cmx_ota/" + TOKEN + "/config/json/status/#";
  client.subscribe(serverFirmwareResponse.c_str());
  Serial.println("Subscribed to server firmware response on topic: " + serverFirmwareResponse);

  String serverFirmwareErrorResponse = "kp1/" + APP_VERSION + "/cmx_ota/" + TOKEN + "/config/json/status/error";
  client.subscribe(serverFirmwareErrorResponse.c_str());
  Serial.println("Subscribed to server firmware response on topic: " + serverFirmwareErrorResponse);
}
