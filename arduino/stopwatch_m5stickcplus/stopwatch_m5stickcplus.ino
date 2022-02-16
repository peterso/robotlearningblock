// This is a program to measure manipulation performance with the MSRM Task Board. 
// Written by Peter So. December 2020.
// Last updated July 2021
//
// Program will not run on board without being connected to the PbHub unit. 
// Default behavior is board will attempt to WiFi network. Hold M5 button during power up to use without WiFi.
//
// To connect to wifi you will need to ensure the correct credentials are added to the secrets.h file.

//#include <M5StickC.h> //uncomment if using only a M5Stick version device and comment the next line
#include <M5StickCPlus.h> // https://github.com/m5stack/M5StickC-Plus
#include <Wire.h>
#include "porthub.h"
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include <HTTPClient.h>
#include <HTTPUpdate.h>
//#include "kaa.h"

// USER CONFIGURABLE SETTINGS
#define PROTOCOL_ID "MSRM_100"
#define TIMELIMIT 600  // Trial Time Limit in seconds (600 is 10min)
#define FW_VERSION "0.0.1" // Firmware Version for OTA management

//////// SYSTEM SETTINGS /////////
// DO NOT CHANGE SETTINGS BELOW //
const char* ssid = SECRET_SSID;                   // WiFi name
const char* password = SECRET_PASSWORD;           // WiFi password
const char* mqtt_server = "mqtt.cloud.kaaiot.com";
const String TOKEN = SECRET_TOKEN;                // Endpoint token - you get (or specify) it during device provisioning
const String APP_VERSION = SECRET_APP_VERSION;    // Application version - you specify it during device provisioning

const unsigned long fiveSeconds = 1 * 5 * 1000UL;
static unsigned long lastPublish = 0 - fiveSeconds;

#define PTS_BUTTON 1
#define PTS_KEY 1
#define PTS_PLUG 1
#define PTS_BATT1 1
#define PTS_BATT2 1
#define BUTTON_ON 0
#define BUTTON_OFF 1

// Setup PbHub device
PortHub porthub;
uint8_t HUB_ADDR[6]={HUB1_ADDR,HUB2_ADDR,HUB3_ADDR,HUB4_ADDR,HUB5_ADDR,HUB6_ADDR};

// Setup WiFi and PubSub client
WiFiClient espClient;
PubSubClient client(espClient);
//Kaa kaa(&client, SECRET_TOKEN, SECRET_APP_VERSION);

// Setup stopwatch interrupt specific settings
//timer interrupt variable.
volatile unsigned long usecCount = 0;
hw_timer_t *interrupptTimer = NULL;
portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR usecTimer()
{
  portENTER_CRITICAL_ISR(&mutex);
  usecCount += 5;
  portEXIT_CRITICAL_ISR(&mutex);
}

///////////////////////////////
/////// INITIALIZE VARS ///////
///////////////////////////////

//min,sec,msec,usec display.
int display[4] = {0};

//timer start/stop check variable
unsigned long trialTime = 0;
int wifiEnabled = 0;
int countStart = 0;
int started = 0;
int buttonPushLatch = 0;
int keyswitchLatch = 0;
int plugLatch = 0;
int batt1Latch = 0;
int batt2Latch = 0;
int timeLeft = 0;
int ptsCollected = 0;

int startBtnState = -1;
int stopBtnState = -1;
int resetBtnState = -1;
int buttonPushState = -1;
int keyswitchState = -1;
int plugState = -1;
int batt1BtnState = -1;
int batt2BtnState = -1;
int buttonPushState_old = -1;
int stopBtnState_old = -1;
int keyswitchState_old = -1;
int plugState_old = -1;
int batt1BtnState_old = -1;
int batt2BtnState_old = -1;

int TS_button = 0;
int TS_key = 0;
int TS_plug = 0;
int TS_batt1 = 0;
int TS_batt2 = 0;

// Initialize program variables before running the main loop
float force = 0.0;
float cumForce = 0.0;
float startaccX = 0.0;
float startaccY = 0.0;
float startaccZ = 0.0;
float load = 0.0F;
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;
float temp   = 0.0F;

uint8_t BoardState[6]={startBtnState,resetBtnState,keyswitchState,plugState,batt1BtnState,batt2BtnState};

void setup()
{
  // initialize the M5Stack object
  M5.begin(true, true, true); //screen, batt, serial
  porthub.begin();

  // Lcd display setup
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);

  // Setup WiFi connection or boot in LOCAL MODE
  if (!M5.BtnA.isPressed() == 1){ // Press and hold M5 Button during power up to enter LOCAL Mode
    //    M5.Lcd.print("ssid: %s\n", *ssid);
    M5.Lcd.print("Attempting to connect to wifi...\n");
    M5.Lcd.print("If unable to connect, power off and \n");
    M5.Lcd.print("then on while holding M5 button\n");
    M5.Lcd.print("to bypass and run locally only\n\n");
    M5.Lcd.print("Try to configure new wifi credentials by\n");
    M5.Lcd.print("connecting to AutoConnectAP-task-board\n");
    M5.Lcd.print("then browse to 192.168.4.1 with your\n");
    M5.Lcd.print("PC or phone to select new ssid\n");
    M5.Lcd.print("and give new password\n");
    wifiEnabled = 1;
    
    //Wifi Manager Config START
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  
    //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wm;
  
    //reset settings - wipe credentials for testing
    //wm.resetSettings();
  
    bool res;
    // res = wm.autoConnect(); // auto generated AP name from chipid
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    res = wm.autoConnect("AutoConnectAP-task-board","password"); // password protected ap
  
    if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
    }
    //Wifi Manager Config END
    
    // Setup wireless connection
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    // Increase limit, this line fixed problem for my device
    client.setBufferSize(8192);
    setup_wifi();

  } else {
      M5.Lcd.print("Booting Local Mode...");
      wifiEnabled = 0;
  }

  // GPIO setting  
  pinMode(10, OUTPUT);              //GPIO10 the builtin LED

  //interrupt timer setting
  //timerBegin is count per 100 microsec.
  interrupptTimer = timerBegin(0, 80, true);
  //interrupt method setting
  timerAttachInterrupt(interrupptTimer, &usecTimer, true);
  //interrupt timing setting.
  timerAlarmWrite(interrupptTimer, 5, true);
  timerAlarmDisable(interrupptTimer);
  //timerAlarmEnable(interupptTimer);
  digitalWrite(10, LOW); //turn on LED when red button is pressed
  delay(2000); //tmp delay just to verify setup...
  digitalWrite(10, HIGH); //turn off LED when red button is pressed
  
  Serial.begin(115200);
  M5.Imu.Init();

  // Setup load cell device
  //REMOVED

  //Check for new firmware
  delay(1000);
  reportCurrentFirmwareVersion();
  requestNewFirmware();
}

void loop()
{
  // put your main code here, to run repeatedly:

  /////// READ INPUTS //////
  // get device accel data
  M5.Imu.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.Imu.getAccelData(&accX,&accY,&accZ);
  M5.Imu.getAhrsData(&pitch,&roll,&yaw);
  M5.Imu.getTempData(&temp);
  startBtnState = !M5.BtnA.read();
  resetBtnState = !M5.BtnB.read();

  // Read from PbHub Module
  buttonPushState = porthub.hub_d_read_value_A(HUB_ADDR[0]);
  stopBtnState = porthub.hub_d_read_value_B(HUB_ADDR[0]);
  keyswitchState = porthub.hub_d_read_value_A(HUB_ADDR[2]);
  plugState = porthub.hub_d_read_value_A(HUB_ADDR[3]);
  batt1BtnState = porthub.hub_d_read_value_A(HUB_ADDR[1]);
  batt2BtnState = porthub.hub_d_read_value_B(HUB_ADDR[1]);

  if (wifiEnabled == 1)
  {
    // Connect to wifi logic
    if (!client.connected()) {
      Serial.println("Attempting to connect to WiFi...");
      reconnect();
    }
    //client.loop(); //SUSPICIOUS if this is really needed... causes irregular loop execution speeds

    // Reporting logic to remote server
    unsigned long now = millis();
    if (now - lastPublish >= fiveSeconds) // publish to topic every 5 seconds
    {
      lastPublish += fiveSeconds;
      DynamicJsonDocument telemetry(8192); // increased from 1023
      telemetry.createNestedObject();

      telemetry[0]["accX"] = accX * 1000; //Float
      telemetry[0]["accY"] = accY * 1000; //Float
      telemetry[0]["accZ"] = accZ * 1000; //Float
      telemetry[0]["gyroX"] = gyroX; //Float
      telemetry[0]["gyroY"] = gyroY; //Float
      telemetry[0]["gyroZ"] = gyroZ; //Float
      telemetry[0]["keyswitchState"] = keyswitchState; //BOOL
      telemetry[0]["plugState"] = plugState; //BOOL
      telemetry[0]["startButtonState"] = startBtnState; //BOOL
      telemetry[0]["resetButtonState"] = resetBtnState; //BOOL
      telemetry[0]["pushButtonState"] = buttonPushState; // BOOL
      telemetry[0]["stopButtonState"] = stopBtnState; // BOOL
      telemetry[0]["Batt1BtnState"] = batt1BtnState; //BOOL
      telemetry[0]["Batt2BtnState"] = batt2BtnState; //BOOL
      telemetry[0]["trialStarted"] = started; //BOOL
      telemetry[0]["trialTime"] = usecCount; //Float
      telemetry[0]["Button_TS"] = TS_button; //INT
      telemetry[0]["Key_TS"] = TS_key; //INT
      telemetry[0]["Plug_TS"] = TS_plug; //INT
      telemetry[0]["Batt1_TS"] = TS_batt1; //INT
      telemetry[0]["Batt2_TS"] = TS_batt2; //INT
      telemetry[0]["cumForce"] = cumForce;//Float
      telemetry[0]["trialPoints"] = ptsCollected; //INT 
      
      String topic = "kp1/" + APP_VERSION + "/dcx/" + TOKEN + "/json";
      client.publish(topic.c_str(), telemetry.as<String>().c_str());
      Serial.println("Published on topic: " + topic);
    }
  }

  //time calculation
  display[3] = (int)(usecCount % 1000);
  display[2] = (int)((usecCount % 1000000) / 1000);
  display[1] = (int)((usecCount / 1000000) % 60);
  display[0] = (int)((usecCount / 60000000) % 3600);
  
  //Start Button Check
  if (startBtnState != BUTTON_OFF && started == 0 && plugState == 1 && keyswitchState == 1)
  {
    delay(1);
    if (startBtnState != BUTTON_OFF)
      countStart = 1;
      startaccX = accX;
      startaccY = accY;
      startaccZ = accZ;
      digitalWrite(10, LOW); //turn on LED
      Serial.println("Trial Status: M5.BtnA pressed, Trial Stated!");
    delay(1);
  }

  //Stop Button Check
  if (stopBtnState != BUTTON_OFF && started == 1 && buttonPushLatch == 1 && keyswitchLatch == 1 && plugLatch == 1 && batt1Latch == 1 && batt2Latch == 1)
  {
    delay(1);
    if (stopBtnState != BUTTON_OFF)
      countStart = 0;
      digitalWrite(10, HIGH); //turn off LED
      Serial.println("Trial Status: Red Button pressed, Trial Stopped!");
    delay(1);
  }

  //Time Limit Check
  timeLeft = round(TIMELIMIT - usecCount/1000000);
  if (started == 1 && timeLeft <= 0)
  {
    delay(1);
      countStart = 0;
      Serial.print("Trial Status: Time's Up! Trial Time Limit: ");
      Serial.println(TIMELIMIT);
      digitalWrite(10, HIGH); //turn off LED
    delay(1);
  }

  //Button Check
  if (buttonPushState != BUTTON_OFF && started == 1 && buttonPushLatch == 0)
  {
    delay(1);
    buttonPushLatch = 1;
    TS_button = usecCount;
    ptsCollected = ptsCollected + PTS_BUTTON;
    digitalWrite(10, HIGH); //turn off LED when red button is pressed
    delay(50);
    digitalWrite(10, LOW); //turn on LED when red button is pressed
    Serial.println("Trial Status: Button pushed!");
  }

  //Keyswith Check
  if (keyswitchState != BUTTON_OFF && started == 1 && keyswitchLatch == 0)
  {
    delay(1);
    keyswitchLatch = 1;
    TS_key = usecCount;
    ptsCollected = ptsCollected + PTS_KEY;
    digitalWrite(10, HIGH); //turn off LED when red button is pressed
    delay(50);
    digitalWrite(10, LOW); //turn on LED when red button is pressed
    Serial.println("Trial Status: Key switched!");
  }

  //Plug Check
  if (plugState != BUTTON_OFF && started == 1 && plugLatch == 0)
  {
    delay(1);
    plugLatch = 1;
    TS_plug = usecCount;
    ptsCollected = ptsCollected + PTS_PLUG;
    digitalWrite(10, HIGH); //turn off LED when red button is pressed
    delay(50);
    digitalWrite(10, LOW); //turn on LED when red button is pressed
    Serial.println("Trial Status: plug seated!");
  }

  //Battery Hole 1 Check
  if (batt1BtnState != BUTTON_OFF && started == 1 && batt1Latch == 0)
  {
    delay(1);
    batt1Latch = 1;
    TS_batt1 = usecCount;
    ptsCollected = ptsCollected + PTS_BATT1;
    digitalWrite(10, HIGH); //turn off LED when red button is pressed
    delay(50);
    digitalWrite(10, LOW); //turn on LED when red button is pressed
    Serial.println("Trial Status: batt1 inserted!");
  }

  //Battery Hole 2 Check
  if (batt2BtnState != BUTTON_OFF && started == 1 && batt2Latch == 0)
  {
    delay(1);
    batt2Latch = 1;
    TS_batt2 = usecCount;
    ptsCollected = ptsCollected + PTS_BATT2;
    digitalWrite(10, HIGH); //turn off LED when red button is pressed
    delay(50);
    digitalWrite(10, LOW); //turn on LED when red button is pressed
    Serial.println("Trial Status: batt2 inserted!");
  }

  //Time Count Start
  if (countStart == 1 && started == 0)
  {
    timerAlarmEnable(interrupptTimer);
    started = 1;
  }

  //Time Count  Stop
  if (countStart == 0 && started == 1)
  {
    timerAlarmDisable(interrupptTimer);
    started = 0;
    trialTime = usecCount;
  }

  //Count Reset Check
  if (resetBtnState != BUTTON_OFF && started == 0)
  {
    delay(1);
    if (resetBtnState != BUTTON_OFF)
      Serial.println("Trial Status: Trial Reset pressed");
      usecCount = 0;
      buttonPushLatch = 0;
      keyswitchLatch = 0;
      plugLatch = 0;
      batt1Latch = 0;
      batt2Latch = 0;
      TS_button = 0;
      TS_key = 0;
      TS_plug = 0;
      TS_batt1 = 0;
      TS_batt2 = 0;
      trialTime = 0;
      ptsCollected = 0;
      cumForce = 0;
      digitalWrite(10, HIGH); //turn off LED
    delay(1);
  }

  // collect "force" during trial
  if (started == 1)
  {
    force = abs(accX - startaccX) + abs(accY - startaccY) + abs(accZ - startaccZ);  
    cumForce = cumForce + force;
  }

  // Report task board changes to Serial
  if (buttonPushState == 0 && buttonPushState != buttonPushState_old){Serial.println("Blue Push Button Pressed");};
  if (buttonPushState == 1 && buttonPushState != buttonPushState_old){Serial.println("Blue Push Button Released");};
  buttonPushState_old = buttonPushState; // store current value
  if (stopBtnState == 0 && stopBtnState != stopBtnState_old){Serial.println("Red Push Button Pressed");};
  if (stopBtnState == 1 && stopBtnState != stopBtnState_old){Serial.println("Red Push Button Released");};
  stopBtnState_old = stopBtnState; // store current value
  if (keyswitchState == 0 && keyswitchState != keyswitchState_old){Serial.println("Key Switch Closed");};
  if (keyswitchState == 1 && keyswitchState != keyswitchState_old){Serial.println("Key Switch Opened");};
  keyswitchState_old = keyswitchState; // store current value
  if (plugState == 0 && plugState != plugState_old){Serial.println("Plug Switch Closed");};
  if (plugState == 1 && plugState != plugState_old){Serial.println("Plug Switch Opened");};
  plugState_old = plugState; // store current value
  if (batt1BtnState == 0 && batt1BtnState != batt1BtnState_old){Serial.println("Batt1 Button Pressed");};
  if (batt1BtnState == 1 && batt1BtnState != batt1BtnState_old){Serial.println("Batt1 Button Released");};
  batt1BtnState_old = batt1BtnState; // store current value
  if (batt2BtnState == 0 && batt2BtnState != batt2BtnState_old){Serial.println("Batt2 Button Pressed");};
  if (batt2BtnState == 1 && batt2BtnState != batt2BtnState_old){Serial.println("Batt2 Button Released");};
  batt2BtnState_old = batt2BtnState; // store current value

  // update display
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 5);
  M5.Lcd.printf("Smart Task Board");
  M5.Lcd.printf("v %s\n", FW_VERSION);
  M5.Lcd.printf("Wifi On:%d Status:%d\n", wifiEnabled, WiFi.status());
  M5.Lcd.printf("PROTOCOL: %s\n", PROTOCOL_ID);
  M5.Lcd.printf("%d BTN_1:%d TS:%d\n", buttonPushLatch, buttonPushState, TS_button); 
  M5.Lcd.printf("%d KEY_L:%d TS:%d\n", keyswitchLatch, keyswitchState, TS_key); 
  M5.Lcd.printf("%d ETH_L:%d TS:%d\n", plugLatch, plugState, TS_plug); 
  M5.Lcd.printf("%d BAT_1:%d TS:%d\n", batt1Latch, batt1BtnState, TS_batt1); 
  M5.Lcd.printf("%d BAT_2:%d TS:%d\n", batt2Latch, batt2BtnState, TS_batt2); 
  M5.Lcd.printf("Started:%d Time Left: %d Pts:%d\n", started, timeLeft, ptsCollected);
  M5.Lcd.printf("Trial Time:");
  M5.Lcd.printf(" m: s: ms: us\n");
  M5.Lcd.printf("%02d:",display[0]);
  M5.Lcd.printf("%02d:",display[1]);
  M5.Lcd.printf("%03d:",display[2]);
  M5.Lcd.printf("%03d\n",display[3]);
  M5.Lcd.printf("Total Force: %0.2f\n", cumForce);
  M5.Lcd.printf("acX:%0.2f acY:%0.2f acZ:%0.2f\n", accX*1000, accY*1000, accZ*1000);
  M5.Lcd.printf("gyX:%0.2f gyY:%0.2f gyZ:%0.2f\n", gyroX, gyroY, gyroZ);
  M5.Lcd.printf("Token: %s", SECRET_TOKEN);
  
  Serial.printf("Token:%s, CurrentState:BTN_1:%d,KEY_L:%d,ETH_L:%d,BAT1:%d,BAT2:%d, Protocol:%s, TrialRunning:%d, TimeLeft_sec:%d, TrialPts:%d, TotalTrialForce:%0.2f, Key_TS_us:%d, Plug_TS_us:%d, Batt1_TS_us:%d, Batt2_TS_us:%d, Time_us:%d\n", SECRET_TOKEN, buttonPushState, keyswitchState, plugState, batt1BtnState, batt2BtnState, PROTOCOL_ID, started, timeLeft, ptsCollected, cumForce, TS_key, TS_plug, TS_batt1, TS_batt2, usecCount); //print out seconds to the serial monitor
  //delay(10); // delay for screen refresh NOTE: This directly affects performance of clock buttons
}

/////////////////////////////////
// Custom Function Definitions //
/////////////////////////////////
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("\nHandling command message on topic: %s\n", topic);
  
  //Check if message is related to the OTA update
  if (String(topic).startsWith("kp1/" + APP_VERSION + "/cmx_ota/" + TOKEN)) {
    handleOtaUpdate(topic, payload, length); //seems to be running with every message related to cmx_ota... remove this and only check for new firmawre on powerup.
    return;
  }
  
  //Check if message is related to a command message
  if (!String(topic).startsWith("kp1/" + APP_VERSION + "/cex/" + TOKEN)) {
    return;
  }

  DynamicJsonDocument doc(2048);
  deserializeJson(doc, payload, length);
  JsonVariant json_var = doc.as<JsonVariant>();
  
  DynamicJsonDocument commandResponse(2048);
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

  // Receive and parse the payload
  Serial.println("Payload: " + doc.as<String>()); //TODO troubleshoot
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

  //Inserting from OTA Example function subscribeToFirmwareUpdates()
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

//OTA Functions from Kaa
void reportCurrentFirmwareVersion() {
  String reportTopic = "kp1/" + APP_VERSION + "/cmx_ota/" + TOKEN + "/applied/json";
  String reportPayload = "{\"configId\":\"0.0.1\"}"; //UPDATE this to match the OTA upgradeable from field on Kaa
  Serial.println("Reporting current firmware version on topic: " + reportTopic + " and payload: " + reportPayload);
  client.publish(reportTopic.c_str(), reportPayload.c_str());
}

void requestNewFirmware() {
  int requestID = random(0, 99);
  String firmwareRequestTopic = "kp1/" + APP_VERSION + "/cmx_ota/" + TOKEN + "/config/json/" + requestID;
  Serial.println("Requesting firmware using topic: " + firmwareRequestTopic);
  client.publish(firmwareRequestTopic.c_str(), "{\"observe\":true}"); // observe is used to specify whether the client wants to accept server pushes
}

void handleOtaUpdate(char* topic, byte* payload, unsigned int length) {
  Serial.printf("\nHandling firmware update message on topic: %s and payload: ", topic);

  //DEBUG with Denys
  //This fixed the issue!
  //This checks that the topic is indeed a cmx_ota message. 
  if (!String(topic).startsWith("kp1/" + APP_VERSION + "/cmx_ota/" + TOKEN)) {
  return;
  }
  //END DEBUG
  
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

  //  return; //DEBUG escape do not actually update the firmware //commenting this line will enable OTA update.
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
