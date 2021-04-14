// This is a prototype program to measure robot performance with the MSRM Task Board. 
// Written by Peter So. December 2020.
// Program will not run on board without being connected to the PbHub unit. 
// Default program will not connect to the internet.
// To connect to wifi you will need to ensure the correct credentials are added to the secrets.h file.
// Press and hold the M5 button while powering up the device to have the device try to connect to the online DB.

//#include <M5StickC.h>
#include <M5StickCPlus.h>
#include <Wire.h>
#include "porthub.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"
#include "hx711.h"

const char* ssid = SECRET_SSID;                   // WiFi name
const char* password = SECRET_PASSWORD;           // WiFi password
const char* mqtt_server = "mqtt.cloud.kaaiot.com";
const String TOKEN = SECRET_TOKEN;                // Endpoint token - you get (or specify) it during device provisioning
const String APP_VERSION = SECRET_APP_VERSION;    // Application version - you specify it during device provisioning

const unsigned long fiveSeconds = 1 * 5 * 1000UL;
static unsigned long lastPublish = 0 - fiveSeconds;

#define LCDHIGH 240
#define LCDWIDTH 320

#define TIMELIMIT 60  // Trial Time Limit in seconds

#define BUTTON_ON 0
#define BUTTON_OFF 1

PortHub porthub;
uint8_t HUB_ADDR[6]={HUB1_ADDR,HUB2_ADDR,HUB3_ADDR,HUB4_ADDR,HUB5_ADDR,HUB6_ADDR};

WiFiClient espClient;
PubSubClient client(espClient);

//timer interrupt variable.
volatile unsigned long usecCount = 0;
hw_timer_t *interrupptTimer = NULL;
portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

//min,sec,msec,usec display.
int display[4] = {0};

//timer start/stop check variable
unsigned long trialTime = 0;
int countStart = 0;
int started = 0;
int keyswitch = 0;
int plug = 0;
int batt1 = 0;
int batt2 = 0;
int timeLeft = 0;
int ptsCollected = 0;
int wifiEnabled = 0;

int TS_key = 0;
int TS_plug = 0;
int TS_batt1 = 0;
int TS_batt2 = 0;

HX711 scale(33, 32);

void IRAM_ATTR usecTimer()
{
  portENTER_CRITICAL_ISR(&mutex);
  usecCount += 5;
  portEXIT_CRITICAL_ISR(&mutex);
}

void setup()
{
  // initialize the M5Stack object
  M5.begin(true, true, true); //screen, batt, serial
  porthub.begin();

   if (!M5.BtnA.isPressed() == 0){
    wifiEnabled = 1;
    // Setup wireless connection
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    M5.Lcd.setRotation(3);
    M5.Lcd.setCursor(2,11,1);
    //Serial.printf("Connecting to [%s]", ssid);
    //M5.Lcd.print("ssid: [%s]\n", ssid);
    M5.Lcd.print("Connecting to wifi...");
   }

  //GPIO setting  
  pinMode(10, OUTPUT);              //GPIO10 the builtin LED

  // Lcd display setup
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  //M5.Lcd.setCursor(0, LCDWIDTH / 4);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);
  //M5.Lcd.printf(" m: s: ms: us\n");
  //M5.Lcd.printf("00:00:000:000\n");

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
}

float weight = 0.0F;
float load = 0.0F;
float cumWeight = 0.0F;
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

void loop()
{
  // put your main code here, to run repeatedly:

  if (wifiEnabled == 1){
  // Connect to wifi logic
  setup_wifi();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // get device accel data
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  M5.IMU.getAhrsData(&pitch,&roll,&yaw);
  M5.IMU.getTempData(&temp);
//  M5.MPU6886.getAccelData(&accX, &accY, &accZ);
//  M5.MPU6886.getGyroData(&gyroX,&gyroY,&gyroZ);
//  M5.MPU6886.getTempData(&temp);

//Weight Scale Logic
  weight = scale.getGram();
  if (M5.BtnA.wasReleased()) {
    scale.setOffset(scale.averageValue());
  }
  
  // Reporting logic
  unsigned long now = millis();
  if (now - lastPublish >= fiveSeconds) 
  {
    lastPublish += fiveSeconds;
    DynamicJsonDocument telemetry(1023);
    telemetry.createNestedObject();
    telemetry[0]["weight"] = weight;
    telemetry[0]["temp"] = temp;
    telemetry[0]["accX"] = accX * 1000;
    telemetry[0]["accY"] = accY * 1000;
    telemetry[0]["accZ"] = accZ * 1000;
    telemetry[0]["keyswitch"] = porthub.hub_d_read_value_A(HUB_ADDR[2]);
    telemetry[0]["plug"] = porthub.hub_d_read_value_A(HUB_ADDR[3]);
    telemetry[0]["Btn0_A"] = porthub.hub_d_read_value_A(HUB_ADDR[0]);
    telemetry[0]["Btn0_B"] = porthub.hub_d_read_value_B(HUB_ADDR[0]);
    telemetry[0]["Btn1_A"] = porthub.hub_d_read_value_A(HUB_ADDR[1]);
    telemetry[0]["Btn1_B"] = porthub.hub_d_read_value_B(HUB_ADDR[1]);
    telemetry[0]["trialStarted"] = started;
    telemetry[0]["trialTime"] = usecCount;
    telemetry[0]["trialTimeRemaining"] = timeLeft;
    telemetry[0]["trialPoints"] = ptsCollected;
    

    String topic = "kp1/" + APP_VERSION + "/dcx/" + TOKEN + "/json";
    client.publish(topic.c_str(), telemetry.as<String>().c_str());
    Serial.println("Published on topic: " + topic);
  }
  }
  
  //Start Button Check
  if (porthub.hub_d_read_value_A(HUB_ADDR[0]) != BUTTON_OFF && started == 0 && porthub.hub_d_read_value_A(HUB_ADDR[2]) == 1 && porthub.hub_d_read_value_A(HUB_ADDR[3]) == 1)
  {
    delay(1);
    if (porthub.hub_d_read_value_A(HUB_ADDR[0]) != BUTTON_OFF)
      countStart = 1;
      digitalWrite(10, HIGH); //turn off LED
      Serial.println("Button Status: BtnA pressed");

    for (;;)
      if (porthub.hub_d_read_value_A(HUB_ADDR[0]) == BUTTON_OFF)
        break;
    delay(1);
  }

  //Stop Button Check
  if (porthub.hub_d_read_value_A(HUB_ADDR[0]) != BUTTON_OFF && started == 1 && keyswitch == 1 && batt1 == 1 && batt2 == 1)
//  if (porthub.hub_d_read_value_A(HUB_ADDR[0]) != BUTTON_OFF && started == 1 && plug == 1 && batt1 == 1 && batt2 == 1)
  {
    delay(1);
    if (porthub.hub_d_read_value_A(HUB_ADDR[0]) != BUTTON_OFF)
      countStart = 0;
      Serial.println("Button Status: BtnA pressed STOP!");

    for (;;)
      if (porthub.hub_d_read_value_A(HUB_ADDR[0]) == BUTTON_OFF)
        break;
    delay(1);
  }

  //Time Limit Check
  if (started == 1 && timeLeft <= 0)
  {
    delay(1);
      countStart = 0;
      Serial.print("Time's Up! Trial Time Limit: ");
      Serial.println(TIMELIMIT);
    delay(1);
  }

  //Keyswith Check
  if (porthub.hub_d_read_value_A(HUB_ADDR[3]) != BUTTON_OFF && started == 1 && keyswitch == 0)
  {
    delay(1);
    keyswitch = 1;
    TS_key = usecCount;
    digitalWrite(10, HIGH); //turn off LED when red button is pressed
    delay(50);
    digitalWrite(10, LOW); //turn on LED when red button is pressed
    Serial.println("Button Status: Key switched!");
  }


  //Plug Check
  if (porthub.hub_d_read_value_A(HUB_ADDR[2]) != BUTTON_OFF && started == 1 && plug == 0)
  {
    delay(1);
    plug = 1;
    TS_plug = usecCount;
    digitalWrite(10, HIGH); //turn off LED when red button is pressed
    delay(50);
    digitalWrite(10, LOW); //turn on LED when red button is pressed
    Serial.println("Button Status: plug seated!");
  }

  //Battery Hole 1 Check
  if (porthub.hub_d_read_value_A(HUB_ADDR[1]) != BUTTON_OFF && started == 1 && batt1 == 0)
  {
    delay(1);
    batt1 = 1;
    TS_batt1 = usecCount;
    digitalWrite(10, HIGH); //turn off LED when red button is pressed
    delay(50);
    digitalWrite(10, LOW); //turn on LED when red button is pressed
    Serial.println("Button Status: batt1 inserted!");
  }

  //Battery Hole 2 Check
  if (porthub.hub_d_read_value_B(HUB_ADDR[1]) != BUTTON_OFF && started == 1 && batt2 == 0)
  {
    delay(1);
    batt2 = 1;
    TS_batt2 = usecCount;
    digitalWrite(10, HIGH); //turn off LED when red button is pressed
    delay(50);
    digitalWrite(10, LOW); //turn on LED when red button is pressed
    Serial.println("Button Status: batt2 inserted!");
  }

  //Time Count  Start
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
    keyswitch = 0;
    plug = 0;
    batt1 = 0;
    batt2 = 0;
    trialTime = usecCount;
  }

  //Count Reset Check
  if (porthub.hub_d_read_value_B(HUB_ADDR[0]) != BUTTON_OFF && started == 0)
  {
    delay(1);
    if (porthub.hub_d_read_value_B(HUB_ADDR[0]) != BUTTON_OFF)
      Serial.println("Button Status: BtnB pressed");
      usecCount = 0;
      TS_key = 0;
      TS_plug = 0;
      TS_batt1 = 0;
      TS_batt2 = 0;
      trialTime = 0;
      cumWeight = 0;
      digitalWrite(10, HIGH); //turn off LED

    for (;;)
      if (porthub.hub_d_read_value_B(HUB_ADDR[0]) == BUTTON_OFF)
        break;
    delay(1);
  }

  //time calculation
  display[3] = (int)(usecCount % 1000);
  display[2] = (int)((usecCount % 1000000) / 1000);
  display[1] = (int)((usecCount / 1000000) % 60);
  display[0] = (int)((usecCount / 60000000) % 3600);

  //collect weight during trial
  if (started == 1)
  {
    load = porthub.hub_a_read_value(HUB_ADDR[4]);
    cumWeight = load + cumWeight;
  }
 
  //count display
  //portENTER_CRITICAL(&mutex);
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  //M5.Lcd.setCursor(0, LCDWIDTH / 4);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.printf("Smart Task Board\n");
  M5.Lcd.printf("Wifi:%d:%d, %d\n", wifiEnabled, WiFi.status(), timeLeft);
  M5.Lcd.printf(" m: s: ms: us\n");
  M5.Lcd.printf("%02d:",display[0]);
  M5.Lcd.printf("%02d:",display[1]);
  M5.Lcd.printf("%03d:",display[2]);
  M5.Lcd.printf("%03d\n",display[3]);
  M5.Lcd.printf("%d KEY_L:%d KEY_R:%d TS:%d\n", keyswitch, porthub.hub_d_read_value_A(HUB_ADDR[3]), porthub.hub_d_read_value_B(HUB_ADDR[3]), TS_key); 
  M5.Lcd.printf("%d USB_L:%d USB_R:%d TS:%d\n", plug, porthub.hub_d_read_value_A(HUB_ADDR[2]), porthub.hub_d_read_value_B(HUB_ADDR[2]), TS_plug); 
  M5.Lcd.printf("%d BAT_1:%d         TS:%d\n", batt1, porthub.hub_d_read_value_A(HUB_ADDR[1]), TS_batt1); 
  M5.Lcd.printf("%d BAT_2:%d         TS:%d\n", batt2, porthub.hub_d_read_value_B(HUB_ADDR[1]), TS_batt2); 
  M5.Lcd.printf("Trial Time: %d\n", trialTime);
//  M5.Lcd.printf("0:%d/%d 1:%d/%d\n", porthub.hub_d_read_value_A(HUB_ADDR[0]), porthub.hub_d_read_value_B(HUB_ADDR[0]), porthub.hub_d_read_value_A(HUB_ADDR[1]), porthub.hub_d_read_value_B(HUB_ADDR[1]));
//  M5.Lcd.printf("2:%d/%d 3:%d/%d\n", porthub.hub_d_read_value_A(HUB_ADDR[2]), porthub.hub_d_read_value_B(HUB_ADDR[2]), porthub.hub_d_read_value_A(HUB_ADDR[3]), porthub.hub_d_read_value_B(HUB_ADDR[3]));
  M5.Lcd.printf("%d weight: %d, cum: %d\n", started, porthub.hub_a_read_value(HUB_ADDR[4]), cumWeight);
//  M5.Lcd.printf("accX:%d accY:%d accZ:%d\n", accX*1000, accY*1000, accZ*1000);
//M5.Lcd.printf("gyX:%.2f gyY:%.2f gyZ:%.2f\n", gyroX, gyroY, gyroZ);
  //M5.Lcd.printf("%lu", usecCount);
  //M5.Lcd.printf("%d", timeLeft);
  //Serial.println(usecCount); //print out seconds to the serial monitor
  Serial.printf("Key: %d, Plug: %d, Batt1: %d, Batt2: %d, Time: %d\n", TS_key, TS_plug, TS_batt1, TS_batt2, usecCount); //print out seconds to the serial monitor

//  delay(10); // delay for screen refresh NOTE: This directly affects performance of clock buttons
  //portEXIT_CRITICAL(&mutex);
  
  timeLeft = TIMELIMIT - display[1];
}


// Custom Function Definitions
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
