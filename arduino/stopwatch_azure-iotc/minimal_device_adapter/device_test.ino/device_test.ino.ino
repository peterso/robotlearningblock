/**
 * Azure IoT Central example for esp32-azure-kit adapted by pso.
 */

//#include <M5StickCPlus.h>
#include <M5StickC.h>
#include "src/porthub.h"
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include "src/secrets.h"

#include <WiFi.h>
#include "AzureIotHub.h"
#include "src/parson.h"
#include "src/sensor_manager.h"
#include "src/led.h"
//#include "src/hx711.h" //include load cell (loadcell) library


#define TELEMETRY_INTERVAL 1000

/*String containing Hostname, Device Id & Device Key in the format:                         */
// handcrafted connection string
// NOTE: To commission a new device, update the DeviceID and SharedAccessKey in the connection string here.
// https://msrm-connected-taskboards.azureiotcentral.com/devices/details/kresimir-hand-holding/rawdata 
//static const char *connectionString = "HostName=iotc-111c2522-561d-4f03-9267-9c169ad02fe9.azure-devices.net;DeviceId=kresimir-hand-holding;SharedAccessKey=2zqujYgAegVVflrSQuuF7Nn75Cs9CQxU0i12SIx2vG0=";
//static const char *connectionString = "HostName=iotc-111c2522-561d-4f03-9267-9c169ad02fe9.azure-devices.net;DeviceId=zk8s8ih1k1;SharedAccessKey=qJ6BYHNE7ECnqm5KA6+UeUO7WTYfD43EganzpsjWNWo=";
static const char *connectionString = "HostName=iotc-111c2522-561d-4f03-9267-9c169ad02fe9.azure-devices.net;DeviceId=kresimir2;SharedAccessKey=iohrllySeNNMFie1NQAwEuCTck2f3+E1zf4k4Txc6hU=";

/*  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessKey=<device_key>"                */
/*  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessSignature=<device_sas_token>"    */
//static const char *connectionString = "";


// USER CONFIGURABLE SETTINGS
#define PROTOCOL_ID "MSRM_100"
#define TIMELIMIT 600  // Trial Time Limit in seconds (600 is 10min)


//////// SYSTEM SETTINGS /////////
// DO NOT CHANGE SETTINGS BELOW //
const char* ssid = SECRET_SSID;                   // WiFi name
const char* password = SECRET_PASSWORD;           // WiFi password

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
uint8_t HUB_ADDR[6]={HUB1_ADDR,HUB2_ADDR,HUB3_ADDR,HUB4_ADDR,HUB5_ADDR,HUB6_ADDR}; //porthub pin setup

// Setup the Load Cell module
// Pins 33 and 32, i.e. "HX711 scale(33, 32)" work when the weight module is connected directly to the M5Stick device
//HX711 scale(33, 32);    // initialize scale (loadcell) NOTE may need to update the pins bc chaining on the PbHub

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
//float loadcell = 0.0F;
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




typedef struct EVENT_MESSAGE_INSTANCE_TAG
{
  IOTHUB_MESSAGE_HANDLE messageHandle;
  size_t messageTrackingId; // For tracking the messages within the user callback.
} EVENT_MESSAGE_INSTANCE_TAG;

IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle = NULL;
static char propText[1024];
static char msgText[1024];
static int trackingId = 0;

static int fanSpeed = 10;
static int temThreshold = 30;
static bool fan_running = false;
static bool fan_running_with_command = false;
static bool needs_reconnect = false;

static bool hasIoTHub = false;
static bool hasWifi = false;
static uint64_t send_interval_ms;
static uint64_t check_interval_ms;

static bool showHumitureScreen = true;

// forward declarations
static void deviceTwinCallback(DEVICE_TWIN_UPDATE_STATE update_state, const unsigned char *payLoad, size_t size, void *userContextCallback);
static int deviceMethodCallback(const char *method_name, const unsigned char *payload, size_t size, unsigned char **response, size_t *response_size, void *userContextCallback);
static void connectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void *user_context);
static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *userContextCallback);
static void reportedStateCallback(int status_code, void *userContextCallback);

static bool initIotHubClient(void)
{
  Serial.println("initIotHubClient Start!");
  if (platform_init() != 0)
  {
    Serial.println("Failed to initialize the platform.");
    return false;
  }

  if ((iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(connectionString, MQTT_Protocol)) == NULL)
  {
    Serial.println("ERROR: iotHubClientHandle is NULL!");
    return false;
  }

  IoTHubClient_LL_SetRetryPolicy(iotHubClientHandle, IOTHUB_CLIENT_RETRY_EXPONENTIAL_BACKOFF, 1200);
  bool traceOn = true;
  IoTHubClient_LL_SetOption(iotHubClientHandle, "logtrace", &traceOn);

  // Setting twin call back for desired properties receiving.
  if (IoTHubClient_LL_SetDeviceTwinCallback(iotHubClientHandle, deviceTwinCallback, NULL) != IOTHUB_CLIENT_OK)
  {
    Serial.println("IoTHubClient_LL_SetDeviceTwinCallback..........FAILED!");
    return false;
  }

  // Setting direct method callback for direct method calls receiving
  if (IoTHubClient_LL_SetDeviceMethodCallback(iotHubClientHandle, deviceMethodCallback, NULL) != IOTHUB_CLIENT_OK)
  {
    Serial.println("IoTHubClient_LL_SetDeviceMethodCallback..........FAILED!");
    return false;
  }

  // Connection status change callback
  if (IoTHubClient_LL_SetConnectionStatusCallback(iotHubClientHandle, connectionStatusCallback, NULL) != IOTHUB_CLIENT_OK)
  {
    Serial.println("IoTHubClient_LL_SetDeviceMethodCallback..........FAILED!");
    return false;
  }
  Serial.println("initIotHubClient End!");

  // toggle azure led to default off
  toggle_azure_led(0);
  return true;
}

static void closeIotHubClient()
{
  if (iotHubClientHandle != NULL)
  {
    IoTHubClient_LL_Destroy(iotHubClientHandle);
    platform_deinit();
    iotHubClientHandle = NULL;
  }
  Serial.println("closeIotHubClient!");
}

static void sendTelemetry(const char *payload)
{
  if (needs_reconnect)
  {
    closeIotHubClient();
    initIotHubClient();
    needs_reconnect = false;
  }

  EVENT_MESSAGE_INSTANCE_TAG *thisMessage = (EVENT_MESSAGE_INSTANCE_TAG *)malloc(sizeof(EVENT_MESSAGE_INSTANCE_TAG));
  thisMessage->messageHandle = IoTHubMessage_CreateFromByteArray((const unsigned char *)payload, strlen(payload));

  if (thisMessage->messageHandle == NULL)
  {
    Serial.println("ERROR: iotHubMessageHandle is NULL!");
    free(thisMessage);
    return;
  }

  thisMessage->messageTrackingId = trackingId++;

  MAP_HANDLE propMap = IoTHubMessage_Properties(thisMessage->messageHandle);

  (void)sprintf_s(propText, sizeof(propText), "PropMsg_%zu", trackingId);
  if (Map_AddOrUpdate(propMap, "PropName", propText) != MAP_OK)
  {
    Serial.println("ERROR: Map_AddOrUpdate Failed!");
  }

  // send message to the Azure Iot hub
  if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle,
                                     thisMessage->messageHandle, sendConfirmationCallback, thisMessage) != IOTHUB_CLIENT_OK)
  {
    Serial.println("ERROR: IoTHubClient_LL_SendEventAsync..........FAILED!");
    return;
  }

  /* Turn on Azure LED */
  toggle_azure_led(1);

  IoTHubClient_LL_DoWork(iotHubClientHandle);
  Serial.println("IoTHubClient sendTelemetry completed!");
}

static void sendReportedProperty(const char *payload)
{
  if (needs_reconnect)
  {
    closeIotHubClient();
    initIotHubClient();
    needs_reconnect = false;
  }
  if (IoTHubClient_LL_SendReportedState(iotHubClientHandle, (const unsigned char *)payload,
                                        strlen((const char *)payload), reportedStateCallback, NULL) != IOTHUB_CLIENT_OK)
  {
    Serial.println("ERROR: IoTHubClient sendReportedProperty..........FAILED!");
    return;
  }

  Serial.println("IoTHubClient sendReportedProperty completed!");
}

static void set_device_desired_property(const char *name, int value)
{
  if (strcmp("fanSpeed", name) == 0)
  {
    fanSpeed = value;
    if (fan_running)
    {
      start_motor_with_speed(fanSpeed);
    }
  }
  else if (strcmp("temThreshold", name) == 0)
  {
    temThreshold = value;
  }
}

static void deviceTwinCallback(DEVICE_TWIN_UPDATE_STATE update_state, const unsigned char *payLoad, size_t size, void *userContextCallback)
{
  Serial.print("Device Twin Callback method with : ");
  Serial.println((const char *)payLoad);

  JSON_Value *root_value = json_parse_string((const char *)payLoad);
  JSON_Object *root_object = json_value_get_object(root_value);

  if (update_state == DEVICE_TWIN_UPDATE_PARTIAL)
  {
    Serial.print("DEVICE_TWIN_UPDATE_STATE is :");
    Serial.println(update_state);

    int version = (uint8_t)json_object_dotget_number(root_object, "$version");
    for (int i = 0, count = json_object_get_count(root_object); i < count; i++)
    {
      const char *partialName = json_object_get_name(root_object, i);
      if (partialName != NULL && partialName[0] != '$')
      {
        JSON_Object *partialObject = json_object_dotget_object(root_object, partialName);
        int partialValue = (uint8_t)json_object_dotget_number(partialObject, "value");

        (void)sprintf_s(propText, sizeof(propText),
                        "{\"%s\":{\"value\":%d,\"status\":\"completed\",\"desiredVersion\":%d}}",
                        partialName, partialValue, version);

        sendReportedProperty(propText);
        set_device_desired_property(partialName, partialValue);
      }
    }
  }
  else
  {
    JSON_Object *desired, *reported;

    desired = json_object_dotget_object(root_object, "desired");
    reported = json_object_dotget_object(root_object, "reported");

    int version = (uint8_t)json_object_dotget_number(desired, "$version");
    for (int i = 0, count = json_object_get_count(desired); i < count; i++)
    {
      const char *itemName = json_object_get_name(desired, i);
      if (itemName != NULL && itemName[0] != '$')
      {
        int desiredVersion = 0, value = 0;

        JSON_Object *itemObject = json_object_dotget_object(desired, itemName);
        value = (uint8_t)json_object_dotget_number(itemObject, "value");
        set_device_desired_property(itemName, value);
        JSON_Object *keyObject = json_object_dotget_object(reported, itemName);
        if (keyObject != NULL)
        {
          desiredVersion = (uint8_t)json_object_dotget_number(keyObject, "desiredVersion");
        }

        if (keyObject != NULL && (version == desiredVersion))
        {
          Serial.print("key: ");
          Serial.print(itemName);
          Serial.println(" found in reported and versions match.");
        }
        else
        {
          Serial.print("key: ");
          Serial.print(itemName);
          Serial.println(" either not found in reported or versions do not match.");

          (void)sprintf_s(propText, sizeof(propText),
                          "{\"%s\":{\"value\":%d,\"status\":\"completed\",\"desiredVersion\":%d}}",
                          itemName, value, version);
          sendReportedProperty(propText);
        }
      }
    }
  }
  json_value_free(root_value);
}

static int deviceMethodCallback(const char *method_name, const unsigned char *payload, size_t size, unsigned char **response, size_t *response_size, void *userContextCallback)
{
  (void)userContextCallback;
  (void)payload;
  (void)size;

  int result = 200;

  Serial.print("Executed direct method: ");
  Serial.println(method_name);

  Serial.print("Executed direct method payload: ");
  Serial.println((const char *)payload);

  if (strcmp("echo", method_name) == 0)
  {

    JSON_Value *root_value = json_parse_string((const char *)payload);
    JSON_Object *root_object = json_value_get_object(root_value);

    JSON_Value *displayedValue;
    displayedValue = json_object_dotget_value(root_object, "displayedValue");
    if (displayedValue != NULL)
    {
      const char *message = json_value_get_string(displayedValue);

      showHumitureScreen = false;
      oled_show_message(message);
    }
    json_value_free(root_value);
  }
  else if (strcmp("startFan", method_name) == 0)
  {
    fan_running_with_command = true;
    start_motor_with_speed(fanSpeed);
    fan_running = true;
  }
  else if (strcmp("stopFan", method_name) == 0)
  {
    fan_running_with_command = false;
    stop_motor();
    fan_running = false;
  }
  const char *responseMessage = "{ \"Response\": \"Successful\" }";
  *response_size = strlen(responseMessage);
  *response = (unsigned char *)strdup(responseMessage);

  return result;
}

static void connectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void *user_context)
{
  (void)reason;
  (void)user_context;

  Serial.print("iotHubClient connectionStatusCallback result: ");
  Serial.println(result);

  if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
  {
    Serial.println("Network connection.");
  }
  else
  {
    Serial.println("No network connection.");
    Serial.println("Needs reconnect to the IoT Hub.");
    needs_reconnect = true;
  }
}

static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *userContextCallback)
{
  EVENT_MESSAGE_INSTANCE_TAG *eventInstance = (EVENT_MESSAGE_INSTANCE_TAG *)userContextCallback;
  size_t id = eventInstance->messageTrackingId;

  Serial.print("Confirmation received for message tracking id = ");
  Serial.print(id);
  Serial.print(" with result = ");
  Serial.println(ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));

  IoTHubMessage_Destroy(eventInstance->messageHandle);
  free(eventInstance);
  /* Turn off Azure LED */
  toggle_azure_led(0);
}

static void reportedStateCallback(int status_code, void *userContextCallback)
{
  (void)userContextCallback;
  Serial.print("Device Twin reported properties update completed with result: ");
  Serial.println(status_code);
}

static void SetSensorState(float temperature)
{
  if (temperature < temThreshold)
  {
    if (fan_running && !fan_running_with_command)
    {
      stop_motor();
      fan_running = false;
    }
  }
  else
  {
    if (!fan_running && !fan_running_with_command)
    {
      start_motor_with_speed(fanSpeed);
      fan_running = true;
    }
  }
}

static void button_tap_cb(void *arg)
{
  Serial.println("Button tapped");
  // Toggle to show huimiture screen
  showHumitureScreen = ~showHumitureScreen;
}

void setup()
{
  M5.begin(true, true, true); //screen, batt, serial
  porthub.begin();

  // Lcd display setup
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);

  Serial.begin(115200);
  Serial.println("ESP32 Device");
  Serial.println("Initializing...");
  Serial.println(" > WiFi");
  Serial.println("Starting connecting WiFi.");

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
//    setup_wifi();

   // From example code DELETE if not needed
   WiFi.mode(WIFI_AP);
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED)
   {
     delay(500);
     Serial.print(".");
     hasWifi = false;
   }
   hasWifi = true;
   // Turn on the WIFI LED
   toggle_wifi_led(1);

    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

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
  
  M5.Imu.Init();

  // initialize_sensors();
  // // Clean oled screen
  // oled_clean_screen();
  // // Set button tap callback
  // set_buton_tap_cb(button_tap_cb);

  Serial.println(" > IoT Hub");
  if (!initIotHubClient())
  {
    hasIoTHub = false;
    Serial.println("Initializing IoT hub failed.");
    return;
  }
  hasIoTHub = true;
  Serial.println("Start sending events.");
  send_interval_ms = millis();
  check_interval_ms = millis();

  //setup load cell scale
  //TODO Wait a sec then tare the load cell value
//  delay(500);  //wait for user to get off button before taring
//  scale.setOffset(scale.averageValue()); // tare scale
//  loadcell = 0;
}

void loop()
{
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
  // TODO Read from Load Cell Module
//  loadcell = scale.getGram();


  if (hasWifi && hasIoTHub)
  {
    if ((int)(millis() - send_interval_ms) >= TELEMETRY_INTERVAL)
    {
      //get sensor data
//      float temperature = get_temperature();
//      float humidity = get_humidity();
//      float ambientLight = get_ambientLight();
      // int pitch = 0, roll = 0, accelX = 0, accelY = 0, accelZ = 0;
//      float pressure, altitude;
//      int magnetometerX = 0, magnetometerY = 0, magnetometerZ = 0;

//      get_pitch_roll_accel(&pitch, &roll, &accelX, &accelY, &accelZ);
//      get_pressure_altitude(&pressure, &altitude);
//      get_magnetometer(&magnetometerX, &magnetometerY, &magnetometerZ);

      sprintf_s(msgText, sizeof(msgText),
                "{\"accX\":%.2f,\"accY\":%.2f,\"accZ\":%.2f,\
                \"gyroX\":%.2f,\"gyroY\":%.2f,\"gyroZ\":%.2f,\
                \"keyswitchState\":%d,\"plugState\":%d,\"startButtonState\":%d,\"resetButtonState\":%d,\
                \"pushButtonState\":%d,\"stopButtonState\":%d,\"batt1BtnState\":%d,\"batt2BtnState\":%d,\
                \"TS_button\":%d,\"TS_key\":%d,\"TS_plug\":%d,\"TS_batt1\":%d,\"TS_batt2\":%d,\
                \"trialTime\":%d,\"cumForce\":%.2f,\"ptsCollected\":%d,\"trialRunning\":%d}",
                accX, accY, accZ, 
                gyroX, gyroY, gyroZ, 
                keyswitchState, plugState, startBtnState, resetBtnState,
                buttonPushState, stopBtnState, batt1BtnState, batt2BtnState,
                TS_button, TS_key, TS_plug, TS_batt1, TS_batt2,
                usecCount, cumForce, ptsCollected, countStart);
                
//       sprintf_s(msgText, sizeof(msgText),
//                "{\"Temperature\":%.2f,\"Humidity\":%.2f,\"AmbientLight\":%.2f,\
//                \"Pitch\":%d,\"Roll\":%d,\"Pressure\":%.2f,\"Altitude\":%.2f,\
//                \"MagnetometerX\":%d,\"MagnetometerY\":%d,\"MagnetometerZ\":%d}",
//                temp, temp, temp, pitch, roll, accZ, temp,
//                accX, accY, accZ);

//      sprintf_s(msgText, sizeof(msgText),
//                "{\"Temperature\":%.2f,\"Humidity\":%.2f,\"AmbientLight\":%.2f,\
//                \"Pitch\":%d,\"Roll\":%d,\"Pressure\":%.2f,\"Altitude\":%.2f,\
//                \"MagnetometerX\":%d,\"MagnetometerY\":%d,\"MagnetometerZ\":%d}",
//                temperature, humidity, ambientLight, pitch, roll, pressure, altitude,
//                magnetometerX, magnetometerY, magnetometerZ);

      // update temperature and humidity on default screen
//      if (showHumitureScreen)
//      {
//        oled_update_humiture(temperature, humidity);
//      }

//      if (check_for_shake(accelX,accelY,accelZ))
//      {
//        int die = rand() % 6 + 1;
//        (void)sprintf_s(propText, sizeof(propText),
//                          "{\"dieNumber\":%d}", die);
//        sendReportedProperty(propText);
//      }

//      SetSensorState(temperature);      
      sendTelemetry(msgText);

      send_interval_ms = millis();
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
      //TODO Wait a sec then tare the load cell value
//      delay(500);  //wait for user to get off button before taring
//      scale.setOffset(scale.averageValue()); // tare scale
//      loadcell = 0;
      countStart = 1;
      startaccX = accX;
      startaccY = accY;
      startaccZ = accZ;
      digitalWrite(10, LOW); //turn on LED
      Serial.println("Trial Status: M5.BtnA pressed, Trial Started!");
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
    // Trial has ended
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
  M5.Lcd.printf("Smart Task Board\n");
//  M5.Lcd.printf("LoadCell: %0.2f\n", loadcell);
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
  
  //Serial.println(usecCount); //print out seconds to the serial monitor
  //Serial.printf("Key_TS: %d, Plug_TS: %d, Batt1_TS: %d, Batt2_TS: %d, Time: %d\n", TS_key, TS_plug, TS_batt1, TS_batt2, usecCount); //print out seconds to the serial monitor

  
  delay(10);
}
