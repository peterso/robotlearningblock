/**
 * Azure IoT Central example for esp32-azure-kit.
 */
#include <WiFi.h>
#include "AzureIotHub.h"
#include "src/parson.h"
#include "src/sensor_manager.h"
#include "src/led.h"

#define TELEMETRY_INTERVAL 1000

// Please input the SSID and password of WiFi
const char *ssid = "FRITZ!Box 7530 WE";
const char *password = "70348511462386919316";

/*String containing Hostname, Device Id & Device Key in the format:                         */
//const char *HostName="msrm-connected-taskboards.azureiotcentral.com";
//const char *DeviceId="sample-device-01";
//const char *SharedAccessKey="sample-device-01";
//const char *SharedAccessSignature="P5Q4iyJ3lA5RQfXGNjBh894/saC3C9GGoPK/qJFKouU=";

//static const char *connectionString = "HostName=iot-task-boards-hub.azure-devices.net;DeviceId=m5stickcplus-task-board;SharedAccessKey=Zim7IivgoWnuBqqQKjedO0bukRGlUZf/NFC64BmLT7Q=";

// handcrafted connection string
static const char *connectionString = "HostName=iotc-111c2522-561d-4f03-9267-9c169ad02fe9.azure-devices.net;DeviceId=kresimir-hand-holding;SharedAccessKey=2zqujYgAegVVflrSQuuF7Nn75Cs9CQxU0i12SIx2vG0=";

/*  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessKey=<device_key>"                */
/*  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessSignature=<device_sas_token>"    */
//static const char *connectionString = "";

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
  Serial.begin(115200);
  Serial.println("ESP32 Device");
  Serial.println("Initializing...");
  Serial.println(" > WiFi");
  Serial.println("Starting connecting WiFi.");

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

  initialize_sensors();
  // Clean oled screen
  oled_clean_screen();
  // Set button tap callback
  set_buton_tap_cb(button_tap_cb);

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
}

void loop()
{
  if (hasWifi && hasIoTHub)
  {
    if ((int)(millis() - send_interval_ms) >= TELEMETRY_INTERVAL)
    {
      //get sensor data
      float temperature = get_temperature();
      float humidity = get_humidity();
      float ambientLight = get_ambientLight();
      int pitch = 0, roll = 0, accelX = 0, accelY = 0, accelZ = 0;
      float pressure, altitude;
      int magnetometerX = 0, magnetometerY = 0, magnetometerZ = 0;

      get_pitch_roll_accel(&pitch, &roll, &accelX, &accelY, &accelZ);
      get_pressure_altitude(&pressure, &altitude);
      get_magnetometer(&magnetometerX, &magnetometerY, &magnetometerZ);

      sprintf_s(msgText, sizeof(msgText),
                "{\"Temperature\":%.2f,\"Humidity\":%.2f,\"AmbientLight\":%.2f,\
                \"Pitch\":%d,\"Roll\":%d,\"Pressure\":%.2f,\"Altitude\":%.2f,\
                \"MagnetometerX\":%d,\"MagnetometerY\":%d,\"MagnetometerZ\":%d}",
                temperature, humidity, ambientLight, pitch, roll, pressure, altitude,
                magnetometerX, magnetometerY, magnetometerZ);

      // update temperature and humidity on default screen
      if (showHumitureScreen)
      {
        oled_update_humiture(temperature, humidity);
      }

      if (check_for_shake(accelX,accelY,accelZ))
      {
        int die = rand() % 6 + 1;
        (void)sprintf_s(propText, sizeof(propText),
                          "{\"dieNumber\":%d}", die);
        sendReportedProperty(propText);
      }

      SetSensorState(temperature);      
      sendTelemetry(msgText);

      send_interval_ms = millis();
    }
  }
  delay(10);
}
