// Note: This sample is to demonstrate Azure IoT Central client concepts only and is not a guide design principles or style
// Checking of return codes and error values shall be omitted for brevity.  Please practice sound engineering practices
// when writing production code.
// This is taken from https://github.com/iot-for-all/iot-central-high-availability-clients/tree/main/C


#include <stdio.h>
#include <stdlib.h>

#include "iothub.h"
#include "iothub_message.h"
#include "iothub_client_version.h"
#include "azure_c_shared_utility/threadapi.h"
#include "azure_c_shared_utility/shared_util_options.h"
#include "azure_c_shared_utility/hmacsha256.h"
#include "azure_c_shared_utility/buffer_.h"
#include "azure_c_shared_utility/azure_base64.h"

#include "iothub_device_client_ll.h"
#include "iothub_client_options.h"
#include "azure_prov_client/prov_device_ll_client.h"
#include "azure_prov_client/prov_security_factory.h"
#include "iothubtransportmqtt.h"
#include "azure_prov_client/prov_transport_mqtt_client.h"

#include "parson.h"

#include "conio.h" // specific to Windows OS

MU_DEFINE_ENUM_STRINGS_WITHOUT_INVALID(PROV_DEVICE_RESULT, PROV_DEVICE_RESULT_VALUE);
MU_DEFINE_ENUM_STRINGS_WITHOUT_INVALID(PROV_DEVICE_REG_STATUS, PROV_DEVICE_REG_STATUS_VALUES);

// cleaner way to indicate where parameters are not used
#define UNREFERENCED_PARAMETER(P) \
        { \
            (P) = (P); \
        } \

// structure definitions
typedef struct CLIENT_SAMPLE_INFO_TAG
{
    unsigned int sleep_time;
    char* iothub_uri;
    char* access_key_name;
    char* device_key;
    char* device_id;
    int registration_complete;
} CLIENT_SAMPLE_INFO;

typedef struct IOTHUB_CLIENT_SAMPLE_INFO_TAG
{
    int connected;
    int terminate;
} IOTHUB_CLIENT_SAMPLE_INFO;

typedef struct ThreadData {
    int telemetrySendFrequency;
    int reportedPropertySendFrequency;
} THREADDATA, * PTHREADDATA;


// device settings - FILL IN YOUR VALUES HERE
#define SCOPE_ID "0ne00331371";
#define GROUP_SYMMETRIC_KEY "HmSXtW41bA/P2atoawouLkS3RviICH3FM8NzVc0pxkFKSTh6DKgXU5zB8EHUnLmNdZZRBMScmN1QLd1efxILCg==";

// optional device settings - CHANGE IF DESIRED/NECESSARY
#define DEVICE_ID "failover_c"
#define GLOBAL_PROVISIONING_URI "global.azure-devices-provisioning.net"
#define MODEL_ID "dtmi:Sample:Failover;1"

// test setting flags
static bool telemetrySendOn = true;
static bool reportedPropertySendOn = true;
static bool desiredPropertyReceiveOn = true;
static bool directMethodReceiveOn = true;
static bool c2dCommandReceiveOn = true;

// global general purpose variables
static IOTHUB_DEVICE_CLIENT_LL_HANDLE device_ll_handle;
static IOTHUB_CLIENT_SAMPLE_INFO iothub_info;
static CLIENT_SAMPLE_INFO user_ctx;
static bool traceOn = false;  // set true for verbose tracing from the Azure IoT SDK

// forward declarationd
void connectToCentral();


// generate the device key using the symetric group key
static bool generateDeviceKey(const char* device_id, const char* group_sas_key, char *device_key)
{
    BUFFER_HANDLE hkey = Azure_Base64_Decode(group_sas_key);
    if (hkey == NULL)
    {
        printf("Generating the device key failed on the base64 decode\r\n");
        return false;
    }
    BUFFER_HANDLE hhash = BUFFER_new();
    const unsigned char* keyContent;
    BUFFER_content(hkey, &keyContent);
    HMACSHA256_RESULT res = HMACSHA256_ComputeHash((const unsigned char*)keyContent, 64, (const unsigned char*)device_id, strlen(device_id), hhash);
    if (res != HMACSHA256_OK)
    {
        printf("Generating the device key failed on the hmac\r\n");
        return false;
    }

    strcpy(device_key, STRING_c_str(Azure_Base64_Encode(hhash)));

    return true;
}


// handler for reported property hub received confirmation messages
static void reportedStateCallback(int status_code, void* userContextCallback)
{
    UNREFERENCED_PARAMETER(userContextCallback);
    printf("Device Twin reported properties update completed with result: %d\r\n", status_code);
}


// handler for device registration status events
static void deviceRegistrationStatusCallback(PROV_DEVICE_REG_STATUS reg_status, void* user_context)
{
    (void)user_context;
    (void)printf("Provisioning Status: %s\r\n", MU_ENUM_TO_STRING(PROV_DEVICE_REG_STATUS, reg_status));
}


// device registered in DPS callback
static void deviceRegistrationCallback(PROV_DEVICE_RESULT register_result, const char* iothub_uri, const char* device_id, void* user_context)
{
    if (user_context == NULL)
    {
        printf("user_context is NULL\r\n");
    }
    else
    {
        CLIENT_SAMPLE_INFO* user_ctx_foo = (CLIENT_SAMPLE_INFO*)user_context;
        if (register_result == PROV_DEVICE_RESULT_OK)
        {
            (void)printf("Registration Information received from service: %s!\r\n", iothub_uri);
            (void)mallocAndStrcpy_s(&user_ctx_foo->iothub_uri, iothub_uri);
            (void)mallocAndStrcpy_s(&user_ctx_foo->device_id, device_id);
            user_ctx_foo->registration_complete = 1;
        }
        else
        {
            (void)printf("Failure encountered on registration %s\r\n", MU_ENUM_TO_STRING(PROV_DEVICE_RESULT, register_result));
            user_ctx_foo->registration_complete = 2;
        }
    }
}


// handler for disconnects, reconnect via DPS
static void connectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* user_context)
{
    UNREFERENCED_PARAMETER(reason);
    if (user_context == NULL)
    {
        printf("iothub_connection_status user_context is NULL\r\n");
    }
    else
    {
        IOTHUB_CLIENT_SAMPLE_INFO* iothub_info_local_copy = (IOTHUB_CLIENT_SAMPLE_INFO*)user_context;
        if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
        {
            iothub_info_local_copy->connected = 1;
        }
        else
        {
            iothub_info_local_copy->connected = 0;
            connectToCentral();
        }
    }
}


// handles the Cloud to Device (C2D) message setAlarm
static IOTHUBMESSAGE_DISPOSITION_RESULT cloudToDeviceCallback(IOTHUB_MESSAGE_HANDLE message, void* user_context)
{
    UNREFERENCED_PARAMETER(user_context);

    const char* methodName = IoTHubMessage_GetProperty(message, "method-name");
    if (strcmp(methodName, "setAlarm") == 0)
    {
        const unsigned char* buff_msg;
        size_t buff_len;
        if (IoTHubMessage_GetByteArray(message, &buff_msg, &buff_len) != IOTHUB_MESSAGE_OK)
        {
            (void)printf("Failure retrieving byte array message\r\n");
        }
        else
        {
            (void)printf("Request to set alarm received.  Setting alarm for %.*s\r\n", (int)buff_len, buff_msg);
        }
    }
    else
    {
        printf("Unknown command received\r\n");
        return IOTHUBMESSAGE_REJECTED;
    }

    return IOTHUBMESSAGE_ACCEPTED;;
}


// handles direct method calls from IoT Central (or hub)
static int directMethodCallback(const char* method_name, const unsigned char* payload, size_t size, unsigned char** response, size_t* response_size, void* userContextCallback)
{
    UNREFERENCED_PARAMETER(userContextCallback);
    
    int result = 0;
    // look to see if we have an expected 'echo' direct method call
    if (strcmp(method_name, "echo") == 0)
    {
        // printing to the screen our Method
        printf("Echo direct method called with parameter: %.*s\r\n", size, payload);

        // sending a response to the Method
        result = 200;
        char responseMessage[128] = { 0 };
        sprintf(responseMessage, "%.*s", size, payload);
        *response_size = size;
        *response = (unsigned char*)malloc(*response_size);
        (void)memcpy(*response, responseMessage, *response_size);
    }
    else
    {
        printf("Unknown direct method called: %s Payload: %.*s\r\n", method_name, size, payload);
        result = 404;
    }

    return result;
}


// handles desired properties from IoT Central (or hub)
static void deviceTwinCallback(DEVICE_TWIN_UPDATE_STATE update_state, const unsigned char* payload, size_t size, void* userContextCallback)
{
    UNREFERENCED_PARAMETER(userContextCallback);

    if (update_state == DEVICE_TWIN_UPDATE_COMPLETE)
    {
        printf("Complete twin received with payload: %.*s\r\n", size, (char*)payload);
    }
    else if (update_state == DEVICE_TWIN_UPDATE_PARTIAL)
    {
        JSON_Value* jsonData = json_parse_string((char*)payload);
        JSON_Object* rootObject = json_object(jsonData);
        if (json_object_get_count(rootObject) < 2)
        {
            printf("The JSON received does not conform to what is expected from IoT Central\r\n");
        }
        else
        {
            const char* key = json_object_get_name(rootObject, 0);
            if (strcmp(key, "$version") == 0)
            {
                key = json_object_get_name(rootObject, 1);
            }
            // look to see if we get an expected fanSpeed desirted property
            if (strcmp(key, "fanSpeed") == 0)
            {
                printf("Fan speed change received setting fan speed to %.0f rpm\r\n", json_object_get_number(rootObject, key));

                // acknowledge back to IoT Central the desired property
                char ackPayload[256];
                sprintf(ackPayload, "{\"%s\":{\"value\": %f, \"ac\":200, \"ad\":\"completed\", \"av\": %.0f}}", key, json_object_get_number(rootObject, key), json_object_get_number(rootObject, "$version"));
                if (IoTHubDeviceClient_LL_SendReportedState(device_ll_handle, (const unsigned char*)ackPayload, strlen(ackPayload), reportedStateCallback, NULL) != IOTHUB_CLIENT_OK)
                {
                    (void)printf("ERROR: IoTHubClient_LL_SendReportedState..........FAILED!\r\n");
                }
                else
                {
                    (void)printf("IoTHubClient_LL_SendReportedState accepted message for transmission to IoT Hub.\r\n");

                }
            }
            else
            {
                printf("Unknown desired property: %s\r\n", key);
            }
        }
        json_value_free(jsonData);
    }
}



// sends telemetry on a set frequency
static int sendTelemetry(void* threadArgument)
{
    PTHREADDATA threadData = (PTHREADDATA)threadArgument;

    while (!iothub_info.terminate)
    {
        if (iothub_info.connected != 0)
        {
            static char msgText[128];
            sprintf_s(msgText, sizeof(msgText), "{ \"temp\" : %.2f, \"humidity\" : %.2f }", 20.2, 40.4);

            IOTHUB_MESSAGE_HANDLE msg_handle = IoTHubMessage_CreateFromByteArray((const unsigned char*)msgText, strlen(msgText));
            if (msg_handle == NULL)
            {
                (void)printf("ERROR: iotHubMessageHandle is NULL!\r\n");
            }
            else
            {
                if (IoTHubDeviceClient_LL_SendEventAsync(device_ll_handle, msg_handle, NULL, NULL) != IOTHUB_CLIENT_OK)
                {
                    (void)printf("ERROR: IoTHubClient_LL_SendEventAsync..........FAILED!\r\n");
                }
                else
                {
                    (void)printf("IoTHubClient_LL_SendEventAsync accepted message for transmission to IoT Hub.\r\n");

                }
                IoTHubMessage_Destroy(msg_handle);
            }
            ThreadAPI_Sleep(threadData->telemetrySendFrequency);
        }
        else
        {
            ThreadAPI_Sleep(1);
        }
    }

    return 0;
}


// sends reported properties on a set frequency
static int sendReportedProperty(void* threadArgument)
{
    PTHREADDATA threadData = (PTHREADDATA)threadArgument;

    while (!iothub_info.terminate)
    {
        if (iothub_info.connected != 0)
        {
            static char propertyText[128];
            sprintf_s(propertyText, sizeof(propertyText), "{ \"battery\" : %.2f }", 60.6);

            if (IoTHubDeviceClient_LL_SendReportedState(device_ll_handle, (const unsigned char*)propertyText, strlen(propertyText), reportedStateCallback, NULL) != IOTHUB_CLIENT_OK)
            {
                (void)printf("ERROR: IoTHubClient_LL_SendReportedState..........FAILED!\r\n");
            }
            else
            {
                (void)printf("IoTHubClient_LL_SendReportedState accepted message for transmission to IoT Hub.\r\n");

            }
            ThreadAPI_Sleep(threadData->reportedPropertySendFrequency);
        }
        else
        {
            ThreadAPI_Sleep(1);
        }
    }

    return 0;
}


// message loop making sure that the SDK has time to process outgoing and incoming messages
static int messageLoop(void* threadArgument)
{
    UNREFERENCED_PARAMETER(threadArgument);
    while (!iothub_info.terminate)
    {
        IoTHubDeviceClient_LL_DoWork(device_ll_handle);
        ThreadAPI_Sleep(1);
    }

    return 0;
}


// monitor for the ESC key to be pressed for exiting the program
static int keyboardMonitor(void* threadArgument)
{
    // this is a Windows implementation and will not work on other OS's
    // substitute for stdio getchar() or OS specific implementation
    UNREFERENCED_PARAMETER(threadArgument);

    while (true)
    {
        if (_getch() == 27)  // was the ESC key pressed
        {
            iothub_info.terminate = 1;
            break;
        }

        ThreadAPI_Sleep(1);
    }

    return 0;
}


// connect to IoT Central/Hub via Device Provisioning Servicee (DPS)
void connectToCentral()
{
    // generate a device key for the DEVICE_ID value using the GROUP_SYMMETRIC_KEY
    char device_key[64] = { '\0' };
    generateDeviceKey(DEVICE_ID, GROUP_SYMMETRIC_KEY, device_key);

    SECURE_DEVICE_TYPE hsm_type;
    hsm_type = SECURE_DEVICE_TYPE_SYMMETRIC_KEY;

    (void)IoTHub_Init();
    (void)prov_dev_security_init(hsm_type);

    // Set the symmetric key if using they auth type
    prov_dev_set_symmetric_key_info(DEVICE_ID, device_key);

    PROV_DEVICE_TRANSPORT_PROVIDER_FUNCTION prov_transport = Prov_Device_MQTT_Protocol;

    memset(&user_ctx, 0, sizeof(CLIENT_SAMPLE_INFO));

    // Set ini
    user_ctx.registration_complete = 0;
    user_ctx.sleep_time = 10;

    printf("Provisioning API Version: %s\r\n", Prov_Device_LL_GetVersionString());
    printf("Iothub API Version: %s\r\n", IoTHubClient_GetVersionString());

    PROV_DEVICE_LL_HANDLE handle;
    if ((handle = Prov_Device_LL_Create(GLOBAL_PROVISIONING_URI, SCOPE_ID, prov_transport)) == NULL)
    {
        (void)printf("failed calling Prov_Device_LL_Create\r\n");
    }
    else
    {
        Prov_Device_LL_SetOption(handle, PROV_OPTION_LOG_TRACE, &traceOn);

        // set the device model in the registration payload for auto-association in IoT Central
        char reg_payload[128];
        sprintf(reg_payload, "{\"iotcModelId\":\"%s\"}", MODEL_ID);
        Prov_Device_LL_Set_Provisioning_Payload(handle, reg_payload);

        // register the device via DPS
        if (Prov_Device_LL_Register_Device(handle, deviceRegistrationCallback, &user_ctx, deviceRegistrationStatusCallback, &user_ctx) != PROV_DEVICE_RESULT_OK)
        {
            (void)printf("failed calling Prov_Device_LL_Register_Device\r\n");
        }
        else
        {
            do
            {
                Prov_Device_LL_DoWork(handle);
                ThreadAPI_Sleep(user_ctx.sleep_time);
            } while (user_ctx.registration_complete == 0);
        }
        Prov_Device_LL_Destroy(handle);
    }

    if (user_ctx.registration_complete != 1)
    {
        (void)printf("registration failed!\r\n");
    }
    else
    {
        IOTHUB_CLIENT_TRANSPORT_PROVIDER iothub_transport = MQTT_Protocol;

        (void)printf("Creating IoTHub Device handle\r\n");
        // connect to IoT hub
        if ((device_ll_handle = IoTHubDeviceClient_LL_CreateFromDeviceAuth(user_ctx.iothub_uri, user_ctx.device_id, iothub_transport)) == NULL)
        {
            (void)printf("failed create IoTHub client from connection string %s!\r\n", user_ctx.iothub_uri);
        }
        else
        {
            iothub_info.terminate = 0;
            iothub_info.connected = 0;

            // set verbose tracing in the SDK on or off
            IoTHubDeviceClient_LL_SetOption(device_ll_handle, OPTION_LOG_TRACE, &traceOn);

            // monitor the connection status changes
            (void)IoTHubDeviceClient_LL_SetConnectionStatusCallback(device_ll_handle, connectionStatusCallback, &iothub_info);

            // monitor for cloud to device messages
            if (c2dCommandReceiveOn)
            {
                (void)IoTHubDeviceClient_LL_SetMessageCallback(device_ll_handle, cloudToDeviceCallback, &iothub_info);
            }

            // monitor for direct method calls
            if (directMethodReceiveOn)
            {
                (void)IoTHubDeviceClient_LL_SetDeviceMethodCallback(device_ll_handle, directMethodCallback, NULL);
            }

            // monitor for desired property calls
            if (desiredPropertyReceiveOn)
            {
                (void)IoTHubDeviceClient_LL_SetDeviceTwinCallback(device_ll_handle, deviceTwinCallback, NULL);
            }
        }
    }
}


// Entry point: Connect the device and start processing telemetry, properties and commands
int main()
{
    // connect to IoT Central via DPS
    connectToCentral();

    const int THREAD_COUNT = 4;
    THREAD_HANDLE* hThreadArray = malloc(THREAD_COUNT * sizeof(THREAD_HANDLE*));

    if (hThreadArray == NULL)
    {
        printf("Error allocating memory for the threads\r\n");
        exit(2);
    }

    THREADDATA threadData;

    // set the send frequency for the telemetry and reported property threads
    threadData.telemetrySendFrequency = 5000;
    threadData.reportedPropertySendFrequency = 12000;

    // this thread will handle communication to/from the IoT Hub
    ThreadAPI_Create(&hThreadArray[0], messageLoop, NULL);

    // this thread monitors the keyboard for termination key press (ESC)
    ThreadAPI_Create(&hThreadArray[1], keyboardMonitor, NULL);

    // this thread will send telemetry to IoT Central every 5 seconds
    if (telemetrySendOn)
    {
        ThreadAPI_Create(&hThreadArray[2], sendTelemetry, &threadData);
    }

    // this thread will send reported properties to IoT Central every 10 seconds
    if (reportedPropertySendOn)
    {
        ThreadAPI_Create(&hThreadArray[3], sendReportedProperty, &threadData);
    }

    printf("\r\n\r\n*** Press the ESC key in the console window to stop the device ***\r\n\r\n");

    // wait for the threads to exit
    int threadResult = 0;
    ThreadAPI_Join(hThreadArray[0], &threadResult);
    ThreadAPI_Join(hThreadArray[1], &threadResult);
    if (telemetrySendOn)
    {
        ThreadAPI_Join(hThreadArray[2], &threadResult);
    }
    if (reportedPropertySendOn)
    {
        ThreadAPI_Join(hThreadArray[3], &threadResult);
    }

    // Clean up the iothub sdk handle
    IoTHubDeviceClient_LL_Destroy(device_ll_handle);

    free(hThreadArray);
    free(user_ctx.iothub_uri);
    free(user_ctx.device_id);
    prov_dev_security_deinit();

    // Free all the sdk subsystem
    IoTHub_Deinit();

    return 0;  // exit
}
