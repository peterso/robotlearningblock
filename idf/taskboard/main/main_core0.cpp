/**
 * Robothon Task Board Firmware
 */

#include <hal/board/TaskBoardDriver_v1.hpp>

#include <util/Timing.hpp>
#include <network/HTTPServer.hpp>
#include <network/WifiManager.hpp>
#include <network/OTAUpdater.hpp>
#include <network/KaaHandler.hpp>
#include <hal/NonVolatileStorage.hpp>
#include <hal/ScreenController.hpp>
#include <hal/PbHubController.hpp>
#include <hal/HardwareLowLevelController.hpp>
#include <microros/MicroROS.hpp>
#include <microros/MicroROSController.hpp>
#include <version.hpp>

#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_wifi.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Forward declarations
struct WebSocketTaskArgs
{
    HTTPServer& http_server;
    TaskBoardDriver& task_board_driver;
    TaskExecutor& task_executor;
    MicroROSController& micro_ros_controller;
};

void websockets_task(
        void* args);

/**
 * @brief Get the task board implementation
 *
 * @details This functions returns an instance of the actual class that implements the current configuration
 * of the task board. This is useful to allow the user to change the task board implementation without
 * changing the main code.
 *
 * @param hardware_low_level_controller The hardware low level controller
 *
 * @return TaskBoardDriver& The task board implementation
 */
TaskBoardDriver& get_task_board_implementation(
        HardwareLowLevelController& hardware_low_level_controller)
{
    TaskBoardDriver_v1* task_board_driver = new TaskBoardDriver_v1(hardware_low_level_controller);

    return *task_board_driver;
}

/**
 * @brief System entry point
 */
extern "C" void app_main(
        void)
{
    // ------------------------
    // System Initialization
    // ------------------------

    // Initialize Non-Volatile Storage (NVS) flash memory
    // If no free pages are available or a new version is found, erase and reinitialize
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize TCP/IP stack for network communications
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop for system events
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Disable Power Save modes for improved performance
    esp_wifi_set_ps(WIFI_PS_NONE);

    // ------------------------
    // Hardware Initialization
    // ------------------------

    // Initialize M5Stack hardware
    M5.begin();

    // Initialize hardware controllers and drivers
    PbHubController pb_hub_controller;
    HardwareLowLevelController hardware_low_level_controller = {pb_hub_controller, M5};
    TaskBoardDriver& task_board_driver = get_task_board_implementation(hardware_low_level_controller);
    ScreenController screen_controller(hardware_low_level_controller);
    NonVolatileStorage non_volatile_storage(task_board_driver.get_unique_id());

    // Wait for PbHubController to initialize
    // This can take variable time after boot
    while (!pb_hub_controller.check_status())
    {
        ESP_LOGI("app_main", "Waiting for PbHubController to start");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // ------------------------
    // System Setup
    // ------------------------

    // Initialize task executor
    TaskExecutor task_executor(screen_controller, non_volatile_storage);

    // Intialize screen
    screen_controller.clear();
    screen_controller.print("FW: " + FW_VERSION_STRING);
    vTaskDelay(pdMS_TO_TICKS(1000));
    screen_controller.clear();
    screen_controller.print("-> TaskExecutor started");

    // Initialize micro-ROS for ROS communication
    MicroROSController micro_ros_controller;

    // Setup HTTP server for remote control
    HTTPServer http_server(task_board_driver, task_executor, micro_ros_controller, non_volatile_storage);

    // ------------------------
    // Network Configuration
    // ------------------------

    // Update board status to read button states
    task_board_driver.update();

    // Get references to buttons
    const SensorReader& BUTTON_A = *task_board_driver.get_sensor_by_name("ON_BOARD_BUTTON_A");
    const SensorReader& BUTTON_B = *task_board_driver.get_sensor_by_name("ON_BOARD_BUTTON_B");
    const SensorReader& BUTTON_PWR = *task_board_driver.get_sensor_by_name("ON_BOARD_BUTTON_PWR");
    const SensorReader& RED_BUTTON = *task_board_driver.get_sensor_by_name("RED_BUTTON");
    const SensorReader& BLUE_BUTTON = *task_board_driver.get_sensor_by_name("BLUE_BUTTON");

    // Check if system should start in local mode (Button A pressed during boot)
    const bool start_local_mode = BUTTON_A.read() == true;
    std::string ip_string = "";

    if (start_local_mode)
    {
        ESP_LOGI("app_main", "Starting in local mode");
        screen_controller.print("-> Starting in local mode");
        ip_string = "Local mode";
    }
    // Otherwise, initialize WiFi provisioning
    else
    {
        ESP_LOGI("app_main", "Initializing provisioning");

        // Check if WiFi credentials should be reset (Red + Blue buttons pressed)
        bool reset_provisioning = RED_BUTTON.read() == true && BLUE_BUTTON.read() == true;

        if (reset_provisioning)
        {
            screen_controller.print("-> Resetting WiFi credentials");
        }

        // Initialize WiFi manager
        WifiManager wifi_manager(http_server, "Robothon Task Board", reset_provisioning);

        // Display appropriate connection instructions based on provisioning state
        if (wifi_manager.is_provisioned())
        {
            screen_controller.print("-> Trying " + wifi_manager.get_ssid());
            screen_controller.print("-> Reset with red and blue buttons");
        }
        else
        {
            screen_controller.print("-> Connect to \"" + wifi_manager.get_provisioning_ssid() + "\"");
            screen_controller.print("-> Navigate to 192.168.4.1");
        }

        // Wait for WiFi connection
        wifi_manager.wait_for_connection();

        // Display connection info
        screen_controller.print("-> Connected to " + wifi_manager.get_ssid());
        screen_controller.print("-> IP: " + wifi_manager.get_ip());
        ip_string = wifi_manager.get_ip();
    }

    // ------------------------
    // Handle OTA Updates
    // ------------------------
    if (!start_local_mode)
    {
        OTAUpdater ota_updater(screen_controller);

        // If can update, wait up to 3 seconds to red button press
        if (ota_updater.can_update())
        {
            screen_controller.clear();

            screen_controller.print("-> Red button to update to " + ota_updater.get_latest_version());

            // Wait for red button press
            for (uint32_t i = 0; i < 300; i++)
            {
                task_board_driver.update();

                if (RED_BUTTON.read() == true)
                {
                    break;
                }

                vTaskDelay(pdMS_TO_TICKS(10));
            }

            // If red button pressed, start update
            if (RED_BUTTON.read() == true)
            {
                screen_controller.print("-> Updating firmware");
                ota_updater.update();
            }

            // Skipping update
            else
            {
                screen_controller.clear();
                screen_controller.print("-> IP: " + ip_string);
            }
        }
        else
        {
            screen_controller.print("-> No updates available");
        }
    }

    // ------------------------
    // Post Network Configuration
    // ------------------------

    // Initialize HTTP API endpoints
    http_server.initialize_taskboard_api();

    // Create Micro-ROS task on core 1
    MicroROSMainArgs micro_ros_args =
    {micro_ros_controller, task_board_driver, task_executor, xTaskGetCurrentTaskHandle()};

    TaskHandle_t microros_task_handle;
    constexpr uint8_t MICROROS_THREAD_CORE_AFFINITY = 1;
    constexpr uint32_t MICROROS_STACK_SIZE = 8192;
    constexpr uint8_t MICROROS_THREAD_PRIORITY = 4;
    xTaskCreatePinnedToCore(
        microros_main,
        "microros",
        MICROROS_STACK_SIZE,
        &micro_ros_args,
        MICROROS_THREAD_PRIORITY,
        &microros_task_handle,
        MICROROS_THREAD_CORE_AFFINITY);

    // Wait for tasks
    screen_controller.print("-> Waiting tasks to start");
    screen_controller.print("-> Press Button B for default task");

    // ------------------------
    // Kaa IoT initialization
    // ------------------------
    KaaHandler kaa_handler(task_board_driver);

    TaskHandle_t kaa_task_handle;
    constexpr uint8_t KAA_THREAD_CORE_AFFINITY = 0;
    constexpr uint32_t KAA_STACK_SIZE = 8192;
    constexpr uint8_t KAA_THREAD_PRIORITY = 1;

    auto kaa_main = [](void* arg) -> void
            {
                KaaHandler* kaa_handler = static_cast<KaaHandler*>(arg);

                while (true)
                {
                    kaa_handler->send_telemetry();
                    vTaskDelay(pdMS_TO_TICKS(250));
                }
            };

    xTaskCreatePinnedToCore(
        kaa_main,
        "kaa",
        KAA_STACK_SIZE,
        &kaa_handler,
        KAA_THREAD_PRIORITY,
        &kaa_task_handle,
        KAA_THREAD_CORE_AFFINITY);

    // ------------------------
    // Websocket server
    // ------------------------
    TaskHandle_t ws_task_handle;
    constexpr uint8_t WS_THREAD_CORE_AFFINITY = 0;
    constexpr uint32_t WS_STACK_SIZE = 8192;
    constexpr uint8_t WS_THREAD_PRIORITY = 0;

    WebSocketTaskArgs ws_args = {http_server, task_board_driver, task_executor, micro_ros_controller};

    xTaskCreatePinnedToCore(
        websockets_task,
        "ws",
        WS_STACK_SIZE,
        &ws_args,
        WS_THREAD_PRIORITY,
        &ws_task_handle,
        WS_THREAD_CORE_AFFINITY);

    // ------------------------
    // Main Control Loop
    // ------------------------
    while (true)
    {
        // Update board sensor readings
        task_board_driver.update();

        // Check timeout status of current task, block forever
        bool current_task_timeout = false;
        task_executor.execute_operation_on_task(portMAX_DELAY, [&](Task* task, Task* precondition)
                {
                    const bool active_current_task = task != nullptr;
                    const bool active_current_precondition = precondition != nullptr;

                    current_task_timeout = active_current_task && !active_current_precondition &&
                    task->timeout();
                });

        // Handle Button B press - Start/Restart default task
        if (BUTTON_B.read() == true)
        {
            ESP_LOGI("app_main", "Button B pressed, launching default task");
            task_executor.cancel_task();

            // Get and configure default task
            Task& main_task = task_board_driver.get_default_task();
            Task& precondition_main_task = task_board_driver.get_default_task_precondition();

            // Set this task as a human task if the power button is pressed
            main_task.set_human_task(BUTTON_PWR.read() == true);

            // Start task execution
            task_executor.run_task(main_task, precondition_main_task);

            // Button debounce delay
            while (BUTTON_B.read() == true)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                task_board_driver.update();
            }
        }
        // Handle Button A press - Cancel current task
        else if (BUTTON_A.read() == true || current_task_timeout)
        {
            ESP_LOGI("app_main", "Button A pressed, cancelling current task");
            task_executor.cancel_task();

            // Update screen with cancellation message
            screen_controller.clear();
            screen_controller.print(current_task_timeout ? "-> Task timeout" : "-> Task cancelled");

            if (!start_local_mode)
            {
                screen_controller.print("-> IP: " + ip_string);
            }

            screen_controller.print("-> Press button B for default task");

            // Notify Micro-ROS task if it's waiting for a goal
            if (microros_task_handle != nullptr)
            {
                xTaskNotify(microros_task_handle, MANUALLY_CANCELLED_TASK, eSetBits);
            }

            // Button debounce delay
            while (BUTTON_A.read() == true)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                task_board_driver.update();
            }
        }

        // If notifed by Micro-ROS task, cancel current task
        uint32_t value;

        if (pdPASS == xTaskNotifyWait(pdFALSE, 0xFFFFFFFF, &value, pdMS_TO_TICKS(0)))
        {
            // Update screen with cancellation message
            screen_controller.clear();
            screen_controller.print(value == MICROROS_TIMEOUT_TASK ? "-> Task timeout" : "-> Task cancelled");
        }

        // Update task execution status
        task_executor.update();

        // Main loop delay
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void websockets_task(
        void* arg)
{
    WebSocketTaskArgs* ws_args = static_cast<WebSocketTaskArgs*>(arg);
    HTTPServer& http_server = ws_args->http_server;
    TaskBoardDriver& task_board_driver = ws_args->task_board_driver;
    TaskExecutor& task_executor = ws_args->task_executor;
    MicroROSController& micro_ros_controller = ws_args->micro_ros_controller;

    const std::string& unique_id = task_board_driver.get_unique_id();

    auto send_task_board_status = [&http_server, &task_board_driver, &unique_id]()
            {
                JSONHandler json_handler(unique_id);
                json_handler.add_taskboard_status(task_board_driver);
                json_handler.add_custom("ws_data_type", "taskboard_status");

                char* status = json_handler.get_json_string();

                if (status != nullptr)
                {
                    http_server.send_data_to_clients((uint8_t*)status, strlen(status));
                }
            };

    auto send_system_status = [&http_server, &micro_ros_controller, &unique_id]()
            {
                JSONHandler json_handler(unique_id);
                json_handler.add_microros_info(micro_ros_controller);
                json_handler.add_freertos_summary();
                json_handler.add_custom("ws_data_type", "system_status");

                char* status = json_handler.get_json_string();

                if (status != nullptr)
                {
                    http_server.send_data_to_clients((uint8_t*)status, strlen(status));
                }
            };

    auto send_task_status = [&http_server, &task_executor, &unique_id]()
            {
                JSONHandler json_handler(unique_id);
                task_executor.execute_operation_on_task(portMAX_DELAY,
                        [&json_handler](Task* task, Task* precondition)
                        {
                            if (task != nullptr)
                            {
                                json_handler.add_task_status(*task, precondition);
                            }
                        });

                json_handler.add_custom("ws_data_type", "task_status");

                char* status = json_handler.get_json_string();

                if (status != nullptr)
                {
                    http_server.send_data_to_clients((uint8_t*)status, strlen(status));
                }
            };

    constexpr uint32_t LOOP_RATE_MS = 20;
    constexpr uint32_t TASK_BOARD_STATUS_RATE_MS = 20;
    constexpr uint32_t SYSTEM_STATUS_RATE_MS = 240;
    constexpr uint32_t TASK_STATUS_RATE_MS = 240;

    static_assert(TASK_BOARD_STATUS_RATE_MS % LOOP_RATE_MS == 0,
            "TASK_BOARD_STATUS_RATE_MS must be a multiple of LOOP_RATE_MS");
    static_assert(SYSTEM_STATUS_RATE_MS % LOOP_RATE_MS == 0,
            "SYSTEM_STATUS_RATE_MS must be a multiple of LOOP_RATE_MS");
    static_assert(TASK_STATUS_RATE_MS % LOOP_RATE_MS == 0, "TASK_STATUS_RATE_MS must be a multiple of LOOP_RATE_MS");

    uint32_t counter = 0;

    while (true)
    {
        // Send system status every TASK_BOARD_STATUS_RATE_MS
        if (counter % TASK_BOARD_STATUS_RATE_MS == 0)
        {
            send_task_board_status();
        }

        // Send system status every SYSTEM_STATUS_RATE_MS
        if (counter % SYSTEM_STATUS_RATE_MS == 0)
        {
            send_system_status();
        }

        // Send task status every TASK_STATUS_RATE_MS
        if (counter % TASK_STATUS_RATE_MS == 0)
        {
            send_task_status();
        }

        // Wait for the next loop iteration
        vTaskDelay(pdMS_TO_TICKS(LOOP_RATE_MS));
        counter += LOOP_RATE_MS;
    }
}
