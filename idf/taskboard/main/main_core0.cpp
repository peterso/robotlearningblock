/**
 * Roboton Task Board Firmware
 */

#include <hal/board/TaskBoardDriver_v1.hpp>

#include <util/Timing.hpp>
#include <network/HTTPServer.hpp>
#include <network/WifiManager.hpp>
#include <hal/NonVolatileStorage.hpp>
#include <hal/ScreenController.hpp>
#include <hal/PbHubController.hpp>
#include <hal/HardwareLowLevelController.hpp>
#include <microros/MicroROS.hpp>
#include <microros/MicroROSController.hpp>

#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_wifi.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

TaskBoardDriver & get_task_board_implementation(HardwareLowLevelController & hardware_low_level_controller)
{
    TaskBoardDriver_v1 * task_board_driver = new TaskBoardDriver_v1(hardware_low_level_controller);

    return *task_board_driver;
}

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());

    // Initialize event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize hardware
    M5.begin();
    PbHubController pb_hub_controller;
    HardwareLowLevelController hardware_low_level_controller = {pb_hub_controller, M5};
    TaskBoardDriver & task_board_driver = get_task_board_implementation(hardware_low_level_controller);
    ScreenController screen_controller(hardware_low_level_controller);
    NonVolatileStorage non_volatile_storage(task_board_driver.get_unique_id());

    // Ensure that PbHubController is up and running
    // Sometimes it takes a while for the PbHubController to start after boot
    while (!pb_hub_controller.check_status())
    {
        ESP_LOGI("app_main", "Waiting for PbHubController to start");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Start task executor
    TaskExecutor task_executor(screen_controller, non_volatile_storage);
    screen_controller.clear();
    screen_controller.print("-> TaskExecutor started");

    // Initialize Micro-ROS node
    MicroROSController micro_ros_controller;

    // Initilize HTTP server
    HTTPServer http_server(task_board_driver, task_executor, micro_ros_controller, non_volatile_storage);

    // Initialize Wi-Fi provisioning
    screen_controller.clear();

    // Update status to read buttons
    task_board_driver.update();

    // Get references to sensors used in main loop
    const SensorReader & BUTTON_A = *task_board_driver.get_sensor_by_name("ON_BOARD_BUTTON_A");
    const SensorReader & BUTTON_B = *task_board_driver.get_sensor_by_name("ON_BOARD_BUTTON_B");
    const SensorReader & BUTTON_PWR = *task_board_driver.get_sensor_by_name("ON_BOARD_BUTTON_PWR");
    const SensorReader & RED_BUTTON = *task_board_driver.get_sensor_by_name("RED_BUTTON");
    const SensorReader & BLUE_BUTTON = *task_board_driver.get_sensor_by_name("BLUE_BUTTON");

    if(BUTTON_A.read() == true)
    {
        ESP_LOGI("app_main", "Starting in local mode");
        screen_controller.print("-> Starting in local mode");
    }
    else
    {
        ESP_LOGI("app_main", "Initializing provisioning");
        bool reset_provisioning =
            RED_BUTTON.read() == true &&
            BLUE_BUTTON.read() == true;

        if (reset_provisioning)
        {
            screen_controller.print("-> Resetting WiFi credentials");
        }

        WifiManager wifi_manager(http_server, "Roboton Task Board", reset_provisioning);

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

        wifi_manager.wait_for_connection();

        screen_controller.print("-> Connected to " + wifi_manager.get_ssid());
        screen_controller.print("-> IP: " + wifi_manager.get_ip());
    }

    // Initialize task board API
    http_server.initialize_taskboard_api();

    // Initialize micro-ROS task con core 1
    MicroROSMainArgs micro_ros_args = {micro_ros_controller, task_board_driver, task_executor};
    TaskHandle_t microros_task_handle;
    xTaskCreatePinnedToCore(microros_main, "microros", 20000, &micro_ros_args, 4, &microros_task_handle, 1);

    // Wait for red button press to start
    screen_controller.print("-> Waiting tasks to start");
    screen_controller.print("-> Press Button B for default task");

    // Main loop
    while(true)
    {
        // Update board status
        task_board_driver.update();

        // Reset task if Button B is pressed
        if(BUTTON_B.read() == true)
        {
            ESP_LOGI("app_main", "Button B pressed, restarting task");
            task_executor.cancel_task();

            Task & main_task = task_board_driver.get_default_task();
            Task & precondition_main_task = task_board_driver.get_default_task_precondition();

            main_task.restart();
            main_task.set_human_task(BUTTON_PWR.read() == true);
            precondition_main_task.restart();

            task_executor.run_task(main_task, precondition_main_task);

            // Debounce
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // If A button is pressed, cancel tasks
        else if(BUTTON_A.read() == true)
        {
            ESP_LOGI("app_main", "Button A pressed, cancelling task");
            task_executor.cancel_task();

            screen_controller.clear();
            screen_controller.print("-> Task cancelled");
            screen_controller.print("-> IP: " + micro_ros_controller.get_agent_ip());
            screen_controller.print("-> Press button B for default task");

            // Notify micro-ROS task for the case where it is waiting for a goal
            if (microros_task_handle != nullptr)
            {
                xTaskNotify(microros_task_handle, MANUALLY_CANCELLED_TASK, eSetBits);
            }

            // Debounce
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Update task executor
        task_executor.update();

        // Cyclic delay
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
