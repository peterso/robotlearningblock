/**
 * Robothon Task Board Firmware
 */

#include <microros/MicroROSTaskExecutor.hpp>
#include <microros/MicroROS.hpp>
#include <microros/MicroROSTypes.hpp>
#include <microros/MicroROSTask.hpp>
#include <util/Timing.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Timing configuration for periodic publishers
constexpr uint32_t TASKBOARD_STATUS_PUBLISHER_PERIOD_MS = 50;
constexpr uint32_t TASK_STATUS_PUBLISHER_PERIOD_MS = 40;

/**
 * @brief micro-ROS main function
 *
 * @details This function runs on Core 1 and handles all ROS communication and task status updates.
 * It manages the interface between the task board and ROS, publishing board status and task states
 * while handling incoming ROS commands.
 *
 * @details It is expected to be launched by main function and attached to Core 1.
 */
void microros_main(
        void* arg)
{
    // ------------------------
    // Initialize Controllers
    // ------------------------

    // Cast input argument to access controller references
    MicroROSMainArgs* args = (MicroROSMainArgs*)arg;
    MicroROSController& microros_controller = args->microros_controller;
    TaskBoardDriver& task_board_driver = args->task_board_driver;
    TaskExecutor& task_executor = args->task_executor;
    TaskHandle_t main_task_handle = args->main_task_handle;

    // Initialize Micro-ROS task executor to handle ROS-based task execution
    MicroROSTaskExecutor microros_task_executor(task_executor, microros_controller, task_board_driver,
            main_task_handle);

    // ------------------------
    // Setup Periodic Publishers
    // ------------------------

    // Configure periodic board status publisher
    // Publishes sensor states and board configuration to ROS
    TimedOperation publish_taskboard_status(TASKBOARD_STATUS_PUBLISHER_PERIOD_MS,
            [&microros_controller, &task_board_driver]()
            {
                MicroROSTypes::TaskBoardStatus status(task_board_driver);
                microros_controller.publish_taskboard_status(status);
            });

    // ------------------------
    // Task Executor Callback
    // ------------------------
    // In order to avoid deadlock, the task executor callback will feed a queue of messages to be sent to micro-ROS
    // This way, the micro-ROS task can handle the messages at its own pace
    QueueHandle_t task_executor_queue = xQueueCreate(10, sizeof(MicroROSTypes::TaskStatus*));

    task_executor.add_callback(
        TASK_STATUS_PUBLISHER_PERIOD_MS,
        [&task_executor_queue, &task_board_driver](const Task* task, const Task* precondition)
        {
            // Create a new task status message
            MicroROSTypes::TaskStatus* task_status = nullptr;

            if (nullptr != task)
            {
                // If there's an active task, publish its status
                task_status = new MicroROSTypes::TaskStatus(*task, task_board_driver.get_unique_id());
                task_status->set_waiting_precondition(nullptr != precondition);

                if (precondition != nullptr)
                {
                    task_status->set_time(0);
                }
            }
            else
            {
                // If no active task, publish empty status
                task_status = new MicroROSTypes::TaskStatus(task_board_driver.get_unique_id());
            }

            // If the queue add fails, delete the message
            if (nullptr != task_status && pdTRUE != xQueueSend(task_executor_queue, &task_status, 0))
            {
                ESP_LOGE("microros_main", "Task status queue full, skipping message");
                delete task_status;
            }
        });

    // ------------------------
    // Main micro-ROS Update Loop
    // ------------------------
    while (true)
    {
        // Update ROS communication and publish periodic messages
        microros_controller.update();
        publish_taskboard_status.update();
        microros_task_executor.update();

        // Consume the task executor queue
        MicroROSTypes::TaskStatus* task_status = nullptr;

        if (pdTRUE == xQueueReceive(task_executor_queue, &task_status, 0))
        {
            if (task_status != nullptr)
            {
                microros_controller.publish_task_status(*task_status);
                delete task_status;
            }
        }

        // Check for notifications
        uint32_t value;

        if (pdPASS == xTaskNotifyWait(pdFALSE, 0xFFFFFFFF, &value, 0))
        {
            // Check for manual task cancellation
            if (value & MANUALLY_CANCELLED_TASK)
            {
                microros_task_executor.cancel_task();
            }

        }

        // Main loop delay
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
