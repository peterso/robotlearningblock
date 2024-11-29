/**
 * Robothon Task Board Firmware
 */

#include <microros/MicroROSTaskExecutor.hpp>
#include <microros/MicroROS.hpp>
#include <microros/MicroROSTypes.hpp>
#include <microros/MicroROSTask.hpp>
#include <util/Timing.hpp>

// Timing configuration for periodic publishers
constexpr uint32_t TASKBOARD_STATUS_PUBLISHER_PERIOD_MS = 250;
constexpr uint32_t TASK_STATUS_PUBLISHER_PERIOD_MS = 500;

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
    // In order to avoid deadlock, let's register a callback to the task executor that defers the
    // micro-ROS publish operation.
    TaskHandle_t microros_task_handle = xTaskGetCurrentTaskHandle();
    task_executor.add_callback(
        TASK_STATUS_PUBLISHER_PERIOD_MS,
        [&microros_task_handle](const Task* task, const Task* precondition)
        {
            // Notify micro-ROS task that a task update is required
            xTaskNotify(microros_task_handle, MICROROS_TASK_UPDATE_REQUIRED, eSetBits);
        });

    auto handle_task_executor_notification =
            [&microros_controller, &task_board_driver](Task* task, Task* precondition)
            {
                if (nullptr != task)
                {
                    // If there's an active task, publish its status
                    MicroROSTypes::TaskStatus task_status(*task, task_board_driver.get_unique_id());
                    task_status.set_waiting_precondition(nullptr != precondition);

                    if (precondition != nullptr)
                    {
                        task_status.set_time(0);
                    }

                    microros_controller.publish_task_status(task_status);
                }
                else
                {
                    // If no active task, publish empty status
                    MicroROSTypes::TaskStatus empty_task_status(task_board_driver.get_unique_id());
                    microros_controller.publish_task_status(empty_task_status);
                }
            };

    // ------------------------
    // Main micro-ROS Update Loop
    // ------------------------
    while (true)
    {
        // Update ROS communication and publish periodic messages
        microros_controller.update();
        publish_taskboard_status.update();
        microros_task_executor.update();

        // Check for notifications
        uint32_t value;

        if (pdPASS == xTaskNotifyWait(pdFALSE, 0xFFFFFFFF, &value, pdMS_TO_TICKS(0)))
        {
            // Check required update
            if (value & MICROROS_TASK_UPDATE_REQUIRED)
            {
                task_executor.execute_operation_on_task(portMAX_DELAY, handle_task_executor_notification);
            }

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
