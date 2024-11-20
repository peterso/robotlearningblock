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
    MicroROSTaskExecutor microros_task_executor(task_executor, microros_controller, task_board_driver, main_task_handle);

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

    // Configure periodic task status publisher
    // Publishes current task state and precondition status to ROS
    TimedOperation publish_task_status(TASK_STATUS_PUBLISHER_PERIOD_MS,
            [&microros_controller, &task_executor, &task_board_driver]()
            {
                const Task* current_task = task_executor.current_task();

                if (nullptr != current_task)
                {
                    // If there's an active task, publish its status
                    MicroROSTypes::TaskStatus task_status(*current_task, task_board_driver.get_unique_id());
                    task_status.set_waiting_precondition(nullptr != task_executor.current_precondition());
                    if(task_executor.current_precondition() != nullptr)
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
            });

    // ------------------------
    // Task Completion Callback
    // ------------------------

    // Register callback for task completion events
    // TODO(pgarrido): This may be called from other thread/core so it shall be protected
    task_executor.add_finish_task_callback([&microros_controller, &task_board_driver](const Task& task)
            {
                // Publish final task status when task completes
                MicroROSTypes::TaskStatus task_status(task, task_board_driver.get_unique_id());
                task_status.set_waiting_precondition(false);
                microros_controller.publish_task_status(task_status);
            });

    // ------------------------
    // Main micro-ROS Update Loop
    // ------------------------
    while (true)
    {
        // Update ROS communication and publish periodic messages
        microros_controller.update();
        publish_taskboard_status.update();
        publish_task_status.update();
        microros_task_executor.update();

        // Check for manual task cancellation from Core 0
        uint32_t value;

        if (pdPASS == xTaskNotifyWait(pdFALSE, 0xFFFFFFFF, &value, pdMS_TO_TICKS(0)))
        {
            if (value == MANUALLY_CANCELLED_TASK)
            {
                microros_task_executor.cancel_task();
            }
        }

        // Main loop delay
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
