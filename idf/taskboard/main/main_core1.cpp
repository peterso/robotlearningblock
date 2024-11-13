/**
 * Roboton Task Board Firmware
 */

#include <microros/MicroROSTaskExecutor.hpp>
#include <microros/MicroROS.hpp>
#include <microros/MicroROSTypes.hpp>
#include <microros/MicroROSTask.hpp>
#include <util/Timing.hpp>

void microros_main(void *arg)
{
    MicroROSMainArgs * args = (MicroROSMainArgs *)arg;
    MicroROSController & microros_controller = args->microros_controller;
    TaskBoardDriver & task_board_driver = args->task_board_driver;
    TaskExecutor & task_executor = args->task_executor;

    MicroROSTaskExecutor microros_task_executor(task_executor, microros_controller, task_board_driver);

    TimedOperation publish_board_status(250, [&microros_controller, &task_board_driver](){
        MicroROSTypes::TaskBoardStatus status(task_board_driver);
        microros_controller.publish_board_status(status);
    });

    TimedOperation publish_task_status(500, [&microros_controller, &task_executor, &task_board_driver](){
        const Task * current_task = task_executor.current_task();
        if (nullptr != current_task)
        {
            MicroROSTypes::TaskStatus task_status(*current_task, task_board_driver.get_unique_id());
            task_status.set_waiting_precondition(nullptr != task_executor.current_precondition());
            microros_controller.publish_task_status(task_status);
        }
        else
        {
            MicroROSTypes::TaskStatus empty_task_status(task_board_driver.get_unique_id());
            microros_controller.publish_task_status(empty_task_status);
        }
    });

    // TODO(pgarrido): This may be called from other thread/core so it shall be protected
    task_executor.add_finish_task_callback([&microros_controller, &task_board_driver](const Task & task){
        MicroROSTypes::TaskStatus task_status(task, task_board_driver.get_unique_id());
        task_status.set_waiting_precondition(false);
        microros_controller.publish_task_status(task_status);
    });

    while(true)
    {
        microros_controller.update();
        publish_board_status.update();
        publish_task_status.update();
        microros_task_executor.update();

        // Handle manually cancelled task
        uint32_t value;
        if (pdPASS == xTaskNotifyWait(pdFALSE, 0xFFFFFFFF, &value, pdMS_TO_TICKS(0)))
        {
            if (value == MANUALLY_CANCELLED_TASK)
            {
                microros_task_executor.cancel_task();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
