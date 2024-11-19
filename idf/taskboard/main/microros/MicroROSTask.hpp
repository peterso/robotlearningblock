/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <microros/MicroROSTypes.hpp>
#include <task/SequentialTask.hpp>
#include <task/TaskStepEqual.hpp>
#include <task/TaskStepEqualToRandom.hpp>
#include <task/TaskStepGreaterEqualThan.hpp>
#include <util/Timing.hpp>
#include <hal/TaskBoardDriver.hpp>

#include <rclc/rclc.h>

#include <esp_log.h>

#include <roboton_taskboard_msgs/msg/task_status.h>
#include <roboton_taskboard_msgs/action/execute_task.h>

/**
 * @struct MicroROSTask
 *
 * @brief Implementation of SequentialTask that is received from a micro-ROS action goal
 */
struct MicroROSTask :
    public SequentialTask
{
    const char* TAG = "MicroROSTask";       ///< Logging tag

    /**
     * @brief Constructs a new MicroROSTask object
     *
     * @param goal_handle Pointer to the micro-ROS action goal handle
     * @param task_board_driver Reference to the task board driver
     */
    MicroROSTask(
            const rclc_action_goal_handle_t* goal_handle,
            TaskBoardDriver& task_board_driver)
        : SequentialTask(steps_, "MicroROS Task")
        , goal_handle_(goal_handle)
    {
        const roboton_taskboard_msgs__action__ExecuteTask_SendGoal_Request* goal_request =
                (const roboton_taskboard_msgs__action__ExecuteTask_SendGoal_Request*) goal_handle->ros_goal_request;

        this->set_human_task(goal_request->goal.human_task);

        this->set_task_name(goal_request->goal.task.name.data);

        const uint32_t num_steps = goal_request->goal.task.steps.size;
        steps_.reserve(num_steps);

        for (size_t i = 0; i < num_steps; i++)
        {
            auto& msg_step = goal_request->goal.task.steps.data[i];

            const SensorReader* sensor = task_board_driver.get_sensor_by_name(msg_step.sensor_name.data);

            if (sensor == nullptr)
            {
                ESP_LOGI(TAG, "Sensor %s not found, skipping step", msg_step.sensor_name.data);
                continue;
            }

            SensorMeasurement target = MicroROSTypes::SensorMeasurement::from_microros(msg_step.target);

            TaskStep* step;

            switch (msg_step.type)
            {
                case roboton_taskboard_msgs__msg__TaskStep__TASK_STEP_TYPE_EQUAL:
                    step = new TaskStepEqual(*sensor, target, msg_step.tolerance);
                    break;
                case roboton_taskboard_msgs__msg__TaskStep__TASK_STEP_TYPE_EQUAL_RANDOM:
                    step = new TaskStepEqualToRandom(*sensor, msg_step.tolerance);
                    break;
                case roboton_taskboard_msgs__msg__TaskStep__TASK_STEP_TYPE_GREATER_EQUAL:
                    step = new TaskStepGreaterEqualThan(*sensor, target);
                    break;
                default:
                    break;
            }

            if (nullptr != step)
            {
                steps_.push_back(step);
            }
        }

    }

    /**
     * @brief Virtual destructor
     */
    virtual ~MicroROSTask()
    {
        for (auto step : steps_)
        {
            delete step;
        }
    }

    /**
     * @brief Gets the micro-ROS action goal handle associated with this task
     *
     * @return Pointer to the goal handle
     */
    const rclc_action_goal_handle_t* goal_handle() const
    {
        return goal_handle_;
    }

    /**
     * @brief Checks if the task is valid
     *
     * @return true if the task has at least one step, false otherwise
     */
    bool is_valid()
    {
        return steps_.size() > 0;
    }

private:

    rclc_action_goal_handle_t const* const goal_handle_;    ///< Pointer to the micro-ROS action goal handle
    std::vector<const TaskStep*> steps_;                    ///< List of task steps
};
