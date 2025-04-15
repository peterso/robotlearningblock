/**
 * Robothon Task Board Firmware
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

#include <robothon_taskboard_msgs/msg/task_status.h>
#include <robothon_taskboard_msgs/action/execute_task.h>

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
        const robothon_taskboard_msgs__action__ExecuteTask_SendGoal_Request* goal_request =
                (const robothon_taskboard_msgs__action__ExecuteTask_SendGoal_Request*) goal_handle->ros_goal_request;

        this->set_human_task(goal_request->goal.human_task);

        this->set_task_name(goal_request->goal.task.name.data);

        const uint32_t num_steps = goal_request->goal.task.steps.size;
        steps_.reserve(num_steps);

        for (size_t i = 0; i < num_steps; i++)
        {
            auto& msg_step = goal_request->goal.task.steps.data[i];

            SensorReader* sensor = task_board_driver.get_sensor_by_name(msg_step.sensor_name.data);

            if (sensor == nullptr)
            {
                ESP_LOGI(TAG, "Sensor %s not found, skipping step", msg_step.sensor_name.data);
                continue;
            }

            SensorMeasurement target = MicroROSTypes::SensorMeasurement::from_microros(msg_step.target);

            TaskStep* step = nullptr;

            switch (msg_step.type)
            {
                case robothon_taskboard_msgs__msg__TaskStep__TASK_STEP_TYPE_EQUAL:
                    step = new TaskStepEqual(*sensor, target, msg_step.tolerance);
                    break;
                case robothon_taskboard_msgs__msg__TaskStep__TASK_STEP_TYPE_EQUAL_RANDOM:
                    step = new TaskStepEqualToRandom(*sensor, msg_step.tolerance);
                    break;
                case robothon_taskboard_msgs__msg__TaskStep__TASK_STEP_TYPE_GREATER_EQUAL:
                    step = new TaskStepGreaterEqualThan(*sensor, target);
                    break;
                default:
                    break;
            }

            if (nullptr != step)
            {
                // Configure clue timeout if required
                if (msg_step.clue_trigger_name.data != nullptr)
                {
                    SensorReader* clue_timeout_sensor = task_board_driver.get_sensor_by_name(
                        msg_step.clue_trigger_name.data);

                    if (clue_timeout_sensor != nullptr)
                    {
                        step->set_clue_timeout(*clue_timeout_sensor, microros_to_usec(msg_step.clue_timeout));
                    }
                }

                steps_.push_back(step);
            }
        }

        if (steps_.size() == 0)
        {
            steps_from_default_task_ = true;

            for (size_t i = 0; i < task_board_driver.get_default_task().total_steps(); i++)
            {
                steps_.push_back(&task_board_driver.get_default_task().step(i));
            }
        }

    }

    /**
     * @brief Virtual destructor
     */
    virtual ~MicroROSTask()
    {
        if (!steps_from_default_task_)
        {
            for (auto step : steps_)
            {
                delete step;
            }
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
    std::vector<const TaskStepBase*> steps_;                ///< List of task steps
    bool steps_from_default_task_ = false;                  ///< Flag to indicate if steps are from default task
};
