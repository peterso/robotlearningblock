/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <hal/TaskBoardDriver.hpp>
#include <task/TaskExecutor.hpp>
#include <microros/MicroROSTask.hpp>
#include <microros/MicroROSController.hpp>
#include <microros/MicroROSTypes.hpp>
#include <util/Timing.hpp>

/**
 * @struct MicroROSTaskExecutor
 *
 * @brief Task executor for handling micro-ROS task execution
 */
struct MicroROSTaskExecutor
{
    const char* TAG = "MicroROSTaskExecutor";       ///< Logging tag

    /**
     * @brief Constructs a new MicroROSTaskExecutor object
     *
     * @param task_executor Reference to the task executor
     * @param microros_controller Reference to the micro-ROS controller
     * @param task_board_driver Reference to the task board driver
     */
    MicroROSTaskExecutor(
            TaskExecutor& task_executor,
            MicroROSController& microros_controller,
            TaskBoardDriver& task_board_driver)
        : task_executor_(task_executor)
        , microros_controller_(microros_controller)
        , task_board_driver_(task_board_driver)
    {
        microros_controller_.set_handle_goal([this](rclc_action_goal_handle_t* goal_handle)
                {
                    return this->handle_goal(goal_handle);
                });

        microros_controller_.set_handle_cancel([this](rclc_action_goal_handle_t* goal_handle)
                {
                    return this->handle_cancel(goal_handle);
                });
    }

    /**
     * @brief Updates the task executor
     *
     * @details This function should be called periodically to update the task executor
     */
    void update()
    {
        publish_feedback_.update();

        // Handle end of task
        if (nullptr != micro_ros_task_ && micro_ros_task_->done())
        {
            MicroROSTypes::ResultMessage result_message(*micro_ros_task_);
            microros_controller_.publish_goal_result(result_message,
                    micro_ros_task_->goal_handle(), GOAL_STATE_SUCCEEDED);
            delete micro_ros_task_;
            micro_ros_task_ = nullptr;
        }
    }

    /**
     * @brief Cancels the current task
     */
    void cancel_task()
    {
        if (nullptr != micro_ros_task_)
        {
            ESP_LOGI(TAG, "Cancelled task");

            MicroROSTypes::ResultMessage result_message;
            microros_controller_.publish_goal_result(result_message, micro_ros_task_->goal_handle(),
                    GOAL_STATE_CANCELED);
            delete micro_ros_task_;
            micro_ros_task_ = nullptr;
        }
    }

private:

    /**
     * @brief Publishes feedback on the current task
     */
    void publish_feedback()
    {
        if (nullptr != micro_ros_task_)
        {
            MicroROSTypes::FeedbackMessage feedback(*micro_ros_task_);
            microros_controller_.publish_feedback(feedback, micro_ros_task_->goal_handle());
        }
    }

    /**
     * @brief Handles a new goal request within the main task executor
     *
     * @param goal_handle Pointer to the goal handle
     *
     * @return Return Code
     */
    rcl_ret_t handle_goal(
            rclc_action_goal_handle_t* goal_handle)
    {
        if (task_executor_.running())
        {
            ESP_LOGI(TAG, "Task already running, rejecting goal");

            return RCL_RET_ACTION_GOAL_REJECTED;
        }

        micro_ros_task_ = new MicroROSTask(goal_handle, task_board_driver_);

        if (!micro_ros_task_->is_valid())
        {
            ESP_LOGI(TAG, "Invalid task, rejecting goal");
            delete micro_ros_task_;
            micro_ros_task_ = nullptr;

            return RCL_RET_ACTION_GOAL_REJECTED;
        }

        task_executor_.run_task(*micro_ros_task_);

        return RCL_RET_ACTION_GOAL_ACCEPTED;
    }

    /**
     * @brief Handles a goal cancellation request within the main task executor
     *
     * @param goal_handle Pointer to the goal handle
     *
     * @return True if the goal was successfully cancelled
     */
    bool handle_cancel(
            rclc_action_goal_handle_t* goal_handle)
    {
        if (nullptr != micro_ros_task_ && micro_ros_task_->goal_handle() == goal_handle)
        {
            task_executor_.cancel_task();

            MicroROSTypes::ResultMessage result_message;
            microros_controller_.publish_goal_result(result_message, micro_ros_task_->goal_handle(),
                    GOAL_STATE_CANCELED);

            delete micro_ros_task_;
            micro_ros_task_ = nullptr;
        }

        return true;
    }

    /**
     * @brief Periodic operation for publishing feedback on a running task from ROS 2
     */
    TimedOperation publish_feedback_ = {250, [this]()
                                        {
                                            this->publish_feedback();
                                        }
    };

    MicroROSTask* micro_ros_task_ = nullptr;        ///< Pointer to the current micro-ROS task

    TaskExecutor& task_executor_;                   ///< Reference to the task executor
    MicroROSController& microros_controller_;       ///< Reference to the micro-ROS controller
    TaskBoardDriver& task_board_driver_;            ///< Reference to the task board driver
};
