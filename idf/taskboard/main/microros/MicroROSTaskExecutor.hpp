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


struct MicroROSTaskExecutor
{
    const char * TAG = "MicroROSTaskExecutor";

    MicroROSTaskExecutor(TaskExecutor & task_executor, MicroROSController & microros_controller, TaskBoardDriver & task_board_driver)
    : task_executor_(task_executor)
    , microros_controller_(microros_controller)
    , task_board_driver_(task_board_driver)
    {
        microros_controller_.set_handle_goal([this](rclc_action_goal_handle_t * goal_handle){
            return this->handle_goal(goal_handle);
        });

        microros_controller_.set_handle_cancel([this](rclc_action_goal_handle_t * goal_handle){
            return this->handle_cancel(goal_handle);
        });
    }

    void update()
    {
        publish_feedback_.update();

        // Handle end of task
        if (nullptr != micro_ros_task_ && micro_ros_task_->done())
        {
            MicroROSTypes::ResultMessage result_message(*micro_ros_task_);
            microros_controller_.publish_goal_result(result_message, micro_ros_task_->goal_handle(), GOAL_STATE_SUCCEEDED);
            delete micro_ros_task_;
            micro_ros_task_ = nullptr;
        }
    }

    void cancel_task()
    {
        if (nullptr != micro_ros_task_)
        {
            ESP_LOGI(TAG, "Cancelled task");

            MicroROSTypes::ResultMessage result_message;
            microros_controller_.publish_goal_result(result_message, micro_ros_task_->goal_handle(), GOAL_STATE_CANCELED);
            delete micro_ros_task_;
            micro_ros_task_ = nullptr;
        }
    }

private:

    void publish_feedback()
    {
        if (nullptr != micro_ros_task_)
        {
            MicroROSTypes::FeedbackMessage feedback(*micro_ros_task_);
            microros_controller_.publish_feedback(feedback, micro_ros_task_->goal_handle());
        }
    }

    rcl_ret_t handle_goal(rclc_action_goal_handle_t * goal_handle)
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

    bool handle_cancel(rclc_action_goal_handle_t * goal_handle)
    {
        if (nullptr != micro_ros_task_ && micro_ros_task_->goal_handle() == goal_handle)
        {
            task_executor_.cancel_task();

            MicroROSTypes::ResultMessage result_message;
            microros_controller_.publish_goal_result(result_message, micro_ros_task_->goal_handle(), GOAL_STATE_CANCELED);

            delete micro_ros_task_;
            micro_ros_task_ = nullptr;
        }

        return true;
    }

    TimedOperation publish_feedback_ = {250, [this](){
        this->publish_feedback();
    }};

    MicroROSTask * micro_ros_task_ = nullptr;

    TaskExecutor & task_executor_;
    MicroROSController & microros_controller_;
    TaskBoardDriver & task_board_driver_;
};