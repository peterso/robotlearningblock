/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <hal/ScreenController.hpp>
#include <hal/NonVolatileStorage.hpp>

#include <esp_log.h>

struct TaskExecutor
{
    const char * TAG = "TaskExecutor";

    static constexpr int64_t ANALOG_UPDATE_INTERVAL = 5e4; // 100 ms

    using FinishTaskCallback = std::function<void(const Task &)>;

    TaskExecutor(ScreenController & screen_controller, NonVolatileStorage & non_volatile_storage)
    : screen_controller_(screen_controller)
    , non_volatile_storage_(non_volatile_storage)
    , last_analog_update_(esp_timer_get_time())
    {
    }

    void update()
    {
        // Handle precondition first
        if (precondition_ != nullptr && current_task_ != nullptr)
        {
            // If update returns true, the task has changed, update the screen
            if(precondition_->update() || force_screen_update_)
            {
                screen_controller_.print_task_status(
                    "Configure " + current_task_->name(),
                    *precondition_,
                    TFT_BLACK,
                    TFT_ORANGE);

                force_screen_update_ = false;

                // Print clue
                screen_controller_.print_task_clue(precondition_->get_clue_string());
            }

            // Update analog clue
            update_analog_clue(*precondition_);

            // Get rid of precondition if done
            if(precondition_->done())
            {
                precondition_ = nullptr;
                force_screen_update_ = true;
                ESP_LOGI(TAG, "Precondition done, starting task %s", current_task_->name().c_str());
            }
        }

        // If there is no precondition, handle current task
        if (precondition_ == nullptr && current_task_ != nullptr)
        {
            // If update returns true, the task has changed, update the screen
            if(current_task_->update() || force_screen_update_)
            {
                screen_controller_.print_task_status(
                    current_task_->name(),
                    *current_task_,
                    TFT_WHITE,
                    current_task_->is_human_task() ? TFT_DARKGREY : TFT_BLACK);
                force_screen_update_ = false;

                // Print clue
                screen_controller_.print_task_clue(current_task_->get_clue_string());
            }

            // Update analog clue
            update_analog_clue(*current_task_);

            // Print time
            screen_controller_.print_task_status_time(current_task_->elapsed_time() / 1e6);

            // Check if task is done
            if(current_task_->done())
            {
                // Store register
                non_volatile_storage_.add_new_register(*current_task_, current_task_->is_human_task());

                // Call finish task callbacks
                for(auto & callback : finish_task_callbacks_)
                {
                    callback(*current_task_);
                }

                current_task_ = nullptr;
            }
        }
    }

    bool run_task(Task & task)
    {
        ESP_LOGI(TAG, "Running task: %s", task.name().c_str());
        return run_task_inner(task, nullptr);
    }

    bool run_task(Task & task, Task & precondition)
    {
        ESP_LOGI(TAG, "Running task: %s with precondition", task.name().c_str());
        return run_task_inner(task, &precondition);
    }

    void cancel_task()
    {
        current_task_ = nullptr;
        precondition_ = nullptr;
        force_screen_update_ = true;

    }

    bool running() const
    {
        return current_task_ != nullptr;
    }

    const Task * current_task() const
    {
        return current_task_;
    }

    const Task * current_precondition() const
    {
        return precondition_;
    }

    void add_finish_task_callback(FinishTaskCallback callback)
    {
        finish_task_callbacks_.push_back(callback);
    }
private:

    void update_analog_clue(Task & task)
    {
        // Check if analog clue is available
        if(last_analog_update_ + ANALOG_UPDATE_INTERVAL < esp_timer_get_time())
        {
            SensorMeasurement current_measurement(false);
            SensorMeasurement target_measurement(false);
            if(task.get_clue(current_measurement, target_measurement) &&
                current_measurement.get_type() == SensorMeasurement::Type::ANALOG &&
                target_measurement.get_type() == SensorMeasurement::Type::ANALOG)
            {
                screen_controller_.print_task_clue_analog(current_measurement.get_analog(), target_measurement.get_analog());
            }
            last_analog_update_ = esp_timer_get_time();
        }
    }

    bool run_task_inner(Task & task, Task * precondition)
    {
        bool ret = false;
        if (current_task_ == nullptr && !task.done())
        {
            current_task_ = &task;
            current_task_->restart();

            precondition_ = precondition;

            if (precondition_ != nullptr)
            {
                precondition_->restart();
                precondition_->update();

                // Quick cancel precondition if already done
                if (precondition_->done())
                {
                    ESP_LOGI(TAG, "Precondition already done, cancelling");
                    precondition_ = nullptr;
                }
            }

            force_screen_update_ = true;
            ret = true;
        }
        return ret;
    }

    ScreenController & screen_controller_;
    NonVolatileStorage & non_volatile_storage_;

    Task * current_task_ = nullptr;
    Task * precondition_ = nullptr;
    bool force_screen_update_ = false;
    int64_t last_analog_update_ = 0;

    std::vector<FinishTaskCallback> finish_task_callbacks_;
};