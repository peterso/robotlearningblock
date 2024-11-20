/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <hal/ScreenController.hpp>
#include <hal/NonVolatileStorage.hpp>

#include <esp_log.h>

/**
 * @struct TaskExecutor
 *
 * @brief Class for managing task execution and screen output
 */
struct TaskExecutor
{
    const char* TAG = "TaskExecutor";  ///< Logging tag

    //! Analog update interval in microseconds
    static constexpr int64_t ANALOG_UPDATE_INTERVAL = 5e4; // 100 ms

    //! Callback type for task finish event
    using FinishTaskCallback = std::function<void (const Task&)>;

    /**
     * @brief Constructs a new TaskExecutor object
     *
     * @param screen_controller Reference to screen controller for output
     * @param non_volatile_storage Reference to non-volatile storage for task logging
     */
    TaskExecutor(
            ScreenController& screen_controller,
            NonVolatileStorage& non_volatile_storage)
        : screen_controller_(screen_controller)
        , non_volatile_storage_(non_volatile_storage)
        , last_analog_update_(esp_timer_get_time())
    {
    }

    /**
     * @brief Updates the state of the task executor
     *
     * @details Should be called periodically to refresh sensor readings
     */
    void update()
    {
        // Handle precondition first
        if (precondition_ != nullptr && current_task_ != nullptr)
        {
            // If update returns true, the task has changed, update the screen
            if (precondition_->update() || force_screen_update_)
            {
                screen_controller_.print_task_status(
                    "Conf " + current_task_->name(),
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
            if (precondition_->done())
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
            if (current_task_->update() || force_screen_update_)
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
            if (current_task_->done())
            {
                // Store register
                non_volatile_storage_.add_new_register(*current_task_, current_task_->is_human_task());

                // Call finish task callbacks
                for (auto& callback : finish_task_callbacks_)
                {
                    callback(*current_task_);
                }

                current_task_ = nullptr;
            }
        }
    }

    /**
     * @brief Runs a task
     *
     * @param task Task to run
     *
     * @return true if task was started, false if task is already running or done
     */
    bool run_task(
            Task& task)
    {
        ESP_LOGI(TAG, "Running task: %s", task.name().c_str());

        return run_task_inner(task, nullptr);
    }

    /**
     * @brief Runs a task with a precondition
     *
     * @param task Task to run
     * @param precondition Precondition task
     *
     * @return true if task was started, false if task is already running or done
     */
    bool run_task(
            Task& task,
            Task& precondition)
    {
        ESP_LOGI(TAG, "Running task: %s with precondition", task.name().c_str());

        return run_task_inner(task, &precondition);
    }

    /**
     * @brief Cancels the current task
     */
    void cancel_task()
    {
        ESP_LOGI(TAG, "Cancelling task");

        current_task_ = nullptr;
        precondition_ = nullptr;
        force_screen_update_ = true;

    }

    /**
     * @brief Checks if a task is currently running
     *
     * @return true if a task is running, false otherwise
     */
    bool running() const
    {
        return current_task_ != nullptr;
    }

    /**
     * @brief Gets the current task
     *
     * @return Pointer to the current task, or nullptr if no task is running
     */
    const Task* current_task() const
    {
        return current_task_;
    }

    /**
     * @brief Gets the current precondition task
     *
     * @return Pointer to the current precondition task, or nullptr if no task is running
     */
    const Task* current_precondition() const
    {
        return precondition_;
    }

    /**
     * @brief Adds a callback for task finish events
     *
     * @param callback Callback function to be called when a task finishes
     */
    void add_finish_task_callback(
            FinishTaskCallback callback)
    {
        finish_task_callbacks_.push_back(callback);
    }

private:

    /**
     * @brief Updates the analog clue display
     *
     * @param task Task to get the clue from
     */
    void update_analog_clue(
            Task& task)
    {
        // Check if analog clue is available
        if (last_analog_update_ + ANALOG_UPDATE_INTERVAL < esp_timer_get_time())
        {
            SensorMeasurement current_measurement(false);
            SensorMeasurement target_measurement(false);

            if (task.get_clue(current_measurement, target_measurement) &&
                    current_measurement.get_type() == SensorMeasurement::Type::ANALOG &&
                    target_measurement.get_type() == SensorMeasurement::Type::ANALOG)
            {
                screen_controller_.print_task_clue_analog(
                    current_measurement.get_analog(), target_measurement.get_analog());
            }

            last_analog_update_ = esp_timer_get_time();
        }
    }

    /**
     * @brief Internal function to run a task
     *
     * @param task Task to run
     * @param precondition Precondition task
     *
     * @return true if task was started, false if task is already running or done
     */
    bool run_task_inner(
            Task& task,
            Task* precondition)
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

    ScreenController& screen_controller_;                       ///< Reference to screen controller
    NonVolatileStorage& non_volatile_storage_;                  ///< Reference to non-volatile storage

    Task* current_task_ = nullptr;                              ///< Pointer to the current task
    Task* precondition_ = nullptr;                              ///< Pointer to the current precondition task
    bool force_screen_update_ = false;                          ///< Flag to force screen update
    int64_t last_analog_update_ = 0;                            ///< Last analog update time

    std::vector<FinishTaskCallback> finish_task_callbacks_;     ///< Vector of task finish callbacks
};
