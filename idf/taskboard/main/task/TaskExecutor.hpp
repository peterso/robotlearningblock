/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <hal/ScreenController.hpp>
#include <hal/NonVolatileStorage.hpp>
#include <util/Timing.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <esp_log.h>

/**
 * @struct TaskExecutor
 *
 * @brief Class for managing task execution and screen output
 *
 * @details This class is prune to be called from multiple execution contexts, so it should be thread-safe
 */
struct TaskExecutor
{
    const char* TAG = "TaskExecutor";  ///< Logging tag

    //! Callback task updates
    using TaskExecutorCallback = std::function<void (const Task*, const Task*)>;

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
        mutex_ = xSemaphoreCreateMutex();
    }

    /**
     * @brief Updates the state of the task executor
     *
     * @details Should be called periodically to refresh sensor readings
     */
    void update()
    {
        // Lock mutex for thread safety
        xSemaphoreTake(mutex_, portMAX_DELAY);

        // Notify periodically the task callbacks, even if there is no task running
        for (auto& callback : task_callbacks_)
        {
            if (callback.first.triggered())
            {
                callback.second(current_task_, precondition_);
            }
        }

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
            }

            // Print clue
            precondition_->show_clue(screen_controller_);

            // Get rid of precondition if done
            if (precondition_->done())
            {
                precondition_ = nullptr;
                force_screen_update_ = true;
                ESP_LOGI(TAG, "Precondition done, starting task %s", current_task_->name().c_str());

                // Start task
                current_task_->restart();
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

                blinkLED();
            }

            // Print clue
            current_task_->show_clue(screen_controller_);

            // Print time
            screen_controller_.print_task_status_time(current_task_->elapsed_time() / 1e6);

            // Check if task is done
            if (current_task_->done())
            {
                // Store register
                non_volatile_storage_.add_new_register(*current_task_, current_task_->is_human_task());

                // Call finish task callbacks
                for (auto& callback : task_callbacks_)
                {
                    callback.second(current_task_, precondition_);
                }

                current_task_ = nullptr;
            }
        }

        // Unlock mutex
        xSemaphoreGive(mutex_);
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
        // Lock mutex for thread safety
        xSemaphoreTake(mutex_, portMAX_DELAY);

        ESP_LOGI(TAG, "Running task: %s", task.name().c_str());

        const bool ret = run_task_inner(task, nullptr);

        // Unlock mutex
        xSemaphoreGive(mutex_);

        return ret;
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
        // Lock mutex for thread safety
        xSemaphoreTake(mutex_, portMAX_DELAY);

        ESP_LOGI(TAG, "Running task: %s with precondition", task.name().c_str());

        const bool ret = run_task_inner(task, &precondition);

        // Unlock mutex
        xSemaphoreGive(mutex_);

        return ret;
    }

    /**
     * @brief Cancels the current task
     */
    void cancel_task()
    {
        // Lock mutex for thread safety
        xSemaphoreTake(mutex_, portMAX_DELAY);

        ESP_LOGI(TAG, "Cancelling task");

        current_task_ = nullptr;
        precondition_ = nullptr;
        force_screen_update_ = true;

        // Unlock mutex
        xSemaphoreGive(mutex_);
    }

    /**
     * @brief Checks if a task is currently running
     *
     * @return true if a task is running, false otherwise
     */
    bool running() const
    {
        // Lock mutex for thread safety
        xSemaphoreTake(mutex_, portMAX_DELAY);

        const bool ret = current_task_ != nullptr;

        // Unlock mutex
        xSemaphoreGive(mutex_);

        return ret;
    }

    /**
     * @brief Adds a callback for task execution events
     *
     * @param period_ms Period in milliseconds to call the callback
     * @param callback Callback function
     */
    void add_callback(
            const uint32_t& period_ms,
            TaskExecutorCallback callback)
    {
        // Lock mutex for thread safety
        xSemaphoreTake(mutex_, portMAX_DELAY);

        task_callbacks_.push_back(std::make_pair(period_ms, callback));

        // Unlock mutex
        xSemaphoreGive(mutex_);
    }

    void execute_operation_on_task(
            const uint32_t& timeout_ticks,
            std::function<void(Task*, Task*)> operation)
    {
        if (xSemaphoreTake(mutex_, timeout_ticks) == pdTRUE)
        {
            operation(current_task_, precondition_);
            xSemaphoreGive(mutex_);
        }
    }

private:

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

        if (current_task_ == nullptr)
        {
            // Ensure task is restarted
            task.restart();

            if (!task.done())
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
        }

        return ret;
    }

    void blinkLED() {
        // Turn LED on
        M5.Power.setLed(255);

        // Start the timer to turn LED off
        if (timer_led_ != nullptr) {
            xTimerStart(timer_led_, 0);
        }
    }

    ScreenController& screen_controller_;                       ///< Reference to screen controller
    NonVolatileStorage& non_volatile_storage_;                  ///< Reference to non-volatile storage

    Task* current_task_ = nullptr;                              ///< Pointer to the current task
    Task* precondition_ = nullptr;                              ///< Pointer to the current precondition task
    bool force_screen_update_ = false;                          ///< Flag to force screen update
    int64_t last_analog_update_ = 0;                            ///< Last analog update time

    SemaphoreHandle_t mutex_;                                   ///< Mutex for thread safety

    TimerHandle_t timer_led_ = xTimerCreate(                    ///< Timer to turn LED off
        "LedOff",              // Timer name
        pdMS_TO_TICKS(500),    // Delay in ms
        pdFALSE,               // Disabled auto-reload 
        nullptr,
        [](TimerHandle_t xTimer) {  // Callback
            // Turn LED off
            M5.Power.setLed(0);
        }
    );

    std::vector<std::pair<Timer, TaskExecutorCallback>> task_callbacks_;      ///< Vector of task update callbacks
};
