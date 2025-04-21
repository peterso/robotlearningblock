/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <task/TaskStep.hpp>

#include <esp_log.h>
#include <esp_random.h>

/**
 * @struct TaskStepWaitRandom
 *
 * @brief Implementation of TaskStep that waits for a random amount of time and succeeds automatically
 */
struct TaskStepWaitRandom :
    public TaskStepAuto
{
    /**
     * @brief Constructs a new TaskStepWaitRandom object
     * 
     * @param name Name identifier for the task step
     */
    TaskStepWaitRandom(
            std::string name = "",
            int64_t minimum_waiting_time_ms = 2000L,
            int64_t maximum_waiting_time_ms = 10000L)
        : TaskStepAuto(name),
            minimum_waiting_time_us_(minimum_waiting_time_ms * 1000L),
            waiting_time_range_us_((maximum_waiting_time_ms - minimum_waiting_time_ms) * 1000L)
    {
        TaskStepAuto::type_ = Type::WAIT_RANDOM;

        initialize_step();
    }

    void initialize_step() const
    {
        initial_time_ = -1L;
        // Generate random value between 2 and 10 seconds
        random_waiting_time_us_ = minimum_waiting_time_us_ + waiting_time_range_us_ * esp_random() / UINT32_MAX;
    }

    /// Virtual method implementation
    bool success() const override
    {
        if (initial_time_ == -1)
        {
            initial_time_ = esp_timer_get_time();
        }

        const bool ret = esp_timer_get_time() - initial_time_ >= random_waiting_time_us_;

        return ret;
    }

    /// Virtual method implementation
    float score() const override
    {
        initialize_step();
        // Score is irrelevant
        return -1.0f;
    }


private:

    /// Virtual method implementation
    void show_clue_implementation(
            ClueScreenController& screen_controller) const override
    {
        if (!success())
        {
            screen_controller.print_task_clue("Waiting time [ms]: " + std::to_string(random_waiting_time_us_ / 1000));
        }
        else
        {
            reset_clue();
        }
    }

    const int64_t minimum_waiting_time_us_ = 2000000L;     ///< Minimum waiting time in microseconds
    const int64_t waiting_time_range_us_   = 8000000L;     ///< Waiting time range in microseconds
    mutable int64_t initial_time_ = -1L;             ///< Initial time when the step started
    mutable int64_t random_waiting_time_us_ = 0L;    ///< Current random target value
};
