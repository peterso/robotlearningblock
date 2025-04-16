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
    TaskStepWaitRandom(std::string name = "")
        : TaskStepAuto(name)
    {
        TaskStepAuto::type_ = Type::WAIT_RANDOM;

        initializeStep();
    }

    void initializeStep() const
    {
        initial_time_ = -1LL;
        // Generate random value between 2 and 10 seconds
        random_waiting_time_us_ = 2000000LL + 8000000LL * esp_random() / UINT32_MAX;
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
        initializeStep();
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

    mutable int64_t initial_time_ = -1LL;             ///< Initial time when the step started
    mutable int64_t random_waiting_time_us_ = 0LL;    ///< Current random target value
};
