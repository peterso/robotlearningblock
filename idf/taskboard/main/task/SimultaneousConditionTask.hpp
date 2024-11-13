/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <task/ParallelTask.hpp>

#include <esp_log.h>

/**
 * @struct SimultaneousConditionTask
 *
 * @brief Implementation of Task interface where all conditions must be met simultaneously
 *
 * @details Unlike ParallelTask where steps can be completed in any order and remain completed,
 *          this task requires all conditions to be satisfied at the same time. If any condition
 *          becomes unsatisfied, that step becomes incomplete again.
 */
struct SimultaneousConditionTask :
    public ParallelTask
{
    /**
     * @brief Constructs a new SimultaneousConditionTask object
     *
     * @param steps Vector of pointers to TaskStep objects defining the simultaneous conditions
     * @param task_name Name identifier for the task
     */
    SimultaneousConditionTask(
            const std::vector<const TaskStep*>& steps,
            const std::string& task_name = "")
        : ParallelTask(steps, task_name)
    {
        steps_status_.resize(steps.size(), false);
    }

    /**
     * @brief Updates all step conditions and their completion states
     *
     * @details Checks all steps' conditions simultaneously. If any previously satisfied
     *          condition becomes unsatisfied, that step's status is reset to incomplete.
     *
     * @return true if any step's status changed, false otherwise
     */
    bool update() override
    {
        bool ret = Task::update();

        for (size_t i = 0; i < steps_.size(); i++)
        {
            bool state = steps_[i]->success();

            if (state != steps_status_[i])
            {
                steps_status_[i] = state;
                ret = true;
            }
        }

        return ret;
    }

};
