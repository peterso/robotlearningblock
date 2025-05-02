/**
 * Robothon Task Board Firmware
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
            const std::vector<const TaskStepBase*>& steps,
            const std::string& task_name = "")
        : ParallelTask(steps, task_name)
    {
        steps_status_.resize(steps.size(), false);
    }

    /// Virtual method implementation
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

                if (!state)
                {
                    steps_score_[i] = -1.0f;
                    steps_finish_time_[i] = -1;
                }
                else
                {
                    steps_score_[i] = steps_[i]->score();
                    steps_finish_time_[i] = elapsed_time();
                }
            }
        }

        return ret;
    }

    /// Virtual method implementation
    int64_t total_task_time() const override
    {
        return elapsed_time();
    }

};
