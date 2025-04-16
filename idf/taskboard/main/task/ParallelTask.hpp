/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <task/Task.hpp>

#include <esp_log.h>

/**
 * @struct ParallelTask
 *
 * @brief Implementation of Task interface that executes steps in parallel,
 * that means that each step can be achieved independently from the others.
 */
struct ParallelTask :
    public Task
{
    /**
     * @brief Constructs a new ParallelTask object
     *
     * @param steps Vector of pointers to TaskStep objects defining the parallel steps
     * @param task_name Name identifier for the task
     */
    ParallelTask(
            const std::vector<const TaskStepBase*>& steps,
            const std::string& task_name = "")
        : Task(steps, task_name, false)
    {
        steps_status_.resize(steps.size(), false);
        steps_score_.resize(steps.size(), -1.0f);
        steps_finish_time_.resize(steps.size(), -1);
    }

    /// Virtual method implementation
    void show_clue(
            ClueScreenController& screen_controller) override
    {
        if (!done())
        {
            for (size_t i = 0; i < steps_.size(); i++)
            {
                if (!steps_status_[i])
                {
                    steps_[i]->show_clue(screen_controller);
                    break;
                }
            }
        }
        else
        {
            screen_controller.print_task_clue("Task Done");
        }
    }

    /// Virtual method implementation
    bool update() override
    {
        bool ret = Task::update();

        for (size_t i = 0; i < steps_.size(); i++)
        {
            if (!steps_status_[i] && steps_[i]->success())
            {
                steps_status_[i] = true;
                steps_score_[i] = steps_[i]->score();
                steps_finish_time_[i] = elapsed_time();
                ret = true;
            }
        }

        return ret;
    }

    /// Virtual method implementation
    bool done() const override
    {
        for (size_t i = 0; i < steps_.size(); i++)
        {
            if (!steps_status_[i])
            {
                return false;
            }
        }

        return true;
    }

    /// Virtual method implementation
    bool step_done(
            size_t step) const override
    {
        return steps_status_[step];
    }

    /// Virtual method implementation
    float step_score(
            size_t step) const override
    {
        return steps_score_[step];
    }

    /// Virtual method implementation
    float final_score() const override
    {
        float final_score = 0;
        uint16_t steps_scored_count = 0;
        for (const float step_score : steps_score_)
        {
            if (step_score >= 0.0)
            {
                final_score += step_score;
                steps_scored_count++;
            }
        }
        if (steps_scored_count > 0)
        {
            final_score /= steps_scored_count;
        }
        return final_score;
    }

    /// Virtual method implementation
    int64_t step_done_time(
            size_t step) const override
    {
        return steps_finish_time_[step];
    }

    /// Virtual method implementation
    void restart() override
    {
        std::fill(steps_status_.begin(), steps_status_.end(), false);
        Task::restart();
    }

protected:

    std::vector<bool> steps_status_;            ///< Completion status for each step
    std::vector<float> steps_score_;        ///< Score for each step
    std::vector<int64_t> steps_finish_time_;    ///< Completion status for each step
};
