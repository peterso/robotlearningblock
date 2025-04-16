/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <task/Task.hpp>

#include <esp_log.h>

/**
 * @struct SequentialTask
 *
 * @brief Implementation of Task interface that executes steps in sequential order
 */
struct SequentialTask :
    public Task
{
    const char* TAG = "SequentialTask";                    ///< Logging tag
    /**
     * @brief Constructs a new SequentialTask object
     *
     * @param steps Vector of pointers to TaskStep objects defining the sequence
     * @param task_name Name identifier for the task
     * @param first_task_init_timer If true, timer starts only after first step completion
     */
    SequentialTask(
            const std::vector<const TaskStepBase*>& steps,
            const std::string& task_name = "",
            bool first_task_init_timer = false)
        : Task(steps, task_name, first_task_init_timer)
    {
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~SequentialTask() = default;

    /// Virtual method implementation
    void show_clue(
            ClueScreenController& screen_controller) override
    {
        if (current_step_ < steps_.size())
        {
            steps_[current_step_]->show_clue(screen_controller);
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

        // Initialize step finish time if not done yet
        if (steps_finish_time_.size() == 0)
        {
            steps_score_.resize(steps_.size(), -1.0f);
            steps_finish_time_.resize(steps_.size(), -1);
            steps_completion_time_.resize(steps_.size(), -1);
            steps_time_sum_ = 0;
        }

        if (current_step_ < steps_.size())
        {
            if (steps_[current_step_]->success())
            {
                steps_score_[current_step_] = steps_[current_step_]->score();
                steps_finish_time_[current_step_] = elapsed_time();
                if (current_step_ == 0)
                {
                    steps_completion_time_[current_step_] = steps_finish_time_[current_step_];
                }
                else
                {
                    steps_completion_time_[current_step_] = steps_finish_time_[current_step_] - steps_finish_time_[current_step_ - 1];
                }

                if (!steps_[current_step_]->is_auto())
                {
                    steps_time_sum_ += steps_completion_time_[current_step_];
                }
                
                current_step_++;

                // Restart status of the next step
                if (current_step_ < steps_.size())
                {
                    steps_[current_step_]->restart_step();
                }

                ret = true;
            }
        }

        return ret;
    }

    /// Virtual method implementation
    bool done() const override
    {
        return current_step_ == steps_.size();
    }

    /// Virtual method implementation
    bool step_done(
            size_t step) const override
    {
        return step < current_step_;
    }

    /// Virtual method implementation
    float step_score(
            size_t step) const override
    {
        if (step >= steps_score_.size())
        {
            return -1;
        }

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
        if (step >= steps_completion_time_.size())
        {
            return -1;
        }

        return steps_completion_time_[step];
    }
    
    /// Virtual method implementation
    int64_t total_task_time() const
    {
        return steps_time_sum_;
    }

    /// Virtual method implementation
    void restart() override
    {
        current_step_ = 0;
        Task::restart();
    }

private:

    size_t current_step_ = 0;                       ///< Index of current step being executed
    std::vector<float> steps_score_;                ///< Score for each step
    std::vector<int64_t> steps_finish_time_;        ///< Completion status for each step
    std::vector<int64_t> steps_completion_time_;    ///< Completion time for each step
    int64_t steps_time_sum_ = 0;                    ///< Calculated task total time
};
