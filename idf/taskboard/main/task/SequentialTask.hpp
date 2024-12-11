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
    /**
     * @brief Constructs a new SequentialTask object
     *
     * @param steps Vector of pointers to TaskStep objects defining the sequence
     * @param task_name Name identifier for the task
     * @param first_task_init_timer If true, timer starts only after first step completion
     */
    SequentialTask(
            const std::vector<const TaskStep*>& steps,
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
            steps_finish_time_.resize(steps_.size(), -1);
        }

        if (current_step_ < steps_.size())
        {
            if (steps_[current_step_]->success())
            {
                steps_finish_time_[current_step_] = elapsed_time();
                current_step_++;

                // Restart status of the next step
                if (current_step_ < steps_.size())
                {
                    steps_[current_step_]->sensor().start_read();
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
    int64_t step_done_time(
            size_t step) const override
    {
        if (step >= steps_finish_time_.size())
        {
            return -1;
        }

        return steps_finish_time_[step];
    }

    /// Virtual method implementation
    void restart() override
    {
        current_step_ = 0;
        Task::restart();
    }

private:

    size_t current_step_ = 0;                   ///< Index of current step being executed
    std::vector<int64_t> steps_finish_time_;    ///< Completion status for each step
};
