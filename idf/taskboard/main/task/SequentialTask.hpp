/**
 * Roboton Task Board Firmware
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
            bool first_task_init_timer = true)
        : Task(steps, task_name, first_task_init_timer)
    {
        steps_finish_time_.resize(steps.size(), -1);
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~SequentialTask() = default;

    /**
     * @brief Gets a text hint for the current step
     *
     * @return string
     */
    std::string get_clue_string() override
    {
        std::string clue = "";

        if (current_step_ < steps_.size())
        {
            clue = "Waiting " + steps_[current_step_]->sensor().name();
        }
        else
        {
            clue = "Task Done";
        }

        return clue;
    }

    /**
     * @brief Gets feedback values for the current step
     *
     * @param[out] current_value Current sensor measurement
     * @param[out] target_value Expected sensor measurement
     *
     * @return true if values were retrieved, false if not
     */
    bool get_clue(
            SensorMeasurement& current_value,
            SensorMeasurement& target_value) const override
    {
        bool ret = false;

        if (current_step_ < steps_.size())
        {
            const SensorMeasurement& sensor_value = steps_[current_step_]->sensor().read();
            current_value = sensor_value;
            target_value = steps_[current_step_]->expected_value();
            ret = true;
        }

        return ret;
    }

    /**
     * @brief Updates completion status and advances to next step if current is complete
     *
     * @return true if current step was completed, false otherwise
     */
    bool update() override
    {
        bool ret = Task::update();

        if (current_step_ < steps_.size())
        {
            if (steps_[current_step_]->success())
            {
                steps_finish_time_[current_step_] = elapsed_time();
                current_step_++;
                ret = true;
            }
        }

        return ret;
    }

    /**
     * @brief Checks if all steps have been completed
     *
     * @return true if all steps are done, false otherwise
     */
    bool done() const override
    {
        return current_step_ == steps_.size();
    }

    /**
     * @brief Checks if a specific step has been completed
     *
     * @param step Index of the step to check
     *
     * @return true if specified step is done (previous to current), false otherwise
     */
    bool step_done(
            size_t step) const override
    {
        return step < current_step_;
    }

    /**
     * @brief Gets step done time
     *
     * @param step Index of the step to check
     *
     * @return Step done time in microseconds, -1 if step is not done
     */
    int64_t step_done_time(
            size_t step) const
    {
        return steps_finish_time_[step];
    }

    /**
     * @brief Resets task to start from first step
     */
    void restart() override
    {
        current_step_ = 0;
        Task::restart();
    }

private:

    size_t current_step_ = 0;                   ///< Index of current step being executed
    std::vector<int64_t> steps_finish_time_;    ///< Completion status for each step
};
