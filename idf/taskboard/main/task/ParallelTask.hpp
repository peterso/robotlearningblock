/**
 * Roboton Task Board Firmware
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
struct ParallelTask : public Task
{
    /**
     * @brief Constructs a new ParallelTask object
     *
     * @param steps Vector of pointers to TaskStep objects defining the parallel steps
     * @param task_name Name identifier for the task
     */
    ParallelTask(const std::vector<const TaskStep*> & steps, const std::string & task_name = "")
    : Task(steps, task_name, false)
    {
        steps_status_.resize(steps.size(), false);
    }

    /**
     * @brief Gets a text hint for the next incomplete step
     *
     * @return string
     */
    std::string get_clue_string() override
    {
        std::string clue = "";

        if(done())
        {
            clue = "Task Done";
        }
        else
        {
            // Get first step that is not done
            clue = "Waiting sensor ";

            for (size_t i = 0; i < steps_.size(); i++)
            {
                if (!steps_status_[i])
                {
                    clue += steps_[i]->sensor().name();
                    break;
                }
            }
        }

        return clue;
    }

    /**
     * @brief Gets feedback values for the first incomplete step
     *
     * @param[out] current_value Current sensor measurement
     * @param[out] target_value Expected sensor measurement
     *
     * @return true if values were retrieved, false otherwise
     */
    bool get_clue(SensorMeasurement & current_value, SensorMeasurement & target_value) const override
    {
        bool ret = false;

        for (size_t i = 0; i < steps_.size(); i++)
        {
            if (!steps_status_[i])
            {
                const SensorMeasurement & sensor_value = steps_[i]->sensor().read();
                current_value = sensor_value;
                target_value = steps_[i]->expected_value();
                ret = true;
                break;
            }
        }

        return ret;
    }

    /**
     * @brief Updates completion status of all steps
     *
     * @return true if any step's status changed, false otherwise
     */
    bool update() override
    {
        bool ret = Task::update();

        for (size_t i = 0; i < steps_.size(); i++)
        {
            if (!steps_status_[i] && steps_[i]->success())
            {
                steps_status_[i] = true;
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
        for (size_t i = 0; i < steps_.size(); i++)
        {
            if (!steps_status_[i])
            {
                return false;
            }
        }

        return true;
    }

    /**
     * @brief Checks if a specific step is completed
     *
     * @param step Index of the step to check
     *
     * @return true if specified step is done, false otherwise
     */
    bool step_done(size_t step) const override
    {
        return steps_status_[step];
    }

    /**
     * @brief Resets all steps to incomplete status
     */
    void restart() override
    {
        std::fill(steps_status_.begin(), steps_status_.end(), false);
        Task::restart();
    }

protected:

    std::vector<bool> steps_status_;    ///< Completion status for each step
};