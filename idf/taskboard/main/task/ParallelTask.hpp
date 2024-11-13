/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <task/Task.hpp>

#include <esp_log.h>

struct ParallelTask : public Task
{
    ParallelTask(const std::vector<const TaskStep*> & steps, const std::string & task_name = "")
    : Task(steps, task_name, false)
    {
        steps_status_.resize(steps.size(), false);
    }

    std::string get_clue() override
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

    bool get_analog_clue(float & current_value, float & target_value) const override
    {
        bool ret = false;

        for (size_t i = 0; i < steps_.size(); i++)
        {
            if (!steps_status_[i])
            {
                const SensorMeasurement & sensor_value = steps_[i]->sensor().read();
                if (sensor_value.get_type() == SensorMeasurement::Type::ANALOG)
                {
                    current_value = sensor_value.get_analog();
                    target_value = steps_[i]->expected_value().get_analog();
                    ret = true;
                }
                break;
            }
        }

        return ret;
    }

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

    bool step_done(size_t step) const override
    {
        return steps_status_[step];
    }

    void restart() override
    {
        std::fill(steps_status_.begin(), steps_status_.end(), false);
        Task::restart();
    }

protected:
    std::vector<bool> steps_status_;
};