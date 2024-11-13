/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <task/Task.hpp>

#include <esp_log.h>


struct SequentialTask : public Task
{
    SequentialTask(const std::vector<const TaskStep*> & steps, const std::string & task_name = "",  bool first_task_init_timer = true)
    : Task(steps, task_name, first_task_init_timer)
    {}

    virtual ~SequentialTask() = default;

    std::string get_clue() override
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

    bool get_analog_clue(float & current_value, float & target_value) const override
    {
        bool ret = false;
        if (current_step_ < steps_.size())
        {
            const SensorMeasurement & sensor_value = steps_[current_step_]->sensor().read();
            if (sensor_value.get_type() == SensorMeasurement::Type::ANALOG)
            {
                current_value = sensor_value.get_analog();
                target_value = steps_[current_step_]->expected_value().get_analog();
                ret = true;
            }
        }

        return ret;
    }

    bool update() override
    {
        bool ret = Task::update();

        if (current_step_ < steps_.size())
        {
            if (steps_[current_step_]->success())
            {
                current_step_++;
                ret = true;
            }
        }

        return ret;
    }

    bool done() const override
    {
        return current_step_ == steps_.size();
    }

    bool step_done(size_t step) const override
    {
        return step < current_step_;
    }

    void restart() override
    {
        current_step_ = 0;
        Task::restart();
    }

private:
    size_t current_step_ = 0;
};
