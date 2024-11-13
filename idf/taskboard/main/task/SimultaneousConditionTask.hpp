/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <task/ParallelTask.hpp>

#include <esp_log.h>

struct SimultaneousConditionTask : public ParallelTask
{
    SimultaneousConditionTask(const std::vector<const TaskStep*> & steps, const std::string & task_name = "")
    : ParallelTask(steps, task_name)
    {
        steps_status_.resize(steps.size(), false);
    }

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
