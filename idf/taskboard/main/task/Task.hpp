/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <task/TaskStep.hpp>

#include <esp_timer.h>
#include <esp_log.h>

#include <string>
#include <cstdint>

struct Task
{
    Task(const std::vector<const TaskStep*> & steps, const std::string & task_name = "", bool first_task_init_timer = true)
    : steps_(steps)
    , init_time_(esp_timer_get_time())
    , task_name_(task_name)
    , first_task_init_timer_(first_task_init_timer)
    {}

    virtual bool done() const = 0;

    virtual bool step_done(size_t step) const = 0;

    virtual std::string get_clue() = 0;

    virtual bool get_analog_clue(float & current_value, float & target_value) const = 0;

    virtual void restart()
    {
        init_time_ = esp_timer_get_time();
        previous_done_state_ = false;
    }

    int64_t elapsed_time() const
    {
        int64_t ret = task_time_ - init_time_;

        if (first_task_init_timer_ && steps_.size() > 0 && !step_done(0))
        {
            ret = 0;
        }

        return ret;
    }

    virtual bool update()
    {
        bool ret = false;

        const bool done_state = done();
        ret = previous_done_state_ != done_state;
        previous_done_state_ = done_state;

        if(!done_state)
        {
            task_time_ = esp_timer_get_time();
        }

        // Do not start timer until first step is done
        if (first_task_init_timer_ && steps_.size() > 0 && !step_done(0))
        {
            init_time_ = task_time_;
        }

        return ret;
    }

    size_t total_steps() const
    {
        return steps_.size();
    }

    const std::string & name() const
    {
        return task_name_;
    }

    const TaskStep & step(size_t step) const
    {
        return *steps_[step];
    }

    bool is_human_task() const
    {
        return is_human_task_;
    }

    void set_human_task(bool is_human_task)
    {
        is_human_task_ = is_human_task;
    }

protected:
    const std::vector<const TaskStep*> & steps_;

    void set_task_name(const std::string & task_name)
    {
        task_name_ = task_name;
    }

private:
    int64_t init_time_ = 0;
    int64_t task_time_;
    bool previous_done_state_ = false;
    std::string task_name_;
    bool first_task_init_timer_;
    bool is_human_task_ = false;
};