/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <task/TaskStep.hpp>
#include <util/UUID.hpp>

#include <esp_timer.h>
#include <esp_log.h>

#include <string>
#include <cstdint>
#include <vector>

/**
 * @struct Task
 * @brief Task interface for representing a sequence of steps in the task board
 */
struct Task
{
    //! @brief TaskUniqueIdentifier type
    using TaskUniqueIdentifier = uint8_t[16];

    //! @brief Task default timeout in seconds
    static constexpr int64_t DEFAULT_TIMEOUT_S = 10 * 60;  // 10 minutes

    /**
     * @brief Constructs a new Task object
     *
     * @param steps Vector of pointers to TaskStep objects defining the sequence of steps
     * @param task_name Name identifier for the task
     * @param first_task_init_timer If true, timer starts only after first step completion
     */
    Task(
            const std::vector<const TaskStep*>& steps,
            const std::string& task_name = "",
            bool first_task_init_timer = false)
        : steps_(steps)
        , init_time_(esp_timer_get_time())
        , task_name_(task_name)
        , first_task_init_timer_(first_task_init_timer)
    {
        uuid_generate(unique_id_);
        unique_id_str_ = uuid_to_string(unique_id_);
    }

    /**
     * @brief Checks if the entire task has been completed
     *
     * @return true if task is done, false otherwise
     */
    virtual bool done() const = 0;

    /**
     * @brief Checks if a specific step has been completed
     *
     * @param step Index of the step to check
     *
     * @return true if the specified step is done, false otherwise
     */
    virtual bool step_done(
            size_t step) const = 0;

    /**
     * @brief Gets step done time
     *
     * @param step Index of the step to check
     *
     * @return Step done time in microseconds, -1 if step is not done
     */
    virtual int64_t step_done_time(
            size_t step) const = 0;

    /**
     * @brief Gets a text hint or instruction for the next step
     *
     * @return string
     */
    virtual std::string get_clue_string() = 0;

    /**
     * @brief Gets feedback for the current expected sensor value
     *
     * @param[out] current_value Current sensor or state value
     * @param[out] target_value Target value to achieve
     *
     * @return true if analog feedback is available, false otherwise
     */
    virtual bool get_clue(
            SensorMeasurement& current_value,
            SensorMeasurement& target_value) const = 0;

    /**
     * @brief Checks if the task has timed out
     *
     * @return true if task has timed out, false otherwise
     */
    bool timeout() const
    {
        return init_time_ + (DEFAULT_TIMEOUT_S * 1000000) < esp_timer_get_time();
    }

    /**
     * @brief Resets the task to its initial state
     */
    virtual void restart()
    {
        init_time_ = esp_timer_get_time();
        previous_done_state_ = false;
    }

    /**
     * @brief Gets the elapsed time since task start
     *
     * @return Elapsed time in microseconds
     */
    int64_t elapsed_time() const
    {
        int64_t ret = task_time_ - init_time_;

        if (first_task_init_timer_ && steps_.size() > 0 && !step_done(0))
        {
            ret = 0;
        }

        return ret;
    }

    /**
     * @brief Updates task state and timing
     *
     * @return true if task completion state changed, false otherwise
     */
    virtual bool update()
    {
        bool ret = false;

        const bool done_state = done();
        ret = previous_done_state_ != done_state;
        previous_done_state_ = done_state;

        if (!done_state)
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

    /**
     * @brief Gets the total number of steps in the task
     *
     * @return Number of steps
     */
    size_t total_steps() const
    {
        return steps_.size();
    }

    /**
     * @brief Gets the task name
     *
     * @return Reference to task name string
     */
    const std::string& name() const
    {
        return task_name_;
    }

    /**
     * @brief Gets a reference to a specific step
     *
     * @param step Index of the step to retrieve
     *
     * @return Const reference to the TaskStep object
     */
    const TaskStep& step(
            size_t step) const
    {
        return *steps_[step];
    }

    /**
     * @brief Checks if this is a human-operated task
     *
     * @return true if task requires human operation, false if robot
     */
    bool is_human_task() const
    {
        return is_human_task_;
    }

    /**
     * @brief Sets whether this is a human-operated task
     *
     * @param is_human_task true for human operation, false for robot
     */
    void set_human_task(
            bool is_human_task)
    {
        is_human_task_ = is_human_task;
    }

    const std::string& unique_id() const
    {
        return unique_id_str_;
    }

protected:

    /**
     * @brief Sets the task name
     * @param task_name New name for the task
     */
    void set_task_name(
            const std::string& task_name)
    {
        task_name_ = task_name;
    }

    const std::vector<const TaskStep*>& steps_;     ///< Sequence of steps in the task

private:

    TaskUniqueIdentifier unique_id_;                ///< Unique identifier for the task
    std::string unique_id_str_;                     ///< Unique identifier as string

    int64_t init_time_ = 0;                         ///< Task start time in microseconds
    int64_t task_time_;                             ///< Current task time in microseconds
    bool previous_done_state_ = false;              ///< Previous task completion state
    std::string task_name_;                         ///< Task identifier
    bool first_task_init_timer_;                    ///< If true, timing starts after first step
    bool is_human_task_ = false;                    ///< Task operation mode
};
