/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <builtin_interfaces/msg/time.h>

#include <rmw_microros/time_sync.h>

#include <esp_log.h>
#include <esp_timer.h>

/**
 * @brief Converts microseconds to ROS Time message
 *
 * @details Converts a microsecond timestamp into a ROS2 builtin_interfaces Time message,
 *          splitting it into seconds and nanoseconds components
 *
 * @return ROS Time message equivalent
 */
inline builtin_interfaces__msg__Time usec_to_microros(
        const int64_t usec)
{
    builtin_interfaces__msg__Time time;
    time.sec = usec / 1000000;
    time.nanosec = (usec % 1000000) * 1000;

    return time;
}

/**
 * @brief Converts a ROS Time message to microseconds
 *
 * @details Converts a ROS2 builtin_interfaces Time message into a microsecond timestamp
 *
 * @param time ROS Time message to convert
 *
 * @return Microsecond timestamp
 */
inline int64_t microros_to_usec(
        const builtin_interfaces__msg__Time& time)
{
    return time.sec * 1000000 + time.nanosec / 1000;
}

/**
 * @brief Get the synchronized time in ROS Time message format

 * @return ROS Time message equivalent
 */
inline builtin_interfaces__msg__Time get_rmw_time()
{
    int64_t rmw_millis = rmw_uros_epoch_millis();
    builtin_interfaces__msg__Time time;
    time.sec = rmw_millis / 1000;
    time.nanosec = (rmw_millis % 1000) * 1000000;

    return time;
}

/**
 * @brief Timer
 *
 * @details Wrapper util for timing operations
 */
struct Timer
{
    /**
     * @brief Constructs a new Timer object
     *
     * @param period_ms Period in milliseconds
     */
    Timer(
            const uint32_t period_ms)
        : period_ms_(period_ms)
        , start_time_(esp_timer_get_time())
    {
    }

    /**
     * @brief Restarts the timer
     */
    void restart()
    {
        start_time_ = esp_timer_get_time();
    }

    /**
     * @brief Check if the timer has elapsed
     *
     * @details If the timer has elapsed, the timer is restarted
     *
     * @return true if the timer has elapsed
     */
    bool triggered()
    {
        bool ret = false;
        const uint32_t current_time = esp_timer_get_time();

        if (current_time - start_time_ > period_ms_ * 1000)
        {
            start_time_ = current_time;
            ret = true;
        }

        return ret;
    }

private:

    uint32_t period_ms_;        ///< Period in milliseconds
    uint32_t start_time_;       ///< Start time in microseconds

};

/**
 * @struct TimedOperation
 *
 * @brief Executes a callback function periodically
 */
struct TimedOperation
{
    /// @brief Function type for periodic callback
    using Callback = std::function<void ()>;

    /**
     * @brief Constructs a new TimedOperation object
     * @param period_ms Period in milliseconds between callback executions
     * @param callback Function to execute periodically
     */
    TimedOperation(
            const uint32_t period_ms,
            const Callback& callback)
        : callback_(callback)
        , period_ms_(period_ms)
        , last_time_(esp_timer_get_time())
    {
    }

    /**
     * @brief Updates the timer and executes callback if period has elapsed
     */
    void update()
    {
        const uint32_t current_time = esp_timer_get_time();

        if (current_time - last_time_ > period_ms_ * 1000)
        {
            callback_();
            last_time_ = current_time;
        }
    }

private:

    Callback callback_;             ///< Function to execute periodically
    const uint32_t period_ms_;      ///< Period between executions in milliseconds
    uint32_t last_time_;            ///< Last execution time in microseconds
};
