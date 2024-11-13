/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <builtin_interfaces/msg/time.h>

#include <esp_log.h>
#include <esp_timer.h>

inline builtin_interfaces__msg__Time usec_to_microros(const int64_t usec)
{
    builtin_interfaces__msg__Time time;
    time.sec = usec / 1000000;
    time.nanosec = (usec % 1000000) * 1000;
    return time;
}

struct TimedOperation
{
    using Callback = std::function<void()>;

    TimedOperation(const uint32_t period_ms, const Callback& callback)
    : callback_(callback),
      period_ms_(period_ms),
      last_time_(esp_timer_get_time())
    {}

    void update()
    {
        const uint32_t current_time = esp_timer_get_time();
        if(current_time - last_time_ > period_ms_ * 1000)
        {
            callback_();
            last_time_ = current_time;
        }
    }

private:
    Callback callback_;
    const uint32_t period_ms_;
    uint32_t last_time_;
};