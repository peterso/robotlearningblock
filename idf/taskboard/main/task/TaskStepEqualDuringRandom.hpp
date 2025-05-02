/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <task/TaskStepEqual.hpp>

#include <esp_log.h>
#include <esp_random.h>

/**
 * @struct TaskStepEqualDuringRandom
 *
 * @brief Implementation of TaskStep that checks for equality maintained during a random amount of time
 */
struct TaskStepEqualDuringRandom :
    public TaskStepEqual
{
    /**
     * @brief Constructs a new TaskStepEqualDuringRandom object
     *
     * @param sensor Reference to the sensor to monitor
     * @param expected_value Target value that sensor should match
     * @param tolerance Allowable deviation from expected value (default: 0.00)
     * @param min_wait_time_ms Minimum wait time in milliseconds
     * @param max_wait_time_ms Maximum wait time in milliseconds
     */
    TaskStepEqualDuringRandom(
            SensorReader& sensor,
            const SensorMeasurement& expected_value,
            const float tolerance = 0.00,
            const int64_t min_wait_time_ms = 2000L,
            const int64_t max_wait_time_ms = 10000L)
        : TaskStepEqual(sensor, expected_value, tolerance)
        , expected_value_(expected_value)
        , tolerance_(tolerance)
        , minimum_waiting_time_us_(min_wait_time_ms * 1000L)
        , waiting_time_range_us_((max_wait_time_ms - min_wait_time_ms) * 1000L)
    {
        TaskStep::type_ = Type::EQUAL_DURING_RANDOM;
    }

    /// Virtual method implementation
    void initialize_step() const override
    {
        initial_time_ = esp_timer_get_time();
        random_waiting_time_us_ = minimum_waiting_time_us_ + waiting_time_range_us_ * esp_random() / UINT32_MAX;
    }

    /// Virtual method implementation
    bool success() const override
    {
        if (SensorMeasurement::equal(sensor_.read(), expected_value_, tolerance_))
        {
            return (esp_timer_get_time() - initial_time_) >= random_waiting_time_us_;
        }
        else
        {
            initial_time_ = esp_timer_get_time();
        }
        return false;
    }

    /// Virtual method implementation
    float score() const override
    {
        return -1.0f;
    }

    /// Virtual method implementation
    bool is_time_counted() const override
    {
        return false;
    }

private:

    /// Virtual method implementation
    void show_clue_implementation(
            ClueScreenController& screen_controller) const override
    {
        if (!success())
        {
            screen_controller.print_task_clue(sensor_.name() + " != " + expected_value_.to_string() + 
                                              " (during " + std::to_string(random_waiting_time_us_ / 1000) + " ms)");

            const auto sensor_value = sensor_.read();

            if (sensor_value.get_type() == SensorMeasurement::Type::ANALOG)
            {
                screen_controller.print_task_clue_analog(sensor_value.get_analog(), expected_value_.get_analog());
            }
        }
        else
        {
            reset_clue();
        }
    }

    const SensorMeasurement expected_value_;         ///< Expected value
    const float tolerance_;                           ///< Tolerance for equality check
    const int64_t minimum_waiting_time_us_;          ///< Minimum wait time in microseconds
    const int64_t waiting_time_range_us_;          ///< Waiting time range in microseconds
    mutable int64_t initial_time_ = -1L;             ///< Initial time when the step started
    mutable int64_t random_waiting_time_us_ = 0L;    ///< Current random target value
};
