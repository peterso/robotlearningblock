/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>
#include <task/TaskStep.hpp>

#include <esp_log.h>

/**
 * @struct TaskStepEqual
 *
 * @brief Implementation of TaskStep that checks for equality between sensor reading and target value
 *
 * @details Performs equality comparison between sensor measurement and expected value,
 *          allowing for an optional tolerance to account for sensor noise or precision limits
 */
struct TaskStepEqual :
    public TaskStep
{
    /**
     * @brief Constructs a new TaskStepEqual object
     *
     * @param sensor Reference to the sensor to monitor
     * @param expected_value Target value that sensor should match
     * @param tolerance Allowable deviation from expected value (default: 0.00)
     */
    TaskStepEqual(
            const SensorReader& sensor,
            const SensorMeasurement& expected_value,
            const float tolerance = 0.00)
        : TaskStep(sensor)
        , expected_value_(expected_value)
        , tolerance_(tolerance)
    {
        TaskStep::type_ = Type::EQUAL;
    }

    /// Virtual method implementation
    bool success() const override
    {
        return SensorMeasurement::equal(sensor_.read(), expected_value_, tolerance_);
    }

    /// Virtual method implementation
    SensorMeasurement expected_value() const override
    {
        return expected_value_;
    }

    /// Virtual method implementation
    void show_clue(
            ClueScreenController& screen_controller) const override
    {
        if (!success())
        {
            screen_controller.print_task_clue(sensor_.name() + " != " + expected_value_.to_string());

            const auto sensor_value = sensor_.read();

            if (sensor_value.get_type() == SensorMeasurement::Type::ANALOG)
            {
                screen_controller.print_task_clue_analog(sensor_value.get_analog(), expected_value_.get_analog());
            }
        }
    }

protected:

    const SensorMeasurement expected_value_;    ///< Target value that sensor should match
    const float tolerance_;                      ///< Allowed deviation from expected value
};
