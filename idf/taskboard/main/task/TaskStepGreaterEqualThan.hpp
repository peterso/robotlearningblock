/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <task/TaskStep.hpp>

#include <esp_log.h>

/**
 * @struct TaskStepGreaterEqualThan
 *
 * @brief Implementation of TaskStep that checks if sensor reading is greater than or equal to target value
 *
 * @details Performs comparison to verify if the current sensor measurement meets or exceeds
 *          a specified threshold value
 */
struct TaskStepGreaterEqualThan :
    public TaskStep
{
    /**
     * @brief Constructs a new TaskStepGreaterEqualThan object
     *
     * @param sensor Reference to the sensor to monitor
     * @param expected_value Minimum threshold value that sensor should meet or exceed
     */
    TaskStepGreaterEqualThan(
            SensorReader& sensor,
            const SensorMeasurement& expected_value)
        : TaskStep(sensor)
        , expected_value_(expected_value)
    {
        TaskStep::type_ = Type::GREATER_OR_EQUAL;
    }

    /// Virtual method implementation
    void initialize_step() const override
    {
    }

    /// Virtual method implementation
    bool success() const override
    {
        return SensorMeasurement::greater_or_equal(sensor_.read(), expected_value_);
    }

    /// Virtual method implementation
    float score() const override
    {
        return 100.0f;
    }

    /// Virtual method implementation
    SensorMeasurement expected_value() const override
    {
        return expected_value_;
    }

private:

    /// Virtual method implementation
    void show_clue_implementation(
            ClueScreenController& screen_controller) const override
    {
        if (!success())
        {
            screen_controller.print_task_clue(sensor_.name() + " != " + expected_value_.to_string());
        }
        else
        {
            reset_clue();
        }
    }

    const SensorMeasurement expected_value_;    ///< Minimum threshold value for comparison
};
