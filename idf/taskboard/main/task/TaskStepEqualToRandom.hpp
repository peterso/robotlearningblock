/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <task/TaskStepEqual.hpp>

#include <esp_log.h>
#include <esp_random.h>

/**
 * @struct TaskStepEqualToRandom
 *
 * @brief Implementation of TaskStep that checks for equality against a dynamically generated random value
 */
struct TaskStepEqualToRandom :
    public TaskStepEqual
{
    /**
     * @brief Constructs a new TaskStepEqualToRandom object
     *
     * @param sensor Reference to the sensor to monitor
     * @param tolerance Allowable deviation from random value (default: 0.00)
     */
    TaskStepEqualToRandom(
            SensorReader& sensor,
            const float tolerance = 0.00)
        : TaskStepEqual(sensor, SensorMeasurement(true), tolerance)
        , random_expected_value_(false)
    {
        TaskStep::type_ = Type::EQUAL_TO_RANDOM;

        // Generate initial random value between 0 and 1
        random_expected_value_ = SensorMeasurement((float) esp_random() / (float) UINT32_MAX);
    }

    /// Virtual method implementation
    void initialize_step() const override
    {
        // Generate initial random value between 0 and 1
        random_expected_value_ = SensorMeasurement((float) esp_random() / (float) UINT32_MAX);
    }

    /// Virtual method implementation
    bool success() const override
    {
        return SensorMeasurement::equal(sensor_.read(), random_expected_value_, tolerance_);
    }

    /// Virtual method implementation
    float score() const override
    {
        return 100.0f;
    }

private:

    /// Virtual method implementation
    void show_clue_implementation(
            ClueScreenController& screen_controller) const override
    {
        if (!success())
        {
            screen_controller.print_task_clue(sensor_.name() + " != " + random_expected_value_.to_string());

            const auto sensor_value = sensor_.read();

            if (sensor_value.get_type() == SensorMeasurement::Type::ANALOG)
            {
                screen_controller.print_task_clue_analog(sensor_value.get_analog(), random_expected_value_.get_analog());
            }
        }
        else
        {
            reset_clue();
        }
    }

    mutable SensorMeasurement random_expected_value_;    ///< Current random target value, updated when matched
};
