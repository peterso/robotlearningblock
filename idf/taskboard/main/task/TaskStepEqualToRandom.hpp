/**
 * Roboton Task Board Firmware
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
            const SensorReader& sensor,
            const float tolerance = 0.00)
        : TaskStepEqual(sensor, SensorMeasurement(true), tolerance)
        , random_expected_value_(false)
    {
        TaskStep::type_ = Type::EQUAL_TO_RANDOM;

        // Generate initial random value between 0 and 1
        random_expected_value_ = SensorMeasurement((float) esp_random() / (float) UINT32_MAX);
    }

    /// Virtual method implementation
    bool success() const override
    {
        const bool ret = SensorMeasurement::equal(sensor_.read(), random_expected_value_, tolerance_);

        if (ret)
        {
            // Generate new random value between 0 and 1
            random_expected_value_ = SensorMeasurement((float) esp_random() / (float) UINT32_MAX);
        }

        return ret;
    }

private:

    mutable SensorMeasurement random_expected_value_;    ///< Current random target value, updated when matched
};
