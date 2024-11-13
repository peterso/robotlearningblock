/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <task/TaskStepEqual.hpp>

#include <esp_log.h>
#include <esp_random.h>

struct TaskStepEqualToRandom : public TaskStepEqual
{
    TaskStepEqualToRandom(const SensorReader & sensor, const float tolerance = 0.00)
    : TaskStepEqual(sensor, SensorMeasurement(true), tolerance)
    , random_expected_value_(false)
    {
        TaskStep::type_ = Type::EQUAL_TO_RANDOM;

        // Generate random value
        random_expected_value_ = SensorMeasurement((float) esp_random() / (float) UINT32_MAX);
    }

    bool success() const override
    {
        const bool ret = SensorMeasurement::equal(sensor_.read(), random_expected_value_, tolerance_);

        if (ret)
        {
            // Generate new random value
            random_expected_value_ = SensorMeasurement((float) esp_random() / (float) UINT32_MAX);
        }

        return ret;
    }

private:
    mutable SensorMeasurement random_expected_value_;
};
