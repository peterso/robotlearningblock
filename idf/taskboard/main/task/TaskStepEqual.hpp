/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>
#include <task/TaskStep.hpp>

#include <esp_log.h>


struct TaskStepEqual : public TaskStep
{
    TaskStepEqual(const SensorReader & sensor, const SensorMeasurement & expected_value, const float tolerance = 0.00)
    : TaskStep(sensor)
    , expected_value_(expected_value)
    , tolerance_(tolerance)
    {
        TaskStep::type_ = Type::EQUAL;
    }

    bool success() const override
    {
        return SensorMeasurement::equal(sensor_.read(), expected_value_, tolerance_);
    }

    SensorMeasurement expected_value() const override
    {
        return expected_value_;
    }

protected:
    const SensorMeasurement expected_value_;
    const float tolerance_;
};
