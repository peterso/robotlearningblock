/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <task/TaskStep.hpp>

#include <esp_log.h>

struct TaskStepGreaterEqualThan : public TaskStep
{
    TaskStepGreaterEqualThan(const SensorReader & sensor, const SensorMeasurement & expected_value)
    : TaskStep(sensor)
    , expected_value_(expected_value)
    {
        TaskStep::type_ = Type::GREATER_OR_EQUAL;
    }

    bool success() const override
    {
        return SensorMeasurement::greater_or_equal(sensor_.read(), expected_value_);
    }

    SensorMeasurement expected_value() const override
    {
        return expected_value_;
    }

private:
    const SensorMeasurement expected_value_;
};

