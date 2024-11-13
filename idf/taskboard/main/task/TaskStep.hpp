/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>

#include <esp_log.h>

struct TaskStep
{
    enum class Type
    {
        EQUAL,
        EQUAL_TO_RANDOM,
        GREATER_OR_EQUAL,
        UNKNOWN
    };

    TaskStep(const SensorReader & sensor)
    : sensor_(sensor)
    {}

    virtual ~TaskStep() = default;

    virtual bool success() const = 0;
    virtual SensorMeasurement expected_value () const = 0;

    const SensorReader & sensor() const
    {
        return sensor_;
    }

    const Type & type() const
    {
        return type_;
    }

protected:
    const SensorReader & sensor_;
    Type type_ = Type::UNKNOWN;
};
