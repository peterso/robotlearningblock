/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <sensor/SensorMeasurement.hpp>

#include <esp_log.h>

#include <functional>
#include <string>

struct SensorUpdater
{
    // Using read lambda function to read the sensor value
    using ReadFunction = std::function<SensorMeasurement()>;

    SensorUpdater(ReadFunction read_function)
        : read_function_(read_function)
        , last_value_(0.0f)
    {}

    virtual void update()
    {
        last_value_ = read_function_();
    }

protected:
    ReadFunction read_function_;
    SensorMeasurement last_value_;
};

struct SensorReader
{
    virtual const std::string & name() const = 0;
    virtual const SensorMeasurement read() const = 0;
};

struct Sensor : public SensorUpdater, public SensorReader
{
    Sensor(const std::string & name, ReadFunction read_function)
        : SensorUpdater(read_function)
        , name_(name)
    {}

    virtual ~Sensor() = default;

    const std::string & name() const override
    {
        return name_;
    }

    virtual const SensorMeasurement read() const override
    {
        return last_value_;
    }

protected:
    std::string name_;
};