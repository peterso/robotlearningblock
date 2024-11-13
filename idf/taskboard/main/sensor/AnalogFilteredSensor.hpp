/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>

#include <esp_log.h>

#include <deque>

struct AnalogFilteredSensor : public Sensor
{
    AnalogFilteredSensor(const std::string & name, uint32_t filter_size, ReadFunction read_function)
        : Sensor(name, read_function)
        , filter_size_(filter_size)
    {
        if (filter_size_ > 1)
        {
            analog_filter_.resize(filter_size);
        }
    }

    virtual ~AnalogFilteredSensor() = default;

    void update() override
    {
        Sensor::update();
        SensorMeasurement value = last_value_;

        // Filter analog values
        if (filter_size_ > 1 && value.get_type() == SensorMeasurement::Type::ANALOG)
        {
            analog_filter_.push_back(value.get_analog());
            if (analog_filter_.size() > filter_size_)
            {
                analog_filter_.pop_front();
            }

            value = filter_value();
        }

        last_value_ = value;
    }

private:
    const uint32_t filter_size_;
    mutable std::deque<SensorMeasurement::AnalogType> analog_filter_;

    SensorMeasurement filter_value() const
    {
        SensorMeasurement::AnalogType value = 0.0f;

        if (!analog_filter_.empty())
        {
            for (const auto & v : analog_filter_)
            {
                value += v;
            }

            value /= analog_filter_.size();
        }

        return SensorMeasurement(value);
    }

};