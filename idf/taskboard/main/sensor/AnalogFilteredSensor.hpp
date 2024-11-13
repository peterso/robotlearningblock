/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>

#include <esp_log.h>

#include <deque>

/**
 * @struct AnalogFilteredSensor
 *
 * @brief Implementation of Sensor that applies filtering to analog measurements
 *
 * @details Extends the basic Sensor class by adding a moving average filter
 *          for analog sensor values. The filter is only applied to analog
 *          measurements; other measurement types pass through unmodified.
 */
struct AnalogFilteredSensor :
    public Sensor
{
    /**
     * @brief Constructs a new AnalogFilteredSensor object
     *
     * @param name Identifier for the sensor
     * @param filter_size Number of samples to use in moving average filter
     * @param read_function Callback function that reads the sensor value
     */
    AnalogFilteredSensor(
            const std::string& name,
            uint32_t filter_size,
            ReadFunction read_function)
        : Sensor(name, read_function)
        , filter_size_(filter_size)
    {
        if (filter_size_ > 1)
        {
            analog_filter_.resize(filter_size);
        }
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~AnalogFilteredSensor() = default;

    /**
     * @brief Updates the sensor value with optional filtering
     *
     * @details If the measurement is analog and filtering is enabled (filter_size > 1),
     *          applies a moving average filter to the reading. Non-analog measurements
     *          or cases where filter_size <= 1 pass through unmodified.
     */
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

    /**
     * @brief Calculates the filtered sensor value
     * @details Computes the arithmetic mean of all samples in the filter buffer.
     *          Returns 0.0f if the buffer is empty.
     *
     * @return Filtered sensor measurement
     */
    SensorMeasurement filter_value() const
    {
        SensorMeasurement::AnalogType value = 0.0f;

        if (!analog_filter_.empty())
        {
            for (const auto& v : analog_filter_)
            {
                value += v;
            }

            value /= analog_filter_.size();
        }

        return SensorMeasurement(value);
    }

    const uint32_t filter_size_;                                         ///< Number of samples to use in moving average filter
    mutable std::deque<SensorMeasurement::AnalogType> analog_filter_;    ///< Circular buffer for filter samples

};
