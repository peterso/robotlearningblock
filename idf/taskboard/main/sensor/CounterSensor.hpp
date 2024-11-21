/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>

#include <esp_log.h>

#include <deque>

/**
 * @struct CounterSensor
 *
 * @brief Sensor that counts boolean triggers
 *
 */
struct CounterSensor :
    public Sensor
{
    /**
     * @brief Constructs a new CounterSensor object
     *
     * @param name Identifier for the sensor
     * @param read_function Callback function that reads the sensor value
     */
    CounterSensor(
            const std::string& name,
            ReadFunction read_function,
            uint64_t press_duration_ms = 0)
        : Sensor(name, read_function)
        , press_duration_ms_(press_duration_ms)
    {
        // Get an initial value
        last_value_ = read_function_();
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~CounterSensor() = default;

    /// Virtual method implementation
    void update() override
    {
        Sensor::update();

        const auto new_state = last_value_.get_boolean();

        if (new_state != previous_state_ && previous_state_ == true)
        {
            if (esp_timer_get_time() - last_change_time_ > press_duration_ms_ * 1000)
            {
                count_++;
            }
        }

        if (new_state != previous_state_)
        {
            last_change_time_ = esp_timer_get_time();
            previous_state_ = new_state;
        }

    }

    /// Virtual method implementation
    const SensorMeasurement read() const override
    {
        return SensorMeasurement(count_);
    }

    /// Virtual method implementation
    void start_read() override
    {
        previous_state_ = false;
        last_change_time_ = 0;
        count_ = 0;
    }

private:

    bool previous_state_ = false;       ///< Last sensor value
    uint64_t last_change_time_ = 0;     ///< Time of last change
    int64_t count_ = 0;                 ///< Number of triggers
    const uint64_t press_duration_ms_;  ///< Duration of a press
};
