/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>

#include <esp_log.h>

#include <deque>

/**
 * @struct TriggeredSensor
 *
 * @brief Implementation of Sensor only updates when triggered
 *
 */
struct TriggeredSensor :
    public Sensor
{
    /**
     * @brief Constructs a new TriggeredSensor object
     *
     * @param name Identifier for the sensor
     * @param read_function Callback function that reads the sensor value
     */
    TriggeredSensor(
            const std::string& name,
            const Sensor& trigger_sensor,
            ReadFunction read_function)
        : Sensor(name, read_function)
        , trigger_sensor_(trigger_sensor)
    {
        // Get an initial value
        last_value_ = read_function_();
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~TriggeredSensor() = default;

    /// Virtual method implementation
    void update() override
    {
        const bool can_update = trigger_sensor_.read().get_type() == SensorMeasurement::Type::BOOLEAN &&
                trigger_sensor_.read().get_boolean();

        if (can_update)
        {
            last_value_ = read_function_();
        }
    }

private:

    const Sensor& trigger_sensor_;  ///< Sensor that enables updates
};
