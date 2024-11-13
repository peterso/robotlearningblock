/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <sensor/SensorMeasurement.hpp>

#include <esp_log.h>

#include <functional>
#include <string>

/**
 * @struct SensorUpdater
 *
 * @brief Interface for updating sensor values
 *
 * @details Provides mechanism to read and store sensor values using a callback function,
 *          allowing for different sensor implementations
 */
struct SensorUpdater
{
    /// @brief Function type for reading sensor values
    using ReadFunction = std::function<SensorMeasurement()>;

    /**
     * @brief Constructs a new SensorUpdater object
     *
     * @param read_function Callback function that reads the sensor value
     */
    SensorUpdater(
            ReadFunction read_function)
        : read_function_(read_function)
        , last_value_(0.0f)
    {
    }

    /**
     * @brief Updates the stored sensor value using the read function
     */
    virtual void update()
    {
        last_value_ = read_function_();
    }

protected:

    ReadFunction read_function_;        ///< Callback function to read sensor value
    SensorMeasurement last_value_;      ///< Last read sensor measurement
};

/**
 * @struct SensorReader
 *
 * @brief Interface for accessing sensor information and values
 *
 * @details Defines the basic interface for reading sensor values and accessing
 *          sensor metadata
 */
struct SensorReader
{
    /**
     * @brief Gets the sensor name
     *
     * @return Reference to sensor name string
     */
    virtual const std::string& name() const = 0;

    /**
     * @brief Gets the current sensor measurement
     *
     * @return Current sensor value
     */
    virtual const SensorMeasurement read() const = 0;
};

/**
 * @struct Sensor
 *
 * @brief Concrete implementation of a sensor combining update and read capabilities
 */
struct Sensor :
    public SensorUpdater,
    public SensorReader
{
    /**
     * @brief Constructs a new Sensor object
     *
     * @param name Identifier for the sensor
     * @param read_function Callback function that reads the sensor value
     */
    Sensor(
            const std::string& name,
            ReadFunction read_function)
        : SensorUpdater(read_function)
        , name_(name)
    {
        update();
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~Sensor() = default;

    /**
     * @brief Gets the sensor name
     *
     * @return Reference to sensor name string
     */
    const std::string& name() const override
    {
        return name_;
    }

    /**
     * @brief Gets the last read sensor measurement
     *
     * @return Last stored sensor value
     */
    virtual const SensorMeasurement read() const override
    {
        return last_value_;
    }

protected:

    std::string name_;    ///< Sensor identifier
};
