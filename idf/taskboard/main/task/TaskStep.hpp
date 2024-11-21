/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>
#include <hal/ClueScreenController.hpp>

#include <esp_log.h>

/**
 * @struct TaskStep
 *
 * @brief Base class representing a single condition or action within a task
 *
 * @details A TaskStep defines a goal condition that must be met by a specific sensor
 *          reading. It provides the interface for checking condition completion and
 *          accessing expected values.
 */
struct TaskStep
{
    /**
     * @enum Type
     *
     * @brief Defines the comparison types for sensor value evaluation
     */
    enum class Type
    {
        EQUAL,              ///< Sensor value must exactly match target
        EQUAL_TO_RANDOM,    ///< Sensor value must match a randomly generated target
        GREATER_OR_EQUAL,   ///< Sensor value must be greater or equal to target
        UNKNOWN             ///< UndefOined comparison type
    };

    /**
     * @brief Constructs a new TaskStep object
     *
     * @param sensor Reference to the sensor that will be monitored
     */
    TaskStep(
            SensorReader& sensor)
        : sensor_(sensor)
    {
        // By default a task step begins a read operation on the sensor
        sensor.start_read();
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~TaskStep() = default;

    /**
     * @brief Checks if the step condition has been met
     *
     * @return true if condition is satisfied, false otherwise
     */
    virtual bool success() const = 0;

    /**
     * @brief Gets the target value for this step
     *
     * @return Expected sensor measurement value
     */
    virtual SensorMeasurement expected_value() const = 0;

    /**
     * @brief Show clue on the screen controller
     *
     * @param screen_controller Reference to the screen controller
     */
    virtual void show_clue(
            ClueScreenController& screen_controller) const = 0;

    /**
     * @brief Gets the associated sensor
     *
     * @return Reference to the sensor being monitored
     */
    SensorReader& sensor() const
    {
        return sensor_;
    }

    /**
     * @brief Gets the comparison type for this step
     *
     * @return Reference to the step's comparison type
     */
    const Type& type() const
    {
        return type_;
    }

protected:

    SensorReader& sensor_;     ///< Reference to the monitored sensor
    Type type_ = Type::UNKNOWN;      ///< Type of comparison for success evaluation
};
