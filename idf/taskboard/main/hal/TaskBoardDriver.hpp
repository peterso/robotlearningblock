/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <task/Task.hpp>
#include <sensor/Sensor.hpp>

#include <cstdint>
#include <string>

/**
 * @struct TaskBoardDriver
 *
 * @brief Interface for interacting with a physical task board configuration
 *
 * @details Provides a common interface for hardware abstraction of task boards,
 *          allowing access to sensors, board identification, and default tasks.
 *          Implementations handle the specific hardware communication details.
 */
struct TaskBoardDriver
{
    /**
     * @brief Updates the state of all sensors on the board
     *
     * @details Should be called periodically to refresh sensor readings
     */
    virtual void update() = 0;

    /**
     * @brief Gets the unique identifier of the task board
     *
     * @return Reference to the board's unique ID string
     */
    virtual const std::string& get_unique_id() const = 0;

    /**
     * @brief Gets the total number of sensors available on the board
     *
     * @return Number of sensors
     */
    virtual uint32_t get_sensor_count() const = 0;

    /**
     * @brief Gets a sensor by its index
     *
     * @param index Zero-based index of the sensor to retrieve
     *
     * @return Pointer to the sensor reader interface, or nullptr if index is invalid
     */
    virtual SensorReader* get_sensor(
            const size_t& index) const = 0;

    /**
     * @brief Gets a sensor by its name
     *
     * @param sensor_name Name identifier of the sensor to retrieve
     *
     * @return Pointer to the sensor reader interface, or nullptr if name not found
     */
    virtual SensorReader* get_sensor_by_name(
            const std::string& sensor_name) const = 0;

    /**
     * @brief Gets the default task configured for this board
     *
     * @return Reference to the default task
     */
    virtual Task& get_default_task() = 0;

    /**
     * @brief Gets the precondition task that must be satisfied before the default task
     *
     * @return Reference to the precondition task
     */
    virtual Task& get_default_task_precondition() = 0;
};
