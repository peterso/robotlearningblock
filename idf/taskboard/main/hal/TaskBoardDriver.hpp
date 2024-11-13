/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <task/Task.hpp>
#include <sensor/Sensor.hpp>

#include <cstdint>
#include <string>

struct TaskBoardDriver
{
    virtual void update() = 0;
    virtual uint32_t get_sensor_count() const = 0;
    virtual const SensorReader* get_sensor(const size_t & index) const = 0;
    virtual const SensorReader * get_sensor_by_name(const std::string & sensor_name) const = 0;
    virtual const std::string & get_unique_id() const = 0;
    virtual Task & get_default_task() = 0;
    virtual Task & get_default_task_precondition() = 0;
};