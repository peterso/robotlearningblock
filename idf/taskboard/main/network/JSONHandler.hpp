/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>
#include <sensor/SensorMeasurement.hpp>
#include <microros/MicroROSController.hpp>
#include <task/Task.hpp>

#include <cJSON.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>

// TODO(pgarrido): This can be highly optimized by mantaining a single cJSON object and updating it

/**
 * @struct JSONHandler
 *
 * @brief Helper class for creating JSON objects
 */
struct JSONHandler
{
    /**
     * @brief Constructs a new JSONHandler object
     *
     * @param unique_id Unique ID of the device
     */
    JSONHandler(
            const std::string& unique_id)
    {
        root_ = cJSON_CreateObject();

        cJSON_AddStringToObject(root_, "device_id", unique_id.c_str());
    }

    /**
     * @brief Destructor
     */
    ~JSONHandler()
    {
        cJSON_Delete(root_);

        if (json_string_ != nullptr)
        {
            cJSON_free(json_string_);
        }
    }

    /**
     * @brief Adds FreeRTOS summary to the JSON object
     */
    void add_freertos_summary()
    {
        // Add freertos task list with stack consumption
        cJSON* tasks = cJSON_CreateArray();
        cJSON_AddItemToObject(root_, "tasks", tasks);

        unsigned long total_run_time = 0;
        uint32_t task_number = uxTaskGetNumberOfTasks();
        TaskStatus_t* task_status_array = (TaskStatus_t*) pvPortMalloc( task_number * sizeof( TaskStatus_t ));
        UBaseType_t task_count = uxTaskGetSystemState(task_status_array, task_number, &total_run_time);

        for (uint32_t i = 0; i < task_count; i++)
        {
            cJSON* task = cJSON_CreateObject();
            cJSON_AddStringToObject(task, "name", task_status_array[i].pcTaskName);
            cJSON_AddNumberToObject(task, "stack", task_status_array[i].usStackHighWaterMark);
            cJSON_AddNumberToObject(task, "cpu", task_status_array[i].ulRunTimeCounter);
            cJSON_AddNumberToObject(task, "priority", task_status_array[i].uxCurrentPriority);
            cJSON_AddNumberToObject(task, "core", xTaskGetCoreID(task_status_array[i].xHandle));
            cJSON_AddItemToArray(tasks, task);
        }

        vPortFree(task_status_array);

        // Add free heap
        cJSON_AddNumberToObject(root_, "free_heap", esp_get_free_heap_size());
        cJSON_AddNumberToObject(root_, "total_time", total_run_time);
    }

    /**
     * @brief Adds sensor measurement to the JSON object
     *
     * @param id Sensor ID
     * @param measurement Sensor measurement
     */
    void add_sensor_measure(
            const std::string& id,
            const SensorMeasurement& measurement)
    {
        if (sensors_ == nullptr)
        {
            // Add sensors array
            sensors_ = cJSON_CreateArray();
            cJSON_AddItemToObject(root_, "sensors", sensors_);
        }

        cJSON* sensor = cJSON_CreateObject();
        cJSON_AddStringToObject(sensor, "id", id.c_str());

        switch (measurement.get_type())
        {
            case SensorMeasurement::Type::BOOLEAN:
                cJSON_AddBoolToObject(sensor, "value", measurement.get_boolean());
                break;
            case SensorMeasurement::Type::ANALOG:
                cJSON_AddNumberToObject(sensor, "value", measurement.get_analog());
                break;
            case SensorMeasurement::Type::VECTOR3:
                cJSON* vector = cJSON_CreateObject();
                cJSON_AddNumberToObject(vector, "x", measurement.get_vector3().x);
                cJSON_AddNumberToObject(vector, "y", measurement.get_vector3().y);
                cJSON_AddNumberToObject(vector, "z", measurement.get_vector3().z);
                cJSON_AddItemToObject(sensor, "value", vector);
                break;
        }

        cJSON_AddItemToArray(sensors_, sensor);
    }

    /**
     * @brief Adds micro-ROS information to the JSON object
     *
     * @param micro_ros_node Reference to the micro-ROS controller
     */
    void add_microros_info(
            const MicroROSController& micro_ros_node)
    {
        cJSON* microros = cJSON_CreateObject();
        cJSON_AddStringToObject(microros, "agent_ip", micro_ros_node.get_agent_ip().c_str());
        cJSON_AddStringToObject(microros, "agent_port", micro_ros_node.get_agent_port().c_str());
        cJSON* connected = micro_ros_node.is_agent_connected() ? cJSON_CreateTrue() : cJSON_CreateFalse();
        cJSON_AddItemToObject(microros, "connected", connected);
        cJSON_AddItemToObject(root_, "microros", microros);
    }

    /**
     * @brief Adds current task status to the JSON object
     *
     * @param task Task to add
     */
    void add_task_status(
            const Task& task)
    {
        cJSON* current_task = cJSON_CreateObject();

        cJSON_AddStringToObject(current_task, "name", task.name().c_str());
        cJSON_AddNumberToObject(current_task, "time", task.elapsed_time() / 1e6);

        cJSON* finished = task.done() ? cJSON_CreateTrue() : cJSON_CreateFalse();
        cJSON_AddItemToObject(current_task, "finished", finished);

        cJSON* steps = cJSON_CreateArray();

        for (size_t i = 0; i < task.total_steps(); i++)
        {
            cJSON* step = cJSON_CreateObject();
            cJSON_AddStringToObject(step, "sensor", task.step(i).sensor().name().c_str());
            cJSON* done = task.step_done(i) ? cJSON_CreateTrue() : cJSON_CreateFalse();
            cJSON_AddItemToObject(step, "done", done);
            cJSON_AddItemToArray(steps, step);
        }

        cJSON_AddItemToObject(current_task, "steps", steps);

        cJSON_AddItemToObject(root_, "current_task", current_task);
    }

    /**
     * @brief Gets the JSON string
     *
     * @return JSON string
     */
    char* get_json_string()
    {
        if (json_string_ == nullptr)
        {
            json_string_ = cJSON_Print(root_);
        }

        return json_string_;
    }

private:

    cJSON* root_ = nullptr;             ///< Root JSON object
    cJSON* sensors_ = nullptr;          ///< Sensors array
    char* json_string_ = nullptr;       ///< JSON string
};
