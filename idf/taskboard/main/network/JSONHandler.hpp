/**
 * Robothon Task Board Firmware
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

/**
 * @struct JSONHandler
 *
 * @brief Helper class for creating JSON objects
 */
struct JSONHandler
{
    /**
     * @brief Constructs a new JSONHandler object
     */
    JSONHandler()
    {
        root_ = cJSON_CreateObject();
    }

    /**
     * @brief Constructs a new JSONHandler object
     *
     * @param unique_id Unique ID of the device
     */
    JSONHandler(
            const std::string& unique_id)
        : JSONHandler()
    {
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
     * @brief Add custom key-value pair to the JSON object
     *
     * @param key Key
     * @param value Value
     */
    void add_custom(
            const std::string& key,
            const std::string& value)
    {
        cJSON_AddStringToObject(root_, key.c_str(), value.c_str());
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
        cJSON_AddNumberToObject(root_, "min_free_heap", esp_get_minimum_free_heap_size());
        cJSON_AddNumberToObject(root_, "total_time", total_run_time);
    }

    /**
     * @brief Adds task board status to the JSON object
     *
     * @param taskboard_driver Sensor ID
     */
    void add_taskboard_status(
            const TaskBoardDriver& task_board_driver)
    {
        // Add sensors array
        cJSON* sensors_ = cJSON_CreateArray();
        cJSON_AddItemToObject(root_, "sensors", sensors_);

        for (size_t i = 0; i < task_board_driver.get_sensor_count(); i++)
        {
            const SensorReader* sensor_dev = task_board_driver.get_sensor(i);

            if (sensor_dev != nullptr)
            {
                cJSON* sensor = cJSON_CreateObject();
                cJSON_AddStringToObject(sensor, "id", sensor_dev->name().c_str());

                const auto measurement = sensor_dev->read();

                switch (measurement.get_type())
                {
                    case SensorMeasurement::Type::BOOLEAN:
                        cJSON_AddBoolToObject(sensor, "value", measurement.get_boolean());
                        break;
                    case SensorMeasurement::Type::ANALOG:
                        cJSON_AddNumberToObject(sensor, "value", measurement.get_analog());
                        break;
                    case SensorMeasurement::Type::VECTOR3:
                    {
                        cJSON* vector = cJSON_CreateObject();
                        cJSON_AddNumberToObject(vector, "x", measurement.get_vector3().x);
                        cJSON_AddNumberToObject(vector, "y", measurement.get_vector3().y);
                        cJSON_AddNumberToObject(vector, "z", measurement.get_vector3().z);
                        cJSON_AddItemToObject(sensor, "value", vector);
                        break;
                    }
                    case SensorMeasurement::Type::INTEGER:
                    {
                        const double value = static_cast<double>(measurement.get_integer());
                        cJSON_AddNumberToObject(sensor, "value", value);
                        cJSON* is_integer = cJSON_CreateTrue();
                        cJSON_AddItemToObject(sensor, "is_integer", is_integer);
                        break;
                    }
                    case SensorMeasurement::Type::EMPTY:
                        break;
                }

                cJSON_AddItemToArray(sensors_, sensor);
            }
        }
    }

    /**
     * @brief Adds task board status to the JSON object in Kaa format
     *
     * @param taskboard_driver Sensor ID
     */
    void add_taskboard_status_kaaiot(
            const TaskBoardDriver& task_board_driver)
    {
        for (size_t i = 0; i < task_board_driver.get_sensor_count(); i++)
        {
            const SensorReader* sensor_dev = task_board_driver.get_sensor(i);

            if (sensor_dev != nullptr)
            {
                const auto measurement = sensor_dev->read();

                switch (measurement.get_type())
                {
                    case SensorMeasurement::Type::BOOLEAN:
                        cJSON_AddBoolToObject(root_, sensor_dev->name().c_str(), measurement.get_boolean());
                        break;
                    case SensorMeasurement::Type::ANALOG:
                        cJSON_AddNumberToObject(root_, sensor_dev->name().c_str(), measurement.get_analog());
                        break;
                    case SensorMeasurement::Type::VECTOR3:
                    {
                        {
                            std::string vector_name = sensor_dev->name() + "_x";
                            cJSON_AddNumberToObject(root_, vector_name.c_str(), measurement.get_vector3().x);
                            vector_name = sensor_dev->name() + "_y";
                            cJSON_AddNumberToObject(root_, vector_name.c_str(), measurement.get_vector3().y);
                            vector_name = sensor_dev->name() + "_z";
                            cJSON_AddNumberToObject(root_, vector_name.c_str(), measurement.get_vector3().z);
                        }
                        break;
                    }
                    case SensorMeasurement::Type::INTEGER:
                    {
                        const double value = static_cast<double>(measurement.get_integer());
                        cJSON_AddNumberToObject(root_, sensor_dev->name().c_str(), value);
                        break;
                    }
                    case SensorMeasurement::Type::EMPTY:
                        break;
                }
            }
        }
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
            const Task& task,
            const Task* precondition)
    {
        cJSON* current_task = cJSON_CreateObject();

        cJSON_AddStringToObject(current_task, "name", task.name().c_str());
        cJSON_AddStringToObject(current_task, "unique_id", task.unique_id().c_str());
        cJSON_AddNumberToObject(current_task, "time", precondition ? 0 : task.elapsed_time() / 1e6);

        cJSON* waiting_precondition = nullptr == precondition ? cJSON_CreateFalse() : cJSON_CreateTrue();
        cJSON_AddItemToObject(current_task, "waiting_precondition", waiting_precondition);

        cJSON* finished = task.done() ? cJSON_CreateTrue() : cJSON_CreateFalse();
        cJSON_AddItemToObject(current_task, "finished", finished);

        cJSON* steps = cJSON_CreateArray();

        for (size_t i = 0; i < task.total_steps(); i++)
        {
            if (!task.step(i).show_to_user())
            {
                continue;
            }

            cJSON* step = cJSON_CreateObject();
            cJSON_AddStringToObject(step, "sensor", task.step(i).name().c_str());
            cJSON* done = task.step_done(i) ? cJSON_CreateTrue() : cJSON_CreateFalse();
            cJSON_AddItemToObject(step, "done", done);

            if (task.step_done(i))
            {
                cJSON_AddNumberToObject(step, "score", task.step_score(i));
                cJSON_AddNumberToObject(step, "finish_time", task.step_done_time(i) / 1e6);
            }

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
        if (json_string_ != nullptr)
        {
            cJSON_free(json_string_);
        }

        json_string_ = cJSON_PrintUnformatted(root_);

        return json_string_;
    }

private:

    cJSON* root_ = nullptr;             ///< Root JSON object
    char* json_string_ = nullptr;       ///< JSON string
};
