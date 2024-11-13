/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <hal/TaskBoardDriver.hpp>
#include <sensor/SensorMeasurement.hpp>
#include <util/Timing.hpp>

#include <roboton_taskboard_msgs/msg/task_board_status.h>
#include <roboton_taskboard_msgs/msg/task_status.h>
#include <roboton_taskboard_msgs/action/execute_task.h>

struct MicroROSTypes
{
    struct TaskStep
    {
        TaskStep() = delete;

        static uint8_t get_microros_type(const ::TaskStep::Type & type)
        {
            uint8_t ret = roboton_taskboard_msgs__msg__TaskStep__TASK_STEP_TYPE_UNKNOWN;

            switch(type)
            {
                case ::TaskStep::Type::EQUAL:
                    ret = roboton_taskboard_msgs__msg__TaskStep__TASK_STEP_TYPE_EQUAL;
                    break;
                case ::TaskStep::Type::EQUAL_TO_RANDOM:
                    ret = roboton_taskboard_msgs__msg__TaskStep__TASK_STEP_TYPE_EQUAL_RANDOM;
                    break;
                case ::TaskStep::Type::GREATER_OR_EQUAL:
                    ret = roboton_taskboard_msgs__msg__TaskStep__TASK_STEP_TYPE_GREATER_EQUAL;
                    break;
                default:
                    ret = roboton_taskboard_msgs__msg__TaskStep__TASK_STEP_TYPE_UNKNOWN;
                    break;
            }

            return ret;
        }
    };

    struct TaskStatus
    {
        TaskStatus(const Task & task, const std::string & unique_id)
        {
            microros_msg_.header.stamp = usec_to_microros(esp_timer_get_time());

            microros_msg_.header.frame_id.data = const_cast<char*>(unique_id.c_str());
            microros_msg_.header.frame_id.size = unique_id.size();
            microros_msg_.header.frame_id.capacity = unique_id.size() + 1;

            microros_msg_.task_name.data = (char*)task.name().c_str();
            microros_msg_.task_name.size = task.name().size();
            microros_msg_.task_name.capacity = task.name().size() + 1;

            microros_msg_.elapsed_time = usec_to_microros(task.elapsed_time());

            microros_msg_.steps.data = new roboton_taskboard_msgs__msg__TaskStep[task.total_steps()];
            microros_msg_.steps.size = task.total_steps();
            microros_msg_.steps.capacity = task.total_steps();

            microros_msg_.status.data = new bool[task.total_steps()];
            microros_msg_.status.size = task.total_steps();
            microros_msg_.status.capacity = task.total_steps();

            for (size_t i = 0; i < task.total_steps(); i++)
            {
                const ::TaskStep & step = task.step(i);

                roboton_taskboard_msgs__msg__TaskStep & msg_step = microros_msg_.steps.data[i];
                msg_step.sensor_name.data = (char*)step.sensor().name().c_str();
                msg_step.sensor_name.size = step.sensor().name().size();
                msg_step.sensor_name.capacity = step.sensor().name().size() + 1;

                // msg_step.type = step.to_microros_step_type();
                msg_step.type = MicroROSTypes::TaskStep::get_microros_type(step.type());
                msg_step.target = {};
                msg_step.target.type = MicroROSTypes::SensorMeasurement::get_microros_type(step.expected_value().get_type());
                msg_step.tolerance = 0.0;

                switch (step.expected_value().get_type())
                {
                case ::SensorMeasurement::Type::BOOLEAN:
                    msg_step.target.bool_value.data = new bool;
                    msg_step.target.bool_value.data[0] = step.expected_value().get_boolean();
                    msg_step.target.bool_value.size = 1;
                    msg_step.target.bool_value.capacity = 1;
                    break;
                case ::SensorMeasurement::Type::ANALOG:
                    msg_step.target.analog_value.data = new float;
                    msg_step.target.analog_value.data[0] = step.expected_value().get_analog();
                    msg_step.target.analog_value.size = 1;
                    msg_step.target.analog_value.capacity = 1;
                    break;
                case ::SensorMeasurement::Type::VECTOR3:
                    msg_step.target.vector3_value.data = new geometry_msgs__msg__Vector3;
                    msg_step.target.vector3_value.data[0].x = step.expected_value().get_vector3().x;
                    msg_step.target.vector3_value.data[0].y = step.expected_value().get_vector3().y;
                    msg_step.target.vector3_value.data[0].z = step.expected_value().get_vector3().z;
                    msg_step.target.vector3_value.size = 1;
                    msg_step.target.vector3_value.capacity = 1;
                    break;
                }

                microros_msg_.status.data[i] = task.step_done(i);
            }
        }

        TaskStatus(const std::string & unique_id)
        {
            microros_msg_.header.stamp = usec_to_microros(esp_timer_get_time());

            microros_msg_.header.frame_id.data = const_cast<char*>(unique_id.c_str());
            microros_msg_.header.frame_id.size = unique_id.size();
            microros_msg_.header.frame_id.capacity = unique_id.size() + 1;

            static const char empty_string = '\0';
            microros_msg_.task_name.data = (char*)&empty_string;
            microros_msg_.task_name.size = 0;
            microros_msg_.task_name.capacity = 1;

            microros_msg_.steps.data = nullptr;
            microros_msg_.steps.size = 0;
            microros_msg_.steps.capacity = 0;

            microros_msg_.status.data = nullptr;
            microros_msg_.status.size = 0;
            microros_msg_.status.capacity = 0;
        }

        ~TaskStatus()
        {
            for (size_t i = 0; i < microros_msg_.status.size; i++)
            {
                switch (microros_msg_.steps.data[i].target.type)
                {
                case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_BOOL:
                    delete microros_msg_.steps.data[i].target.bool_value.data;
                    break;
                case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_ANALOG:
                    delete microros_msg_.steps.data[i].target.analog_value.data;
                    break;
                case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_VECTOR3:
                    delete microros_msg_.steps.data[i].target.vector3_value.data;
                    break;
                default:
                    break;
                }
            }

            if (microros_msg_.steps.data != nullptr)
            {
                delete microros_msg_.steps.data;
            }

            if (microros_msg_.status.data != nullptr)
            {
                delete microros_msg_.status.data;
            }
        }

        void set_waiting_precondition(const bool & waiting_precondition)
        {
            microros_msg_.waiting_precondition = waiting_precondition;
        }

        const roboton_taskboard_msgs__msg__TaskStatus & get_microros_msg() const
        {
            return microros_msg_;
        }

    private:
        roboton_taskboard_msgs__msg__TaskStatus microros_msg_;
    };

    struct SensorMeasurement
    {
        SensorMeasurement(const ::SensorMeasurement & sensor_measurement)
        {
            microros_msg_ = {};
            microros_msg_.type = get_microros_type(sensor_measurement.get_type());

            switch (sensor_measurement.get_type())
            {
            case ::SensorMeasurement::Type::BOOLEAN:
                microros_msg_.bool_value.data = new bool;
                microros_msg_.bool_value.size = 1;
                microros_msg_.bool_value.capacity = 1;
                microros_msg_.bool_value.data[0] = sensor_measurement.get_boolean();
                break;

            case ::SensorMeasurement::Type::ANALOG:
                microros_msg_.analog_value.data = new float;
                microros_msg_.analog_value.size = 1;
                microros_msg_.analog_value.capacity = 1;
                microros_msg_.analog_value.data[0] = sensor_measurement.get_analog();
                break;

            case ::SensorMeasurement::Type::VECTOR3:
                microros_msg_.vector3_value.data = new geometry_msgs__msg__Vector3;
                microros_msg_.vector3_value.size = 1;
                microros_msg_.vector3_value.capacity = 1;
                microros_msg_.vector3_value.data[0].x = sensor_measurement.get_vector3().x;
                microros_msg_.vector3_value.data[0].y = sensor_measurement.get_vector3().y;
                microros_msg_.vector3_value.data[0].z = sensor_measurement.get_vector3().z;
            default:
                microros_msg_.type = roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_UNKNOWN;
                break;
            }
        }

        static uint8_t get_microros_type(const ::SensorMeasurement::Type & sensor_measurement_type)
        {
            uint8_t ret = roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_UNKNOWN;

            switch(sensor_measurement_type)
            {
                case ::SensorMeasurement::Type::BOOLEAN:
                    ret = roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_BOOL;
                    break;
                case ::SensorMeasurement::Type::ANALOG:
                    ret = roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_ANALOG;
                    break;
                case ::SensorMeasurement::Type::VECTOR3:
                    ret = roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_VECTOR3;
                    break;
                default:
                    ret = roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_UNKNOWN;
                    break;
            }

            return ret;
        }

        static ::SensorMeasurement::Type get_type_from_microros(const uint8_t & type)
        {
            ::SensorMeasurement::Type ret = ::SensorMeasurement::Type::BOOLEAN;

            switch(type)
            {
                case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_BOOL:
                    ret = ::SensorMeasurement::Type::BOOLEAN;
                    break;
                case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_ANALOG:
                    ret = ::SensorMeasurement::Type::ANALOG;
                    break;
                case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_VECTOR3:
                    ret = ::SensorMeasurement::Type::VECTOR3;
                    break;
                default:
                    ret = ::SensorMeasurement::Type::BOOLEAN;
                    break;
            }

            return ret;
        }

        static ::SensorMeasurement from_microros(const roboton_taskboard_msgs__msg__SensorMeasurement & msg)
        {
            ::SensorMeasurement ret(0.0f);

            switch(msg.type)
            {
                case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_BOOL:
                    if (msg.bool_value.size > 0)
                    {
                        ret = ::SensorMeasurement(msg.bool_value.data[0]);
                    }
                    break;
                case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_ANALOG:
                    if (msg.analog_value.size > 0)
                    {
                        ret = ::SensorMeasurement(msg.analog_value.data[0]);
                    }
                    break;
                case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_VECTOR3:
                    if (msg.vector3_value.size > 0)
                    {
                        ret = ::SensorMeasurement(
                            ::SensorMeasurement::Vector3{
                                (float) msg.vector3_value.data[0].x,
                                (float) msg.vector3_value.data[0].y,
                                (float) msg.vector3_value.data[0].z});
                    }
                    break;
                default:
                    break;
            }

            return ret;
        }

        ~SensorMeasurement()
        {
            switch (microros_msg_.type)
            {
            case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_BOOL:
                delete microros_msg_.bool_value.data;
                break;
            case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_ANALOG:
                delete microros_msg_.analog_value.data;
                break;
            case roboton_taskboard_msgs__msg__SensorMeasurement__SENSOR_MEASUREMENT_TYPE_VECTOR3:
                delete microros_msg_.vector3_value.data;
                break;
            default:
                break;
            }
        }

        const roboton_taskboard_msgs__msg__SensorMeasurement & get_microros_msg() const
        {
            return microros_msg_;
        }

    private:
        roboton_taskboard_msgs__msg__SensorMeasurement microros_msg_;
    };

    struct TaskBoardStatus
    {
        TaskBoardStatus(const TaskBoardDriver & task_board_driver)
        {
            // Fill header
            microros_msg_.header.stamp = usec_to_microros(esp_timer_get_time());
            microros_msg_.header.frame_id.data = const_cast<char*>(task_board_driver.get_unique_id().c_str());
            microros_msg_.header.frame_id.size = task_board_driver.get_unique_id().size();
            microros_msg_.header.frame_id.capacity = task_board_driver.get_unique_id().size() + 1;

            // Fill sensors
            microros_msg_.sensors.data = new roboton_taskboard_msgs__msg__Sensor[task_board_driver.get_sensor_count()];
            microros_msg_.sensors.size = task_board_driver.get_sensor_count();
            microros_msg_.sensors.capacity = task_board_driver.get_sensor_count();

            for (size_t i = 0; i < task_board_driver.get_sensor_count(); i++)
            {
                const auto & sensor = *task_board_driver.get_sensor(i);
                auto & sensor_data = microros_msg_.sensors.data[i];

                sensor_data = {};

                sensor_data.name.data = const_cast<char*>(sensor.name().c_str());
                sensor_data.name.size = sensor.name().size();
                sensor_data.name.capacity = sensor.name().size() + 1;

                const auto & sensor_value = sensor.read();
                sensor_data.value.type = SensorMeasurement::get_microros_type(sensor_value.get_type());

                switch (sensor_value.get_type())
                {
                case ::SensorMeasurement::Type::BOOLEAN:
                    sensor_data.value.bool_value.data = new bool;
                    sensor_data.value.bool_value.size = 1;
                    sensor_data.value.bool_value.capacity = 1;
                    sensor_data.value.bool_value.data[0] = sensor_value.get_boolean();
                    break;
                case ::SensorMeasurement::Type::ANALOG:
                    sensor_data.value.analog_value.data = new float;
                    sensor_data.value.analog_value.size = 1;
                    sensor_data.value.analog_value.capacity = 1;
                    sensor_data.value.analog_value.data[0] = sensor_value.get_analog();
                    break;
                case ::SensorMeasurement::Type::VECTOR3:
                    sensor_data.value.vector3_value.data = new geometry_msgs__msg__Vector3;
                    sensor_data.value.vector3_value.size = 1;
                    sensor_data.value.vector3_value.capacity = 1;
                    sensor_data.value.vector3_value.data[0].x = sensor_value.get_vector3().x;
                    sensor_data.value.vector3_value.data[0].y = sensor_value.get_vector3().y;
                    sensor_data.value.vector3_value.data[0].z = sensor_value.get_vector3().z;
                    break;
                }
            }

        }

        ~TaskBoardStatus()
        {
            // Free sensors
            for (size_t i = 0; i < microros_msg_.sensors.size; i++)
            {
                auto & sensor_data = microros_msg_.sensors.data[i];
                switch (SensorMeasurement::get_type_from_microros(sensor_data.value.type))
                {
                case ::SensorMeasurement::Type::BOOLEAN:
                    delete sensor_data.value.bool_value.data;
                    break;
                case ::SensorMeasurement::Type::ANALOG:
                    delete sensor_data.value.analog_value.data;
                    break;
                case ::SensorMeasurement::Type::VECTOR3:
                    delete sensor_data.value.vector3_value.data;
                    break;
                }
            }

            delete microros_msg_.sensors.data;
        }

        const roboton_taskboard_msgs__msg__TaskBoardStatus & get_microros_msg() const
        {
            return microros_msg_;
        }

        private:
            roboton_taskboard_msgs__msg__TaskBoardStatus microros_msg_;
    };

    struct SendGoalRequest
    {

        SendGoalRequest(const size_t max_task_name_size, const size_t max_sensor_name_size, const size_t max_steps_per_task)
        {
            // Initialize goal memory
            microros_msg_.goal.task.name.data = new char[max_task_name_size];
            microros_msg_.goal.task.name.size = 0;
            microros_msg_.goal.task.name.capacity = max_task_name_size;

            microros_msg_.goal.task.steps.data = new roboton_taskboard_msgs__msg__TaskStep[max_steps_per_task];
            microros_msg_.goal.task.steps.size = 0;
            microros_msg_.goal.task.steps.capacity = max_steps_per_task;

            // For each sensor intialize values
            for (size_t i = 0; i < max_steps_per_task; i++)
            {
                microros_msg_.goal.task.steps.data[i].sensor_name.data = new char[max_sensor_name_size];
                microros_msg_.goal.task.steps.data[i].sensor_name.size = 0;
                microros_msg_.goal.task.steps.data[i].sensor_name.capacity = max_sensor_name_size;

                microros_msg_.goal.task.steps.data[i].target.bool_value.data = new bool;
                microros_msg_.goal.task.steps.data[i].target.bool_value.size = 0;
                microros_msg_.goal.task.steps.data[i].target.bool_value.capacity = 1;

                microros_msg_.goal.task.steps.data[i].target.analog_value.data = new float;
                microros_msg_.goal.task.steps.data[i].target.analog_value.size = 0;
                microros_msg_.goal.task.steps.data[i].target.analog_value.capacity = 1;

                microros_msg_.goal.task.steps.data[i].target.vector3_value.data = new geometry_msgs__msg__Vector3;
                microros_msg_.goal.task.steps.data[i].target.vector3_value.size = 0;
                microros_msg_.goal.task.steps.data[i].target.vector3_value.capacity = 1;
            }
        }

        ~SendGoalRequest()
        {
            if (microros_msg_.goal.task.name.data != nullptr)
            {
                delete microros_msg_.goal.task.name.data;
            }

            for (size_t i = 0; i < microros_msg_.goal.task.steps.size; i++)
            {
                if (microros_msg_.goal.task.steps.data[i].sensor_name.data != nullptr)
                {
                    delete microros_msg_.goal.task.steps.data[i].sensor_name.data;
                }

                if (microros_msg_.goal.task.steps.data[i].target.bool_value.data != nullptr)
                {
                    delete microros_msg_.goal.task.steps.data[i].target.bool_value.data;
                }

                if (microros_msg_.goal.task.steps.data[i].target.analog_value.data != nullptr)
                {
                    delete microros_msg_.goal.task.steps.data[i].target.analog_value.data;
                }

                if (microros_msg_.goal.task.steps.data[i].target.vector3_value.data != nullptr)
                {
                    delete microros_msg_.goal.task.steps.data[i].target.vector3_value.data;
                }
            }

            if (microros_msg_.goal.task.steps.data != nullptr)
            {
                delete microros_msg_.goal.task.steps.data;
            }
        }

        const roboton_taskboard_msgs__action__ExecuteTask_SendGoal_Request & get_microros_msg() const
        {
            return microros_msg_;
        }

    private:

        roboton_taskboard_msgs__action__ExecuteTask_SendGoal_Request microros_msg_;
    };

    struct FeedbackMessage
    {
        FeedbackMessage(const Task& task)
        {
            microros_msg_.feedback.elapsed_time = usec_to_microros(task.elapsed_time());
        }

        const roboton_taskboard_msgs__action__ExecuteTask_FeedbackMessage & get_microros_msg() const
        {
            return microros_msg_;
        }

        private:
            roboton_taskboard_msgs__action__ExecuteTask_FeedbackMessage microros_msg_;
    };

    struct ResultMessage
    {

        ResultMessage(const Task & task)
        {
            microros_msg_.result.finish_time = usec_to_microros(task.elapsed_time());
        }

        ResultMessage()
        {
            microros_msg_.result.finish_time = {};
        }

        const roboton_taskboard_msgs__action__ExecuteTask_GetResult_Response & get_microros_msg() const
        {
            return microros_msg_;
        }

    private:
        roboton_taskboard_msgs__action__ExecuteTask_GetResult_Response microros_msg_;
    };
};

