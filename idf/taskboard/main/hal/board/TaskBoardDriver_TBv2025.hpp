/**
 * Robothon Task Board Firmware
 TEST PETER WAS HERE
 */

#pragma once

#include <hal/TaskBoardDriver.hpp>
#include <hal/HardwareLowLevelController.hpp>
#include <sensor/Sensor.hpp>
#include <sensor/AnalogFilteredSensor.hpp>
#include <sensor/CounterSensor.hpp>
#include <sensor/TriggeredSensor.hpp>
#include <task/TaskStepEqual.hpp>
#include <task/TaskStepEqualToRandom.hpp>
#include <task/TaskStepTraceShape.hpp>
#include <task/TaskStepTraceShapeManagePool.hpp>
#include <task/TaskStepTraceShapeFromPool.hpp>
#include <task/TaskStepWaitRandom.hpp>
#include <task/SimultaneousConditionTask.hpp>
#include <task/SequentialTask.hpp>
#include <util/Timing.hpp>

#include <esp_mac.h>

/**
 * @struct TaskBoardDriver_v1
 *
 * @brief Implementation of TaskBoardDriver for version 1 hardware
 */
struct TaskBoardDriver_v1 :
    public TaskBoardDriver
{
    const char* TAG = "TaskBoardDriver_v1";    ///< Logging tag

    /**
     * @brief Constructs a new TaskBoardDriver_v1 object
     *
     * @param hardware_low_level_controller Reference to hardware interface
     */
    TaskBoardDriver_v1(
            HardwareLowLevelController& hardware_low_level_controller)
        : hardware_low_level_controller_(hardware_low_level_controller)
    {
        // Fill unique id
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        char mac_str[18];
        sprintf(mac_str, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        unique_id_ = mac_str;
        char ssid_with_mac[32];
        snprintf(ssid_with_mac, sizeof(ssid_with_mac), "Robothon Task Board %01X%02X", (mac[4] & 0x0F), mac[5]);
        unique_ssid_ = ssid_with_mac;

        // Initialize sensors
        Sensor* ball_goal_1 = new Sensor("BALL_GOAL_1", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller_1.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_0);

                            return SensorMeasurement(value);
                        });

        Sensor* ball_goal_2 = new Sensor("BALL_GOAL_2", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller_1.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_1);

                            return SensorMeasurement(value);
                        });

        Sensor* ball_goal_3 = new Sensor("BALL_GOAL_3", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller_1.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_2);

                            return SensorMeasurement(value);
                        });

        Sensor* ball_goal_4 = new Sensor("BALL_GOAL_4", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller_1.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_3);

                            return SensorMeasurement(value);
                        });

        Sensor* blue_button_left = new Sensor("BLUE_BUTTON_LEFT", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller_1.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_4);

                            return SensorMeasurement(!value); // Button is inverted
                        });

        Sensor* red_button_left = new Sensor("RED_BUTTON_LEFT", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller_1.read_digital_IO1(PbHubController::Channel::
                                    CHANNEL_4);

                            return SensorMeasurement(!value); // Button is inverted
                        });

        Sensor* blue_button_right = new Sensor("BLUE_BUTTON_RIGHT", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller_1.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_5);

                            return SensorMeasurement(!value); // Button is inverted
                        });

        Sensor* red_button_right = new Sensor("RED_BUTTON_RIGHT", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller_1.read_digital_IO1(PbHubController::Channel::
                                    CHANNEL_5);

                            return SensorMeasurement(!value); // Button is inverted
                        });

        Sensor* on_board_button_a = new Sensor("ON_BOARD_BUTTON_A", [&]()
                        {
                            bool value = hardware_low_level_controller_.m5_unified.BtnA.isPressed();

                            return SensorMeasurement(value);
                        });

        Sensor* on_board_button_b = new Sensor("ON_BOARD_BUTTON_B", [&]()
                        {
                            bool value = hardware_low_level_controller_.m5_unified.BtnB.isPressed();

                            return SensorMeasurement(value);
                        });

        Sensor* on_board_button_c = new Sensor("ON_BOARD_BUTTON_C", [&]()
                        {
                            bool value = hardware_low_level_controller_.m5_unified.BtnC.isPressed();

                            return SensorMeasurement(value);
                        });

        Sensor* on_board_button_pwr = new Sensor("ON_BOARD_BUTTON_PWR", [&]()
                        {
                            bool value = hardware_low_level_controller_.m5_unified.BtnPWR.isPressed();

                            return SensorMeasurement(value);
                        });

        Sensor* accelerometer = new Sensor("ACCELEROMETER", [&]()
                        {
                            SensorMeasurement::Vector3 values;
                            hardware_low_level_controller_.m5_unified.Imu.getAccel(&values.x, &values.y, &values.z);

                            return SensorMeasurement(values);
                        });

        Sensor* magnetometer = new Sensor("MAGNETOMETER", [&]()
                        {
                            SensorMeasurement::Vector3 values;
                            hardware_low_level_controller_.m5_unified.Imu.getMag(&values.x, &values.y, &values.z);

                            return SensorMeasurement(values);
                        });

        Sensor* gyroscope = new Sensor("GYROSCOPE", [&]()
                        {
                            SensorMeasurement::Vector3 values;
                            hardware_low_level_controller_.m5_unified.Imu.getGyro(&values.x, &values.y, &values.z);

                            return SensorMeasurement(values);
                        });

        Sensor* temperature = new Sensor("TEMPERATURE", [&]()
                        {
                            float value = 0.0;
                            hardware_low_level_controller_.m5_unified.Imu.getTemp(&value);

                            return SensorMeasurement(value);
                        });

        // Initialize aggregated sensors
        Sensor* touch_screen_position = new Sensor("TOUCH_SCREEN", [&]()
                        {
                            m5gfx::M5GFX& display = hardware_low_level_controller_.m5_unified.Display;
                            const int32_t w = display.width();
                            const int32_t h = display.height();

                            SensorMeasurement::Vector3 values;
                            m5::Touch_Class::touch_detail_t touch_detail = hardware_low_level_controller_.m5_unified.Touch.getDetail(0);
                            if (touch_detail.isPressed())
                            {
                                m5gfx::touch_point_t touch_position = hardware_low_level_controller_.m5_unified.Touch.getTouchPointRaw(0);
                                values.x = touch_position.x * 100.0f / w;
                                values.y = touch_position.y * 100.0f / h;
                                values.z = 1;
                            }
                            else
                            {
                                values.x = -1;
                                values.y = -1;
                                values.z = 0;
                            }

                            return SensorMeasurement(values);
                        });

        // Store sensors
        sensors_.push_back(ball_goal_1);
        sensors_.push_back(ball_goal_2);
        sensors_.push_back(ball_goal_3);
        sensors_.push_back(ball_goal_4);
        sensors_.push_back(blue_button_left);
        sensors_.push_back(red_button_left);
        sensors_.push_back(blue_button_right);
        sensors_.push_back(red_button_right);
        sensors_.push_back(on_board_button_a);
        sensors_.push_back(on_board_button_b);
        sensors_.push_back(on_board_button_c);
        sensors_.push_back(on_board_button_pwr);
        sensors_.push_back(accelerometer);
        sensors_.push_back(magnetometer);
        sensors_.push_back(gyroscope);
        sensors_.push_back(temperature);
        sensors_.push_back(touch_screen_position);

        // Initial update
        update();

        std::vector<TaskStepTraceShape::ShapeType>* default_shapes = new std::vector<TaskStepTraceShape::ShapeType>
        {
            TaskStepTraceShape::ShapeType::TRIANGLE,
            TaskStepTraceShape::ShapeType::CIRCLE,
            TaskStepTraceShape::ShapeType::SQUARE
        };

        // Create shape pool
        std::vector<TaskStepTraceShape::ShapeType>* shape_pool = new std::vector<TaskStepTraceShape::ShapeType> {};

        // Create default tasks
        std::vector<const TaskStepBase*>* precondition_steps = new std::vector<const TaskStepBase*>
        {
            new TaskStepTraceShapeSetPool("SET_SHAPE_POOL", shape_pool, default_shapes),
        };

        default_precondition_task_ = new SimultaneousConditionTask(*precondition_steps, "Precondition Task");

        std::vector<const TaskStepBase*>* main_steps = new std::vector<const TaskStepBase*>
        {
            new TaskStepEqual(*get_sensor_by_name("BLUE_BUTTON_LEFT"), SensorMeasurement(true)),
            new TaskStepEqual(*get_sensor_by_name("BLUE_BUTTON_LEFT"), SensorMeasurement(false)),
            new TaskStepEqual(*get_sensor_by_name("BLUE_BUTTON_RIGHT"), SensorMeasurement(true)),
            new TaskStepTraceShapeFromPool(*get_sensor_by_name("TOUCH_SCREEN"), shape_pool),
            new TaskStepTraceShapeFromPool(*get_sensor_by_name("TOUCH_SCREEN"), shape_pool),
            new TaskStepTraceShapeFromPool(*get_sensor_by_name("TOUCH_SCREEN"), shape_pool),
            new TaskStepWaitRandom("WAIT_FOR_BALL_RELEASE"),
        };

        default_task_ = new SequentialTask(*main_steps, "Default Task");
    }

    /**
     * @brief Virtual destructor for cleanup
     */
    ~TaskBoardDriver_v1()
    {
        // TODO(pgarrido): If this is going to be deleted, free all used memory
    }

    /// Virtual method implementation
    Task& get_default_task() override
    {
        return *default_task_;
    }

    /// Virtual method implementation
    Task& get_default_task_precondition() override
    {
        return *default_precondition_task_;
    }

    /// Virtual method implementation
    const std::string& get_unique_id() const override
    {
        return unique_id_;
    }

    /// Virtual method implementation
    void update() override
    {
        hardware_low_level_controller_.m5_unified.update();
        hardware_low_level_controller_.m5_unified.Imu.update();

        for (auto& item : sensors_)
        {
            item->update();
        }
    }

    /// Virtual method implementation
    uint32_t get_sensor_count() const override
    {
        return sensors_.size();
    }

    /// Virtual method implementation
    SensorReader* get_sensor(
            const size_t& index) const override
    {
        Sensor* sensor = nullptr;

        if (index < sensors_.size())
        {
            sensor = sensors_[index];
        }

        return sensor;
    }

    /// Virtual method implementation
    SensorReader* get_sensor_by_name(
            const std::string& sensor_name) const override
    {
        Sensor* sensor = nullptr;

        for (auto const& s : sensors_)
        {
            if (s->name() == sensor_name)
            {
                sensor = s;
                break;
            }
        }

        return sensor;
    }

    /// Virtual method implementation
    const std::string& get_unique_ssid() const override
    {
        return unique_ssid_;
    }

private:

    HardwareLowLevelController& hardware_low_level_controller_;    ///< Reference to hardware interface
    std::vector<Sensor*> sensors_;                                 ///< List of all board sensors
    std::string unique_id_ = "TaskBoard_v1";                       ///< Board identifier
    std::string unique_ssid_ = "Robothon Task Board";              ///< Board identifier

    Task* default_task_;                    ///< Default main task sequence
    Task* default_precondition_task_;       ///< Default precondition task sequence

};
