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
#include <task/TaskStepFollowPath.hpp>
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
        Sensor* blue_button = new Sensor("BLUE_BUTTON", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_0);

                            return SensorMeasurement(!value); // Button is inverted
                        });

        Sensor* red_button = new Sensor("RED_BUTTON", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller.read_digital_IO1(PbHubController::Channel::
                                    CHANNEL_0);

                            return SensorMeasurement(!value); // Button is inverted
                        });

        Sensor* fader = new AnalogFilteredSensor("FADER", 10, [&]()
                        {
                            uint16_t value =
                            hardware_low_level_controller_.pb_hub_controller.read_analog_IO0(
                                PbHubController::Channel::CHANNEL_5);
                            float float_value = static_cast<float>(value) / 4095.0f;

                            return SensorMeasurement(float_value);
                        });

        Sensor* door_angle = new AnalogFilteredSensor("DOOR_ANGLE", 10, [&]()
                        {
                            uint16_t value =
                            hardware_low_level_controller_.pb_hub_controller.read_analog_IO0(
                                PbHubController::Channel::CHANNEL_4);
                            float float_value = static_cast<float>(value) / 4095.0f;

                            return SensorMeasurement(float_value);
                        });

        Sensor* light_right = new Sensor("LIGHT_RIGHT", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_2);

                            return SensorMeasurement(value);
                        });

        Sensor* light_left = new Sensor("LIGHT_LEFT", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_1);

                            return SensorMeasurement(value);
                        });

        Sensor* probe_goal_analog = new Sensor("PROBE_GOAL_ANALOG", [&]()
                        {
                            uint16_t value =
                            hardware_low_level_controller_.pb_hub_controller.read_analog_IO0(
                                PbHubController::Channel::CHANNEL_3);
                            float float_value = static_cast<float>(value) / 4095.0f;

                            return SensorMeasurement(float_value);
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
        Sensor* door_status = new Sensor("DOOR_OPEN", [=]()
                        {
                            bool is_open = true;

                            if (door_angle->read().get_type() == SensorMeasurement::Type::ANALOG)
                            {
                                if (door_angle->read().get_analog() > 0.7)
                                {
                                    is_open = false;
                                }
                            }

                            return SensorMeasurement(is_open);
                        });

        Sensor* free_cable = new Sensor("FREE_CABLE", [=]()
                        {
                            const bool is_cable_free =
                            !light_right->read().get_boolean() &&
                            !light_left->read().get_boolean();

                            return SensorMeasurement(is_cable_free);
                        });

        Sensor* attached_cable = new Sensor("ATTACHED_CABLE", [=]()
                        {
                            const bool is_cable_attached =
                            light_right->read().get_boolean() &&
                            light_left->read().get_boolean();

                            return SensorMeasurement(is_cable_attached);
                        });

        Sensor* probe_goal = new Sensor("PROBE_GOAL", [=]()
                        {
                            auto analog_measure = probe_goal_analog->read();

                            const bool is_touching_goal = analog_measure.get_analog() < 0.1;

                            return SensorMeasurement(is_touching_goal);
                        });

        Sensor* fader_trigger_blue_button = new TriggeredSensor("FADER_BLUE_BUTTON", *blue_button, [=]()
                        {
                            return fader->read();
                        });

        Sensor* red_button_counter = new CounterSensor("RED_BUTTON_COUNTER", [=]()
                        {
                            return red_button->read();
                        });

        Sensor* touch_screen_position = new Sensor("TOUCH_SCREEN_POSITION", [&]()
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
        sensors_.push_back(blue_button);
        sensors_.push_back(red_button);
        sensors_.push_back(fader);
        sensors_.push_back(door_angle);
        sensors_.push_back(light_right);
        sensors_.push_back(light_left);
        sensors_.push_back(probe_goal_analog);
        sensors_.push_back(on_board_button_a);
        sensors_.push_back(on_board_button_b);
        sensors_.push_back(on_board_button_c);
        sensors_.push_back(on_board_button_pwr);
        sensors_.push_back(accelerometer);
        sensors_.push_back(magnetometer);
        sensors_.push_back(gyroscope);
        sensors_.push_back(temperature);
        sensors_.push_back(door_status);
        sensors_.push_back(free_cable);
        sensors_.push_back(attached_cable);
        sensors_.push_back(probe_goal);
        sensors_.push_back(fader_trigger_blue_button);
        sensors_.push_back(red_button_counter);
        sensors_.push_back(touch_screen_position);

        // Initial update
        update();

        // Create default tasks
        std::vector<const TaskStep*>* precondition_steps = new std::vector<const TaskStep*>
        {
            new TaskStepEqual(*get_sensor_by_name("FADER"), SensorMeasurement(0.0f), 0.1f),
            new TaskStepEqual(*get_sensor_by_name("DOOR_OPEN"), SensorMeasurement(false)),
            new TaskStepEqual(*get_sensor_by_name("PROBE_GOAL"), SensorMeasurement(false)),
            new TaskStepEqual(*get_sensor_by_name("FREE_CABLE"), SensorMeasurement(true)),
        };

        default_precondition_task_ = new SimultaneousConditionTask(*precondition_steps, "Precondition Task");

        TaskStep* timed_fader_operation =
                new TaskStepEqual(*get_sensor_by_name("FADER"), SensorMeasurement(0.5f), 0.05f);
        // timed_fader_operation->set_clue_timeout(*get_sensor_by_name("BLUE_BUTTON"), 3000);

        TaskStep* random_fader_step =
                new TaskStepEqualToRandom(*get_sensor_by_name("FADER"), 0.05f);
        // random_fader_step->set_clue_timeout(*get_sensor_by_name("BLUE_BUTTON"), 3000);

        std::vector<const TaskStep*>* main_steps = new std::vector<const TaskStep*>
        {
            new TaskStepEqual(*get_sensor_by_name("BLUE_BUTTON"), SensorMeasurement(true)),
            timed_fader_operation,
            new TaskStepFollowPath(*get_sensor_by_name("TOUCH_SCREEN_POSITION")),
            random_fader_step,
            new TaskStepEqual(*get_sensor_by_name("FADER_BLUE_BUTTON"), SensorMeasurement(0.2f), 0.05f),
            new TaskStepEqual(*get_sensor_by_name("DOOR_OPEN"), SensorMeasurement(true)),
            new TaskStepEqual(*get_sensor_by_name("PROBE_GOAL"), SensorMeasurement(true)),
            new TaskStepEqual(*get_sensor_by_name("ATTACHED_CABLE"), SensorMeasurement(true)),
            new TaskStepEqual(*get_sensor_by_name("RED_BUTTON_COUNTER"), SensorMeasurement(3ll)),
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

        // Handle floating values at PbHubController
        // TODO(pgarrido): Handle this with Peter
        hardware_low_level_controller_.pb_hub_controller.write_digital_IO0(PbHubController::Channel::CHANNEL_3, true);
        hardware_low_level_controller_.pb_hub_controller.write_digital_IO1(PbHubController::Channel::CHANNEL_3, true);

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
