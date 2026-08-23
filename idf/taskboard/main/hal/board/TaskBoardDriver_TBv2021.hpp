/**
 * Robothon Task Board Firmware
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
            m5::M5Unified& m5_unified)
        : pb_hub_controller_(new PbHubController()),
          hardware_low_level_controller_(*pb_hub_controller_, m5_unified)
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

        // Wait for PbHubController to initialize
        // This can take variable time after boot
        while (!hardware_low_level_controller_.pb_hub_controller.check_status())
        {
            ESP_LOGI("app_main", "Waiting for PbHubController to start");
            vTaskDelay(pdMS_TO_TICKS(10));
        }

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

        Sensor* batt_blue_button = new Sensor("BATT_BLUE_BUTTON", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_1);

                            return SensorMeasurement(!value); // Button is inverted
                        });

        Sensor* batt_red_button = new Sensor("BATT_RED_BUTTON", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller.read_digital_IO1(PbHubController::Channel::
                                    CHANNEL_1);

                            return SensorMeasurement(!value); // Button is inverted
                        });

        Sensor* key_switched_right = new Sensor("KEY_SWITCHED_RIGHT", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_2);

                            return SensorMeasurement(!value); // value is inverted
                        });

        Sensor* key_switched_left = new Sensor("KEY_SWITCHED_LEFT", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller.read_digital_IO1(PbHubController::Channel::
                                    CHANNEL_2);

                            return SensorMeasurement(!value); // value is inverted
                        });

        Sensor* usb_inserted_near = new Sensor("USB_INSERTED_NEAR", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller.read_digital_IO0(PbHubController::Channel::
                                    CHANNEL_3);

                            return SensorMeasurement(!value); // value is inverted
                        });

        Sensor* usb_inserted_far = new Sensor("USB_INSERTED_FAR", [&]()
                        {
                            bool value =
                            hardware_low_level_controller_.pb_hub_controller.read_digital_IO1(PbHubController::Channel::
                                    CHANNEL_3);

                            return SensorMeasurement(!value); // value is inverted
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
       
        Sensor* blue_button_counter = new CounterSensor("BLUE_BUTTON_COUNTER", [=]()
                        {
                            return blue_button->read();
                        });

        Sensor* red_button_counter = new CounterSensor("RED_BUTTON_COUNTER", [=]()
                        {
                            return red_button->read();
                        });

        Sensor* key_switched_left_counter = new CounterSensor("KEY_SWITCHED_LEFT", [=]()
                        {
                            return key_switched_left->read();
                        });

        Sensor* key_switched_right_counter = new CounterSensor("KEY_SWITCHED_RIGHT", [=]()
                        {
                            return key_switched_right->read();
                        });

        Sensor* usb_inserted_near_counter = new CounterSensor("USB_INSERTED_NEAR", [=]()
                        {
                            return usb_inserted_near->read();
                        });

        Sensor* usb_inserted_far_counter = new CounterSensor("USB_INSERTED_FAR", [=]()
                        {
                            return usb_inserted_far->read();
                        });                        

        Sensor* batt_blue_button_counter = new CounterSensor("BATT_BLUE_BUTTON_COUNTER", [=]()
                        {
                            return batt_blue_button->read();
                        });

        Sensor* batt_red_button_counter = new CounterSensor("BATT_RED_BUTTON_COUNTER", [=]()
                        {
                            return batt_red_button->read();
                        });

        Sensor* trial_start_btn_counter = new CounterSensor("TRIAL_START_BTN_COUNTER", [=]()
                        {
                            return on_board_button_b->read();
                        });

        // Store sensors
        sensors_.push_back(blue_button);
        sensors_.push_back(red_button);
        sensors_.push_back(batt_blue_button);
        sensors_.push_back(batt_red_button);
        sensors_.push_back(key_switched_left);
        sensors_.push_back(key_switched_right);
        sensors_.push_back(usb_inserted_near);
        sensors_.push_back(usb_inserted_far);
        sensors_.push_back(on_board_button_a);
        sensors_.push_back(on_board_button_b);
        sensors_.push_back(on_board_button_c);
        sensors_.push_back(on_board_button_pwr);
        sensors_.push_back(accelerometer);
        sensors_.push_back(magnetometer);
        sensors_.push_back(gyroscope);
        sensors_.push_back(temperature);

        sensors_.push_back(blue_button_counter);
        sensors_.push_back(red_button_counter);
        sensors_.push_back(key_switched_left_counter);
        sensors_.push_back(key_switched_right_counter);
        sensors_.push_back(usb_inserted_near_counter);
        sensors_.push_back(usb_inserted_far_counter);
        sensors_.push_back(batt_blue_button_counter);
        sensors_.push_back(batt_red_button_counter);
        sensors_.push_back(trial_start_btn_counter);

        // Initial update
        update();

        // Create default tasks
        std::vector<const TaskStepBase*>* precondition_steps = new std::vector<const TaskStepBase*>
        {
            new TaskStepEqual(*get_sensor_by_name("KEY_SWITCHED_LEFT"), SensorMeasurement(false)),
            new TaskStepEqual(*get_sensor_by_name("KEY_SWITCHED_RIGHT"), SensorMeasurement(false)),
            new TaskStepEqual(*get_sensor_by_name("USB_INSERTED_NEAR"), SensorMeasurement(true)),
            new TaskStepEqual(*get_sensor_by_name("USB_INSERTED_FAR"), SensorMeasurement(false)),
        };

        default_precondition_task_ = new SimultaneousConditionTask(*precondition_steps, "Precondition Task");

       
        std::vector<const TaskStepBase*>* main_steps = new std::vector<const TaskStepBase*>
        {
            new TaskStepEqual(*get_sensor_by_name("BLUE_BUTTON"), SensorMeasurement(true)),
            new TaskStepEqual(*get_sensor_by_name("USB_INSERTED_NEAR"), SensorMeasurement(false)),
            new TaskStepEqual(*get_sensor_by_name("USB_INSERTED_FAR"), SensorMeasurement(true)),
            new TaskStepEqual(*get_sensor_by_name("KEY_SWITCHED_RIGHT"), SensorMeasurement(true)),
            new TaskStepEqual(*get_sensor_by_name("BATT_RED_BUTTON"), SensorMeasurement(true)),
            new TaskStepEqual(*get_sensor_by_name("BATT_BLUE_BUTTON"), SensorMeasurement(true)),
            new TaskStepEqual(*get_sensor_by_name("RED_BUTTON"), SensorMeasurement(true)),
        };

        default_task_ = new SequentialTask(*main_steps, "RGC2021 Protocol");
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
        // hardware_low_level_controller_.pb_hub_controller.write_digital_IO0(PbHubController::Channel::CHANNEL_3, true);
        // hardware_low_level_controller_.pb_hub_controller.write_digital_IO1(PbHubController::Channel::CHANNEL_3, true);

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

    /// Virtual method implementation
    HardwareLowLevelController& get_hardware_low_level_controller() override
    {
        return hardware_low_level_controller_;
    }

private:

    PbHubController* pb_hub_controller_;    ///< Pointer to the PbHubController instance
    HardwareLowLevelController hardware_low_level_controller_;    ///< Reference to hardware interface
    std::vector<Sensor*> sensors_;                                 ///< List of all board sensors
    std::string unique_id_ = "TaskBoard_v1";                       ///< Board identifier
    std::string unique_ssid_ = "Robothon Task Board";              ///< Board identifier

    Task* default_task_;                    ///< Default main task sequence
    Task* default_precondition_task_;       ///< Default precondition task sequence

};
