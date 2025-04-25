/**
 * Robothon Task Board Firmware
 TEST PETER WAS HERE
 */

#pragma once

#include "driver/gpio.h"

#include <hal/TaskBoardDriver.hpp>
#include <hal/HardwareLowLevelController.hpp>
#include <sensor/Sensor.hpp>
#include <sensor/AnalogFilteredSensor.hpp>
#include <sensor/CounterSensor.hpp>
#include <sensor/TriggeredSensor.hpp>
#include <task/TaskStepActuator.hpp>
#include <task/TaskStepEqual.hpp>
#include <task/TaskStepEqualDuringRandom.hpp>
#include <task/TaskStepEqualToRandom.hpp>
#include <task/TaskStepTouchGoalFromPool.hpp>
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

    static constexpr uint8_t PAHUB_CHANNEL_FOR_PBHUB1 = 0;
    static constexpr uint8_t PAHUB_CHANNEL_FOR_PBHUB2 = 1;

    /**
     * @brief Constructs a new TaskBoardDriver_v1 object
     *
     * @param hardware_low_level_controller Reference to hardware interface
     */
    TaskBoardDriver_v1(
            m5::M5Unified& m5_unified)
        : pa_hub_controller_(new PaHubController()),
          pb_hub_controller_1_(new PaHubToPbHubController(*pa_hub_controller_, PAHUB_CHANNEL_FOR_PBHUB1)),
          pb_hub_controller_2_(new PaHubToPbHubController(*pa_hub_controller_, PAHUB_CHANNEL_FOR_PBHUB2)),
          hardware_low_level_controller_(*pb_hub_controller_1_, *pb_hub_controller_2_, m5_unified)
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

        // Wait for PbHubControllers to initialize
        // This can take variable time after boot
        while (!hardware_low_level_controller_.pb_hub_controller_1.check_status())
        {
            ESP_LOGI("app_main", "Waiting for PbHubController 1 to start");
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        while (!hardware_low_level_controller_.pb_hub_controller_2.check_status())
        {
            ESP_LOGI("app_main", "Waiting for PbHubController 2 to start");
            vTaskDelay(pdMS_TO_TICKS(10));
        }

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

        // Initialize actuators
        Actuator* goal_1_led = new Actuator("GOAL_1_LED", [&](Actuator::State state)
                        {
                            hardware_low_level_controller_.pb_hub_controller_2.write_digital_IO0(PbHubController::Channel::CHANNEL_0, state == Actuator::State::ON);
                        });
        Actuator* goal_2_led = new Actuator("GOAL_2_LED", [&](Actuator::State state)
                        {
                            hardware_low_level_controller_.pb_hub_controller_2.write_digital_IO0(PbHubController::Channel::CHANNEL_1, state == Actuator::State::ON);
                        });
        Actuator* goal_3_led = new Actuator("GOAL_3_LED", [&](Actuator::State state)
                        {
                            hardware_low_level_controller_.pb_hub_controller_2.write_digital_IO0(PbHubController::Channel::CHANNEL_2, state == Actuator::State::ON);
                        });
        Actuator* goal_4_led = new Actuator("GOAL_4_LED", [&](Actuator::State state)
                        {
                            hardware_low_level_controller_.pb_hub_controller_2.write_digital_IO0(PbHubController::Channel::CHANNEL_3, state == Actuator::State::ON);
                        });

        Actuator* ball_drop_solenoid = new Actuator("BALL_DROP_SOLENOID", [&](Actuator::State state)
                        {
                            gpio_set_level(GPIO_NUM_27, state == Actuator::State::ON);
                        });
        Actuator* all_goal_leds = new Actuator("ALL_GOAL_LEDS", [&](Actuator::State state)
                        {
                            hardware_low_level_controller_.pb_hub_controller_2.write_digital_IO0(PbHubController::Channel::CHANNEL_0, state == Actuator::State::ON);
                            hardware_low_level_controller_.pb_hub_controller_2.write_digital_IO0(PbHubController::Channel::CHANNEL_1, state == Actuator::State::ON);
                            hardware_low_level_controller_.pb_hub_controller_2.write_digital_IO0(PbHubController::Channel::CHANNEL_2, state == Actuator::State::ON);
                            hardware_low_level_controller_.pb_hub_controller_2.write_digital_IO0(PbHubController::Channel::CHANNEL_3, state == Actuator::State::ON);
                        });

        actuators_.push_back(goal_1_led);
        actuators_.push_back(goal_2_led);
        actuators_.push_back(goal_3_led);
        actuators_.push_back(goal_4_led);
        actuators_.push_back(ball_drop_solenoid);
        actuators_.push_back(all_goal_leds);

        // Initial update
        update();

        std::vector<TraceShapeCommon::ShapeType>* default_shapes = new std::vector<TraceShapeCommon::ShapeType>
        {
            TraceShapeCommon::ShapeType::TRIANGLE,
            TraceShapeCommon::ShapeType::CIRCLE,
            TraceShapeCommon::ShapeType::SQUARE
        };

        // Create shape pool
        std::vector<TraceShapeCommon::ShapeType>* shape_pool = new std::vector<TraceShapeCommon::ShapeType> {};

        // Create touch goal pool
        std::vector<TaskStepTouchGoalFromPool::TouchGoalType>* default_touch_goals = new std::vector<TaskStepTouchGoalFromPool::TouchGoalType>
        {
            TaskStepTouchGoalFromPool::TouchGoalType::TapAGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::TapBGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::DoubleTapAGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::DoubleTapBGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::LongPressAGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::LongPressBackgroundGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::LongPressBGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::TapBackgroundGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::DoubleTapBackgroundGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::DragFromAtoBGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::DragFromBtoAGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::DragFromAtoBackgroundGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::DragFromBtoBackgroundGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::DragFromBackgroundtoAGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::DragFromBackgroundtoBGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::SwipeUpGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::SwipeDownGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::SwipeLeftGoal,
            TaskStepTouchGoalFromPool::TouchGoalType::SwipeRightGoal
        };

        std::vector<TaskStepTouchGoalFromPool::TouchGoalType>* touch_goal_pool = new std::vector<TaskStepTouchGoalFromPool::TouchGoalType> {};

        // Create default tasks
        std::vector<const TaskStepBase*>* precondition_steps = new std::vector<const TaskStepBase*>
        {
            new TaskStepTraceShapeSetPool("SET_SHAPE_POOL", shape_pool, default_shapes),
            new TaskStepTouchGoalSetPool("SET_TOUCH_GOAL_POOL", touch_goal_pool, default_touch_goals),
            new TaskStepActuator(*all_goal_leds, Actuator::State::OFF),
            new TaskStepActuator(*ball_drop_solenoid, Actuator::State::OFF),
            new TaskStepEqual(*get_sensor_by_name("BALL_GOAL_1"), SensorMeasurement(true)),
            new TaskStepEqual(*get_sensor_by_name("BLUE_BUTTON_RIGHT"), SensorMeasurement(true)),
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
            new TaskStepTouchGoalFromPool(*get_sensor_by_name("TOUCH_SCREEN"), touch_goal_pool),
            new TaskStepTouchGoalFromPool(*get_sensor_by_name("TOUCH_SCREEN"), touch_goal_pool),
            new TaskStepTouchGoalFromPool(*get_sensor_by_name("TOUCH_SCREEN"), touch_goal_pool),
            new TaskStepTouchGoalFromPool(*get_sensor_by_name("TOUCH_SCREEN"), touch_goal_pool),
            new TaskStepTouchGoalFromPool(*get_sensor_by_name("TOUCH_SCREEN"), touch_goal_pool),
            new TaskStepEqual(*get_sensor_by_name("BALL_GOAL_2"), SensorMeasurement(true)),
            new TaskStepActuator(*ball_drop_solenoid, Actuator::State::ON),
            new TaskStepEqualDuringRandom(*get_sensor_by_name("BLUE_BUTTON_LEFT"), SensorMeasurement(true), 0.0, 3000L, 8000L),
            new TaskStepActuator(*ball_drop_solenoid, Actuator::State::OFF),
            new TaskStepActuator(*all_goal_leds, Actuator::State::ON),
            new TaskStepEqual(*get_sensor_by_name("BALL_GOAL_1"), SensorMeasurement(true)),
            new TaskStepActuator(*goal_1_led, Actuator::State::OFF),
            new TaskStepEqual(*get_sensor_by_name("BALL_GOAL_3"), SensorMeasurement(true)),
            new TaskStepActuator(*goal_3_led, Actuator::State::OFF),
            new TaskStepEqual(*get_sensor_by_name("BALL_GOAL_4"), SensorMeasurement(true)),
            new TaskStepActuator(*goal_4_led, Actuator::State::OFF),
            new TaskStepEqual(*get_sensor_by_name("BALL_GOAL_2"), SensorMeasurement(true)),
            new TaskStepActuator(*goal_2_led, Actuator::State::OFF),
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

    /// Virtual method implementation
    HardwareLowLevelController& get_hardware_low_level_controller() override
    {
        return hardware_low_level_controller_;
    }

private:

    PaHubController* pa_hub_controller_;    ///< Reference to PaHub controller
    PaHubToPbHubController* pb_hub_controller_1_;    ///< Reference to PbHub controller 1
    PaHubToPbHubController* pb_hub_controller_2_;    ///< Reference to PbHub controller 2
    HardwareLowLevelController hardware_low_level_controller_;    ///< Reference to hardware interface
    std::vector<Sensor*> sensors_;                                 ///< List of all board sensors
    std::vector<Actuator*> actuators_;                             ///< List of all board actuators
    std::string unique_id_ = "TaskBoard_v1";                       ///< Board identifier
    std::string unique_ssid_ = "Robothon Task Board";              ///< Board identifier

    Task* default_task_;                    ///< Default main task sequence
    Task* default_precondition_task_;       ///< Default precondition task sequence

};
