/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>
#include <task/TaskStep.hpp>

#include <esp_log.h>

/**
 * @struct TaskStepFollowPath
 *
 * @brief Implementation of TaskStep that checks how closely a path is followed
 *
 * @details Performs trace score evaluation between measured path and desired path
 */
struct TaskStepFollowPath :
    public TaskStep
{
    /**
     * @brief Constructs a new TaskStepFollowPath object
     *
     * @param sensor Reference to the sensor to monitor
     */
    TaskStepFollowPath(
            SensorReader& sensor)
        : TaskStep(sensor)
    {
        TaskStep::type_ = Type::FOLLOW_PATH;

        initializeStep();
    }

    void initializeStep() const
    {
        constexpr int32_t N_POINTS = 3;
        const int32_t w = 320;
        const int32_t h = 240;

        step_started_ = false;

        // Generate expected path with N_POINTS random points
        expected_path_.clear();
        while (expected_path_.size() < N_POINTS)
        {
            // Generate random point
            SensorMeasurement::Vector3 next_point = SensorMeasurement::Vector3{
                (float) esp_random() * w / UINT32_MAX,
                (float) esp_random() * h / UINT32_MAX,
                0.0f
            };
            // Check if the point is within acceptable boundaries and distance from the previous point
            // if ...
            expected_path_.push_back(SensorMeasurement(next_point));
            // end if
        }

        // Clear measured path
        measured_path_.clear();
    }

    /// Virtual method implementation
    bool success() const override
    {
        const auto sensor_value = sensor_.read();
        bool pressing_screen = (sensor_value.get_vector3().z == 0);
        if (!step_started_)
        {
            step_started_ = pressing_screen;
        }
        else
        {
            measured_path_.push_back(sensor_value.get_vector3());
            if (!pressing_screen)
            {
                initializeStep();
                return true;
            }
        }
        return false;
    }

    /// Virtual method implementation
    float score() const override
    {
        float score = 0.0f;
        // Implement score function
        // ...
        return score;
    }

    /// Virtual method implementation
    SensorMeasurement expected_value() const override
    {
        return expected_path_[0];
    }

protected:

    /// Virtual method implementation
    void show_clue_implementation(
            ClueScreenController& screen_controller) const override
    {
        if (!success())
        {
            screen_controller.print_task_clue(sensor_.name());
            screen_controller.print_task_clue_path(expected_path_, measured_path_);
        }
        else
        {
            reset_clue();
        }
    }

    mutable bool step_started_ = 0;
    mutable std::vector<SensorMeasurement>expected_path_ = {};
    mutable std::vector<SensorMeasurement>measured_path_ = {};
};
