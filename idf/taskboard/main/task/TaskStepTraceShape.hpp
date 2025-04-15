/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>
#include <task/TaskStep.hpp>

#include <esp_log.h>

/**
 * @struct TaskStepTraceShape
 *
 * @brief Implementation of TaskStep that checks how closely a path is followed
 *
 * @details Performs trace score evaluation between measured path and desired path
 */
struct TaskStepTraceShape :
    public TaskStep
{
    /**
     * @brief Constructs a new TaskStepTraceShape object
     *
     * @param sensor Reference to the sensor to monitor
     */
    TaskStepTraceShape(
            SensorReader& sensor)
        : TaskStep(sensor)
    {
        TaskStep::type_ = Type::TRACE_SHAPE;

        initializeStep();
    }

    void initializeStep() const
    {
        constexpr int32_t N_POINTS = 3;

        step_started_ = false;

        // Generate expected path with N_POINTS random points
        expected_path_.clear();
        while (expected_path_.size() < N_POINTS)
        {
            // Generate random point
            SensorMeasurement::Vector3 next_point = SensorMeasurement::Vector3{
                (float) esp_random() * 100.0f / UINT32_MAX,
                (float) esp_random() * 100.0f / UINT32_MAX,
                0.0f
            };
            // Check if the point is too close to the edges of the screen
            if (next_point.x < 5.0f || next_point.x > 95.0f ||
                next_point.y < 30.0f || next_point.y > 90.0f)
            {
                // Do not add
                continue;
            }
            // Check if the point is too close to the last point
            if (!expected_path_.empty())
            {
                const auto last_point = expected_path_.back().get_vector3();
                if (std::hypot(
                        (last_point.x - next_point.x),
                        (last_point.y - next_point.y)) < 30.0f)
                {
                    // Do not add
                    continue;
                }
            }
            expected_path_.push_back(SensorMeasurement(next_point));
        }

        // Clear measured path
        measured_path_.clear();
    }

    /// Virtual method implementation
    bool success() const override
    {
        const auto sensor_value = sensor_.read();
        bool pressing_screen = (sensor_value.get_vector3().z != 0);
        if (!step_started_)
        {
            step_started_ = pressing_screen;
        }
        else
        {
            if (pressing_screen)
            {
                measured_path_.push_back(sensor_value.get_vector3());
            }
            else
            {
                return true;
            }
        }
        return false;
    }

    /// Virtual method implementation
    float score() const override
    {
        // Calculate distance between first point of expected path and first point of measured path
        float first_distance = std::hypot(
                (expected_path_[0].get_vector3().x - measured_path_[0].get_vector3().x),
                (expected_path_[0].get_vector3().y - measured_path_[0].get_vector3().y));

        // Calculate distance between last point of expected path and last point of measured path
        float last_distance = std::hypot(
                (expected_path_[expected_path_.size() - 1].get_vector3().x - measured_path_[measured_path_.size() - 1].get_vector3().x),
                (expected_path_[expected_path_.size() - 1].get_vector3().y - measured_path_[measured_path_.size() - 1].get_vector3().y));

        // Calculate length of expected path
        float expected_length = 0.0f;
        for (size_t i = 1; i < expected_path_.size(); i++)
        {
            expected_length += std::hypot(
                    (expected_path_[i].get_vector3().x - expected_path_[i - 1].get_vector3().x),
                    (expected_path_[i].get_vector3().y - expected_path_[i - 1].get_vector3().y));
        }

        // Calculate length of measured path
        float measured_length = 0.0f;
        for (size_t i = 1; i < measured_path_.size(); i++)
        {
            measured_length += std::hypot(
                    (measured_path_[i].get_vector3().x - measured_path_[i - 1].get_vector3().x),
                    (measured_path_[i].get_vector3().y - measured_path_[i - 1].get_vector3().y));
        }

        // Calculate score based on distances and difference of lenghts
        float score = 100.0f
                - 50.0*first_distance/141.42f
                - 50.0*last_distance/141.42f
                - 50.0*std::abs(expected_length - measured_length)/std::max(expected_length, measured_length);

        if (score < 0.0f)
        {
            score = 0.0f;
        }

        initializeStep();
        
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
