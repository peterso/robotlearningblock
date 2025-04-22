/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>
#include <task/TaskStep.hpp>
#include <task/TaskStepTraceShape.hpp>

#include <esp_log.h>

/**
 * @struct TaskStepTraceShapeFromPool
 *
 * @brief Implementation of TaskStep that checks how closely a path is followed
 *
 * @details Performs trace score evaluation between measured path and desired path
 */
struct TaskStepTraceShapeFromPool :
    public TaskStep
{
    const char* TAG = "TaskStepTraceShapeFromPool";    ///< Logging tag

    /**
     * @brief Constructs a new TaskStepTraceShapeFromPool object
     *
     * @param sensor Reference to the sensor to monitor
     * @param shape_pool Vector of shapes to select from
     */
    TaskStepTraceShapeFromPool(
            SensorReader& sensor,
            std::vector<TaskStepTraceShape::ShapeType>* shape_pool)
        : TaskStep(sensor),
            shape_pool_(shape_pool)
    {
        TaskStep::type_ = Type::TRACE_SHAPE_FROM_POOL;

        initialize_step();
    }

    void initialize_step() const
    {
        // Select a random shape from the pool
        if (shape_pool_->empty())
        {
            ESP_LOGE(TAG, "Shape pool is empty");
            return;
        }

        ESP_LOGI(TAG, "Shape pool size: %zu", shape_pool_->size());
        
        const size_t random_index = esp_random() % shape_pool_->size();
        shape_ = shape_pool_->at(random_index);
        shape_pool_->erase(shape_pool_->begin() + random_index);
        shape_pool_->shrink_to_fit();

        ESP_LOGI(TAG, "Took shape %d from pool", shape_);
        ESP_LOGI(TAG, "Shape pool size: %zu", shape_pool_->size());

        step_started_ = false;

        // Clear paths
        expected_path_.clear();
        measured_path_.clear();

        // Generate expected path of shape shape_
        switch (shape_) {
            case TaskStepTraceShape::FOUR_SIDED:
                draw_four_sided_shape();
                close_shape();
                break;
            case TaskStepTraceShape::POINT:
            case TaskStepTraceShape::LINE:
                draw_open_shape((uint8_t) shape_);
                break;
            case TaskStepTraceShape::TRIANGLE:
                draw_open_shape(3);
                close_shape();
                break;
            case TaskStepTraceShape::SQUARE:
            case TaskStepTraceShape::CIRCLE:
                draw_regular_shape((uint8_t) shape_);
                close_shape();
                break;
        }
    }

    /**
     * @brief Adds the first point to the end of the vector
     */
    void close_shape() const
    {
        expected_path_.push_back(expected_path_[0]);
    }

    /**
     * @brief Draws an open shape with n_points
     * 
     * @param n_points Number of points (vertices)
     */
    void draw_open_shape(const uint8_t n_points) const
    {
        const float limits[4] = { 5.0f, 95.0f, 30.0f, 90.0f };
        const float min_distance = 30.0f;

        while (expected_path_.size() < n_points)
        {
            // Generate random point within the limits
            SensorMeasurement::Vector3 next_point = SensorMeasurement::Vector3{
                limits[0] + (float) esp_random() * (limits[1] - limits[0]) / UINT32_MAX,
                limits[2] + (float) esp_random() * (limits[3] - limits[2]) / UINT32_MAX,
                0.0f
            };

            // Check if the point is too close to any other point
            bool valid_point = true;
            for (const auto& point : expected_path_)
            {
                const auto last_point = point.get_vector3();
                if (std::hypot(
                        (last_point.x - next_point.x),
                        (last_point.y - next_point.y)) < min_distance)
                {
                    valid_point = false;
                    break;
                }
            }

            if (valid_point)
            {
                expected_path_.push_back(SensorMeasurement(next_point));
            }
        }
    }

    /**
     * @brief Draws a four-sided shape
     * 
     * @details The screen is divided into four sectors: Top left, bottom left, top right, and bottom right.
     *          The four-sided shape has one point in per sector.
     *          The points are not too close to each other.
     */
    void draw_four_sided_shape() const
    {
        const float limits[4][4] = {
            {  5.0f, 50.0f, 30.0f, 60.0f }, // Top left
            {  5.0f, 50.0f, 60.0f, 90.0f }, // Bottom left
            { 50.0f, 95.0f, 60.0f, 90.0f }, // Bottom right
            { 50.0f, 95.0f, 30.0f, 60.0f }  // Top right
        };
        const float min_distance = 30.0f;

        while (expected_path_.size() < 4)
        {
            // Generate random point
            SensorMeasurement::Vector3 next_point = SensorMeasurement::Vector3{
                limits[expected_path_.size()][0] + (float) esp_random() * (limits[expected_path_.size()][1] - limits[expected_path_.size()][0]) / UINT32_MAX,
                limits[expected_path_.size()][2] + (float) esp_random() * (limits[expected_path_.size()][3] - limits[expected_path_.size()][2]) / UINT32_MAX,
                0.0f
            };

            // Check if the point is too close to any other point
            bool valid_point = true;
            for (const auto& point : expected_path_)
            {
                const auto last_point = point.get_vector3();
                if (std::hypot(
                        (last_point.x - next_point.x),
                        (last_point.y - next_point.y)) < min_distance)
                {
                    valid_point = false;
                    break;
                }
            }

            if (valid_point)
            {
                expected_path_.push_back(SensorMeasurement(next_point));
            }
        }

        // Choose a random point to start the shape
        const size_t start_point = esp_random() % expected_path_.size();
        std::rotate(expected_path_.begin(), expected_path_.begin() + start_point, expected_path_.end());
    }            

    /**
     * @brief Draws a regular polygon with n_points
     * 
     * @param n_points Number of points (vertices)
     */
    void draw_regular_shape(const uint8_t n_points) const
    {
        // Random center point
        SensorMeasurement::Vector3 center_point = SensorMeasurement::Vector3{
            40.0f + (float) esp_random() * 20.0f / UINT32_MAX,
            40.0f + (float) esp_random() * 20.0f / UINT32_MAX,
            0.0f
        };
        // Random radius between 20 and 35
        const float radius = 20.0f + (float) esp_random() * 15.0f / UINT32_MAX;
        //Random starting angle
        const float start_angle = (float) esp_random() * 2.0f * M_PI / UINT32_MAX;

        const float increment = 2.0f * M_PI / n_points;
        for (size_t i = 0; i < n_points; i++)
        {
            const float angle = start_angle + i * increment;
            const float x = center_point.x + radius * std::cos(angle);
            const float y = center_point.y + radius * std::sin(angle);
            expected_path_.push_back(SensorMeasurement(SensorMeasurement::Vector3{x, y, 0.0f}));
        }
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
                - (std::max(expected_length, measured_length) > 0.0f ?
                    50.0*std::abs(expected_length - measured_length)/std::max(expected_length, measured_length) :
                    0.0f);

        if (score < 0.0f)
        {
            score = 0.0f;
        }

        initialize_step();
        
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

    mutable TaskStepTraceShape::ShapeType shape_;                                     ///< Shape type to be traced
    mutable bool step_started_ = 0;                             ///< Flag to indicate if the step has started
    mutable std::vector<SensorMeasurement>expected_path_ = {};  ///< Path to be traced
    mutable std::vector<SensorMeasurement>measured_path_ = {};  ///< Path measured

    std::vector<TaskStepTraceShape::ShapeType>* shape_pool_;    ///< Vector of shapes to select from
};
