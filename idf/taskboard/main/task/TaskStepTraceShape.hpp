/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>
#include <task/TaskStep.hpp>

#include <esp_log.h>

/**
 * @struct TraceShapeCommon
 *
 * @brief Helper functions common to TaskStepTraceShape and TaskStepTraceShapeFromPool
 */
struct TraceShapeCommon
{

    enum ShapeType
    {
        FOUR_SIDED  = 0,
        POINT       = 1,
        LINE        = 2,
        TRIANGLE    = 3,
        SQUARE      = 4,
        CIRCLE      = 100
    };
    
    /**
     * @brief Generate expected path of shape shape
     * 
     * @param shape Shape type to be traced
     * @param expected_path Path to be traced
     */
    static void generate_expected_path(
            const ShapeType shape,
            std::vector<SensorMeasurement>& expected_path)
    {
        switch (shape) {
            case FOUR_SIDED:
                draw_four_sided_shape(expected_path);
                close_shape(expected_path);
                break;
            case POINT:
            case LINE:
                draw_open_shape((uint8_t) shape, expected_path);
                break;
            case TRIANGLE:
                draw_open_shape(3, expected_path);
                close_shape(expected_path);
                break;
            case SQUARE:
            case CIRCLE:
                draw_regular_shape((uint8_t) shape, expected_path);
                close_shape(expected_path);
                break;
        }
    }

    /**
     * @brief Draws a regular polygon with n_points
     * 
     * @param n_points Number of points (vertices)
     * @param expected_path Path to be traced
     */
    static void draw_regular_shape(const uint8_t n_points, std::vector<SensorMeasurement>& expected_path)
    {
        // Random center point
        SensorMeasurement::Vector3 center_point = SensorMeasurement::Vector3{
            40.0f + (float) esp_random() * 20.0f / UINT32_MAX,
            40.0f + (float) esp_random() * 20.0f / UINT32_MAX,
            0.0f
        };
        // Random radius between 20 and 35
        const float radius = 20.0f + (float) esp_random() * 15.0f / UINT32_MAX;
        // Random starting angle
        const float start_angle = (float) esp_random() * 2.0f * M_PI / UINT32_MAX;

        const float increment = 2.0f * M_PI / n_points;
        for (size_t i = 0; i < n_points; i++)
        {
            const float angle = start_angle + i * increment;
            const float x = center_point.x + radius * std::cos(angle);
            const float y = center_point.y + radius * std::sin(angle);
            expected_path.push_back(SensorMeasurement(SensorMeasurement::Vector3{x, y, 0.0f}));
        }
    }

    /**
     * @brief Draws an open shape with n_points
     * 
     * @param n_points Number of points (vertices)
     * @param expected_path Path to be traced
     */
    static void draw_open_shape(const uint8_t n_points, std::vector<SensorMeasurement>& expected_path)
    {
        const float limits[4] = { 5.0f, 95.0f, 30.0f, 90.0f };
        const float min_distance = 30.0f;

        while (expected_path.size() < n_points)
        {
            // Generate random point within the limits
            SensorMeasurement::Vector3 next_point = SensorMeasurement::Vector3{
                limits[0] + (float) esp_random() * (limits[1] - limits[0]) / UINT32_MAX,
                limits[2] + (float) esp_random() * (limits[3] - limits[2]) / UINT32_MAX,
                0.0f
            };

            // Check if the point is too close to any other point
            bool valid_point = true;
            for (const auto& point : expected_path)
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
                expected_path.push_back(SensorMeasurement(next_point));
            }
        }
    }

    /**
     * @brief Draws a four-sided shape
     * 
     * @details The screen is divided into four sectors: Top left, bottom left, top right, and bottom right.
     *          The four-sided shape has one point in per sector.
     *          The points are not too close to each other.
     * 
     * @param expected_path Path to be traced
     */
    static void draw_four_sided_shape(std::vector<SensorMeasurement>& expected_path)
    {
        const float limits[4][4] = {
            {  5.0f, 50.0f, 30.0f, 60.0f }, // Top left
            {  5.0f, 50.0f, 60.0f, 90.0f }, // Bottom left
            { 50.0f, 95.0f, 60.0f, 90.0f }, // Bottom right
            { 50.0f, 95.0f, 30.0f, 60.0f }  // Top right
        };
        const float min_distance = 30.0f;

        while (expected_path.size() < 4)
        {
            // Generate random point
            SensorMeasurement::Vector3 next_point = SensorMeasurement::Vector3{
                limits[expected_path.size()][0] + (float) esp_random() * (limits[expected_path.size()][1] - limits[expected_path.size()][0]) / UINT32_MAX,
                limits[expected_path.size()][2] + (float) esp_random() * (limits[expected_path.size()][3] - limits[expected_path.size()][2]) / UINT32_MAX,
                0.0f
            };

            // Check if the point is too close to any other point
            bool valid_point = true;
            for (const auto& point : expected_path)
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
                expected_path.push_back(SensorMeasurement(next_point));
            }
        }

        // Choose a random point to start the shape
        const size_t start_point = esp_random() % expected_path.size();
        std::rotate(expected_path.begin(), expected_path.begin() + start_point, expected_path.end());
    }

    /**
     * @brief Adds the first point to the end of the vector
     * 
     * @param expected_path Path to be traced
     */
    static void close_shape(std::vector<SensorMeasurement>& expected_path)
    {
        expected_path.push_back(expected_path[0]);
    }

    /**
     * @brief Calculates the score
     * 
     * @param expected_path Path to be traced
     * @param measured_path Path measured
     * 
     * @return float Score
     */
    static float calculate_score(
            const std::vector<SensorMeasurement>& expected_path,
            const std::vector<SensorMeasurement>& measured_path)
    {
        // Calculate distance between first point of expected path and first point of measured path
        float first_distance = std::hypot(
                (expected_path[0].get_vector3().x - measured_path[0].get_vector3().x),
                (expected_path[0].get_vector3().y - measured_path[0].get_vector3().y));

        // Calculate distance between last point of expected path and last point of measured path
        float last_distance = std::hypot(
                (expected_path[expected_path.size() - 1].get_vector3().x - measured_path[measured_path.size() - 1].get_vector3().x),
                (expected_path[expected_path.size() - 1].get_vector3().y - measured_path[measured_path.size() - 1].get_vector3().y));

        // Calculate length of expected path
        float expected_length = 0.0f;
        for (size_t i = 1; i < expected_path.size(); i++)
        {
            expected_length += std::hypot(
                    (expected_path[i].get_vector3().x - expected_path[i - 1].get_vector3().x),
                    (expected_path[i].get_vector3().y - expected_path[i - 1].get_vector3().y));
        }

        // Calculate length of measured path
        float measured_length = 0.0f;
        for (size_t i = 1; i < measured_path.size(); i++)
        {
            measured_length += std::hypot(
                    (measured_path[i].get_vector3().x - measured_path[i - 1].get_vector3().x),
                    (measured_path[i].get_vector3().y - measured_path[i - 1].get_vector3().y));
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
        
        return score;
    }
};

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
    const char* TAG = "TaskStepTraceShape";    ///< Logging tag

    /**
     * @brief Constructs a new TaskStepTraceShape object
     *
     * @param sensor Reference to the sensor to monitor
     */
    TaskStepTraceShape(
            SensorReader& sensor,
            TraceShapeCommon::ShapeType shape)
        : TaskStep(sensor),
            shape_(shape)
    {
        TaskStep::type_ = Type::TRACE_SHAPE;
    }

    /// Virtual method implementation
    void initialize_step() const override
    {
        step_started_ = false;

        expected_path_.clear();
        measured_path_.clear();

        TraceShapeCommon::generate_expected_path(shape_, expected_path_);
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
        return TraceShapeCommon::calculate_score(expected_path_, measured_path_);
    }

    /// Virtual method implementation
    SensorMeasurement expected_value() const override
    {
        if (expected_path_.empty())
        {
            return SensorMeasurement();
        }
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

    const TraceShapeCommon::ShapeType shape_;                                     ///< Shape type to be traced
    mutable bool step_started_ = 0;                             ///< Flag to indicate if the step has started
    mutable std::vector<SensorMeasurement>expected_path_ = {};  ///< Path to be traced
    mutable std::vector<SensorMeasurement>measured_path_ = {};  ///< Path measured
};
