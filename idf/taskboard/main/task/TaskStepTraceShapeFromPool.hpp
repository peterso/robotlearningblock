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
            std::vector<TraceShapeCommon::ShapeType>* shape_pool)
        : TaskStep(sensor),
            shape_pool_(shape_pool)
    {
        TaskStep::type_ = Type::TRACE_SHAPE_FROM_POOL;
    }

    /// Virtual method implementation
    void initialize_step() const override
    {
        // Select a random shape from the pool
        if (shape_pool_->empty())
        {
            ESP_LOGE(TAG, "Shape pool is empty");
            return;
        }
        
        shape_ = shape_pool_->at(0);
        shape_pool_->erase(shape_pool_->begin());

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

    mutable TraceShapeCommon::ShapeType shape_;                                     ///< Shape type to be traced
    mutable bool step_started_ = 0;                             ///< Flag to indicate if the step has started
    mutable std::vector<SensorMeasurement>expected_path_ = {};  ///< Path to be traced
    mutable std::vector<SensorMeasurement>measured_path_ = {};  ///< Path measured

    std::vector<TraceShapeCommon::ShapeType>* shape_pool_;    ///< Vector of shapes to select from
};
