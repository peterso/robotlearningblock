/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>
#include <task/TaskStep.hpp>
#include <task/TaskStepTraceShape.hpp>

#include <esp_log.h>

/**
 * @struct TaskStepTraceShapeSetPool
 * 
 * @brief Implementation of TaskStep that empties a shape pool and fills it with a vector of shapes
 */
struct TaskStepTraceShapeSetPool :
    public TaskStepAuto
{
    const char* TAG = "TaskStepTraceShapeSetPool";    ///< Logging tag

    /**
     * @brief Constructs a new TaskStepTraceShapeSetPool object
     *
     * @param name Name identifier for the task step
     * @param shape_pool Vector of shapes to be filled
     * @param shapes_vector Vector of shapes to fill the pool with
     */
    TaskStepTraceShapeSetPool(
            std::string name,
            std::vector<TaskStepTraceShape::ShapeType>* shape_pool,
            std::vector<TaskStepTraceShape::ShapeType>* shapes_vector)
        : TaskStepAuto(name)
        , shape_pool_(shape_pool)
        , shapes_vector_(shapes_vector)
    {
        TaskStepAuto::type_ = Type::TRACE_SHAPE_MANAGE_POOL;

        initialize_step();
    }

    void initialize_step() const
    {
        shape_pool_->clear();
        shape_pool_->shrink_to_fit();

        shape_pool_->insert(shape_pool_->end(), shapes_vector_->begin(), shapes_vector_->end());
        ESP_LOGI(TAG, "Shape pool filled. Cointains %zu shapes", shape_pool_->size());
    }

    /// Virtual method implementation
    bool success() const override
    {
        return true;
    }

    /// Virtual method implementation
    float score() const override
    {
        initialize_step();
        return -1.0f;
    }

private:

    /// Virtual method implementation
    void show_clue_implementation(
            ClueScreenController& screen_controller) const override
    {
        screen_controller.print_task_clue("Shape pool filled");
    }

    std::vector<TaskStepTraceShape::ShapeType>* shape_pool_;    ///< Vector of shapes to be filled
    const std::vector<TaskStepTraceShape::ShapeType>* shapes_vector_;    ///< Vector of shapes to fill the pool with
};

/**
 * @struct TaskStepTraceShapeFillPool
 * 
 * @brief Implementation of TaskStep that fills a pool with a vector of shapes
 */
struct TaskStepTraceShapeFillPool :
    public TaskStepAuto
{
    const char* TAG = "TaskStepTraceShapeFillPool";    ///< Logging tag

    /**
     * @brief Constructs a new TaskStepTraceShapeFillPool object
     *
     * @param name Name identifier for the task step
     * @param shape_pool Vector of shapes to be filled
     * @param shapes_vector Vector of shapes to fill the pool with
     */
    TaskStepTraceShapeFillPool(
            std::string name,
            std::vector<TaskStepTraceShape::ShapeType>* shape_pool,
            std::vector<TaskStepTraceShape::ShapeType>* shapes_vector)
        : TaskStepAuto(name)
        , shape_pool_(shape_pool)
        , shapes_vector_(shapes_vector)
    {
        TaskStepAuto::type_ = Type::TRACE_SHAPE_MANAGE_POOL;

        initialize_step();
    }

    void initialize_step() const
    {
        shape_pool_->insert(shape_pool_->end(), shapes_vector_->begin(), shapes_vector_->end());
        ESP_LOGI(TAG, "Shape pool filled. Cointains %zu shapes", shape_pool_->size());
    }

    /// Virtual method implementation
    bool success() const override
    {
        return true;
    }

    /// Virtual method implementation
    float score() const override
    {
        initialize_step();
        return -1.0f;
    }

private:

    /// Virtual method implementation
    void show_clue_implementation(
            ClueScreenController& screen_controller) const override
    {
        screen_controller.print_task_clue("Shape pool filled");
    }

    std::vector<TaskStepTraceShape::ShapeType>* shape_pool_;    ///< Vector of shapes to be filled
    const std::vector<TaskStepTraceShape::ShapeType>* shapes_vector_;    ///< Vector of shapes to fill the pool with
};

/**
 * @brief TaskStepTraceShapeEmptyPool
 * 
 * @details Implementation of TaskStep that empties a shape pool
 */
struct TaskStepTraceShapeEmptyPool :
    public TaskStepAuto
{
    const char* TAG = "TaskStepTraceShapeEmptyPool";    ///< Logging tag

    /**
     * @brief Constructs a new TaskStepTraceShapeEmptyPool object
     *
     * @param name Name identifier for the task step
     * @param shape_pool Vector of shapes to be emptied
     */
    TaskStepTraceShapeEmptyPool(
            std::string name,
            std::vector<TaskStepTraceShape::ShapeType>* shape_pool)
        : TaskStepAuto(name)
        , shape_pool_(shape_pool)
    {
        TaskStepAuto::type_ = Type::TRACE_SHAPE_MANAGE_POOL;

        initialize_step();
    }

    void initialize_step() const
    {
        shape_pool_->clear();
        shape_pool_->shrink_to_fit();
        ESP_LOGI(TAG, "Shape pool emptied. Cointains %zu shapes", shape_pool_->size());
    }

    /// Virtual method implementation
    bool success() const override
    {
        return true;
    }

    /// Virtual method implementation
    float score() const override
    {
        initialize_step();
        return -1.0f;
    }

private:

    /// Virtual method implementation
    void show_clue_implementation(
            ClueScreenController& screen_controller) const override
    {
        screen_controller.print_task_clue("Shape pool emptied");
    }

    std::vector<TaskStepTraceShape::ShapeType>* shape_pool_;    ///< Vector of shapes to be emptied
};