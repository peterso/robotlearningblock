/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>
#include <hal/ClueScreenController.hpp>

#include <esp_log.h>
#include <esp_timer.h>

/**
 * @struct TaskStepClueHandler
 *
 * @brief Base class for task step handlers that show clues on the screen controller
 */
struct TaskStepClueHandler
{
    const char* TAG = "TaskStepClueHandler";    ///< Logging tag

    /**
     * @brief Sets a clue timeout and trigger
     *
     * @param clue_trigger Sensor that triggers the clue
     * @param timeout_ms Timeout for showing the clue
     */
    void set_clue_timeout(
            const SensorReader& clue_trigger,
            uint64_t timeout_ms)
    {
        clue_trigger_ = &clue_trigger;
        timeout_ = timeout_ms * 1000;
    }

    /**
     * @brief Show clue on the screen controller handler
     *
     * @param screen_controller Reference to the screen controller
     */
    void show_clue(
            ClueScreenController& screen_controller) const
    {
        // If no trigger is set, show the clue immediately
        if (nullptr == clue_trigger_)
        {
            show_clue_implementation(screen_controller);
        }
        else
        {
            const auto trigger_value = clue_trigger_->read();

            if (trigger_value.get_type() != SensorMeasurement::Type::BOOLEAN)
            {
                ESP_LOGE(TAG, "Clue trigger sensor must be a boolean sensor");

                return;
            }

            if (first_time_)
            {
                clue_time_start_ = esp_timer_get_time();
                first_time_ = false;
            }

            if (trigger_value.get_boolean() || (esp_timer_get_time() - clue_time_start_) < timeout_)
            {
                show_clue_implementation(screen_controller);
            }
            else
            {
                screen_controller.clear_all_task_clue();
            }

            // Relaunch the clue on trigger
            if (trigger_value.get_boolean())
            {
                clue_time_start_ = esp_timer_get_time();
            }
        }
    }

    const SensorReader* clue_trigger() const
    {
        return clue_trigger_;
    }

    uint64_t clue_timeout_us() const
    {
        return timeout_;
    }

protected:

    /**
     * @brief Resets the clue trigger
     *
     * @return true if the clue is being shown for the first time
     */
    void reset_clue() const
    {
        first_time_ = true;
    }

    /**
     * @brief Actual implementation of the clue showing
     *
     * @param screen_controller Reference to the screen controller
     */
    virtual void show_clue_implementation(
            ClueScreenController& screen_controller) const = 0;

private:

    const SensorReader* clue_trigger_ = nullptr;    ///< Sensor that triggers the clue
    uint64_t timeout_ = 0;                          ///< Timeout for showing the clue
    mutable int64_t clue_time_start_ = 0;           ///< Time when the clue was shown
    mutable bool first_time_ = true;                ///< Flag for first clue showing
};

/**
 * @struct TaskStepBase
 *
 * @brief Base class for task step handlers
 *
 * @details This class provides the interface for checking task step success and calculating scores.
 */
struct TaskStepBase :
    public TaskStepClueHandler
{
    /**
     * @brief Virtual destructor
     */
    virtual ~TaskStepBase() = default;

    /**
     * @enum Type
     *
     * @brief Defines the comparison types for sensor value evaluation
     */
    enum class Type
    {
        EQUAL,              ///< Sensor value must exactly match target
        EQUAL_TO_RANDOM,    ///< Sensor value must match a randomly generated target
        GREATER_OR_EQUAL,   ///< Sensor value must be greater or equal to target
        TRACE_SHAPE,        ///< Sensor value must follow a specific path
        WAIT_RANDOM,        ///< Wait for a random time
        UNKNOWN             ///< Undefined comparison type
    };

    /**
     * @brief restarts the step to the condition needed at the beginning of the task
     */
    virtual void restart_step() const = 0;

    /**
     * @brief Gets the name of the task step
     *
     * @return Reference to task step name string
     */
    virtual const std::string& name() const = 0;

    /**
     * @brief Gets the target value for this step
     *
     * @return Expected sensor measurement value
     */
    virtual SensorMeasurement expected_value() const = 0;

    /**
     * @brief Checks if the step condition has been met
     *
     * @return true if condition is satisfied, false otherwise
     */
    virtual bool success() const = 0;

    /**
     * @brief Calculates the score for this step
     *
     * @details The score is a measurement of how well the task has been performed.
     *
     * @return Score value between 0 and 100, where 0 means no success and 100 means full success.
     */
    virtual float score() const = 0;

    /**
     * @brief Gets the comparison type for this step
     *
     * @return Reference to the step's comparison type
     */
    const Type& type() const
    {
        return type_;
    }

protected:

    Type type_ = Type::UNKNOWN;      ///< Type of comparison for success evaluation
};

/**
 * @struct TaskStep
 *
 * @brief Base class representing a single condition or action within a task
 *
 * @details A TaskStep defines a goal condition that must be met by a specific sensor
 *          reading. It provides the interface for checking condition completion and
 *          accessing expected values.
 */
struct TaskStep :
    public TaskStepBase
{
    /**
     * @brief Constructs a new TaskStep object
     *
     * @param sensor Reference to the sensor that will be monitored
     */
    TaskStep(
            SensorReader& sensor)
        : sensor_(sensor)
    {
        // By default a task step begins a read operation on the sensor
        sensor.start_read();
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~TaskStep() = default;

    /// Virtual method implementation
    void restart_step() const override
    {
        sensor_.start_read();
    }

    /// Virtual method implementation
    const std::string& name() const override
    {
        return sensor_.name();
    }

    /**
     * @brief Gets the associated sensor
     *
     * @return Reference to the sensor being monitored
     */
    SensorReader& sensor() const
    {
        return sensor_;
    }

protected:

    SensorReader& sensor_;           ///< Reference to the monitored sensor
};

struct TaskStepAuto :
    public TaskStepBase
{
    /**
     * @brief Constructs a new TaskStepAuto object
     */
    TaskStepAuto(std::string name = "")
        : name_(name)
    {
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~TaskStepAuto() = default;

    /// Virtual method implementation
    void restart_step() const override
    {
    }

    /// Virtual method implementation
    const std::string& name() const override
    {
        return name_;
    }

    /// Virtual method implementation
    SensorMeasurement expected_value() const override
    {
        return SensorMeasurement();
    }

protected:

    std::string name_ = "TaskStepAuto"; ///< Name of the task step
};
