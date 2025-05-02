/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Actuator.hpp>
#include <task/TaskStep.hpp>

#include <esp_log.h>
#include <esp_random.h>

/**
 * @struct TaskStepActuator
 *
 * @brief Implementation of TaskStep that acts on a Hardware actuator
 */
struct TaskStepActuator :
    public TaskStepAuto
{
    /**
     * @brief Constructs a new TaskStepActuator object
     * 
     * @param actuator Actuator to control
     * @param state State to set the actuator to
     */
    TaskStepActuator(
            Actuator& actuator,
            const Actuator::State state)
        : TaskStepAuto(actuator.name()),
            actuator_(actuator),
            state_(state)
    {
        TaskStepAuto::type_ = Type::ACTUATOR;
    }

    /// Virtual method implementation
    void initialize_step() const override
    {
        actuator_.set_state(state_);
    }

    /// Virtual method implementation
    bool success() const override
    {
        return true;
    }

    /// Virtual method implementation
    float score() const override
    {
        return -1.0f;
    }


private:

    /// Virtual method implementation
    void show_clue_implementation(
            ClueScreenController& screen_controller) const override
    {
        if (!success())
        {
            screen_controller.print_task_clue("Activating " + actuator_.name() + " to state " + std::to_string(state_));
        }
        else
        {
            reset_clue();
        }
    }

    Actuator& actuator_;           ///< Reference to the actuator
    const Actuator::State state_;  ///< State to set the actuator to
};
