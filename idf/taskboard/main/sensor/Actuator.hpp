/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <esp_log.h>

#include <functional>
#include <string>

/**
 * @struct Acuator
 *
 * @brief Concrete implementation of an actuator
 */
struct Actuator
{
    enum State
    {
        OFF = 0,
        ON = 1
    };

    /// @brief Function type for setting actuator state
    using SetFunction = std::function<void(State)>;

    /**
     * @brief Constructs a new Actuator object
     *
     * @param name Identifier for the actuator
     * @param set_state_function Callback function that changes the actuator state
     */
    Actuator(
            const std::string& name,
            SetFunction set_state_function)
        : name_(name),
          set_function_(set_state_function)
    {
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~Actuator() = default;

    /**
     * @brief Gets the name of the actuator
     */
    const std::string& name() const
    {
        return name_;
    }

    /**
     * @brief Sets the state of the actuator
     * 
     * @param status State to set the actuator to
     */
    virtual void set_state(State status) const
    {
        set_function_(status);
    }

protected:

    std::string name_;              ///< Sensor identifier
    SetFunction set_function_;      ///< Callback function to set actuator state
};
