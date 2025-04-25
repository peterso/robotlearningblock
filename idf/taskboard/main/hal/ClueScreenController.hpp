/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <string>
#include <cstdint>

/**
 * @struct ClueScreenController

 * @brief Interface for isolating the API exposed to the task steps
 */
struct ClueScreenController
{
    /**
     * @brief Displays a text clue for task guidance
     *
     * @param clue String containing guidance information
     */
    virtual void print_task_clue(
            const std::string& clue) = 0;

    /**
     * @brief Displays analog feedback with visual indicators
     *
     * @param current_value Current analog sensor value (0.0 to 1.0)
     * @param target_value Target analog value (0.0 to 1.0)
     */
    virtual void print_task_clue_analog(
            const float& current_value,
            const float& target_value) = 0;

     /*
      * @brief Displays a path clue without feedback
      *
      * @param expected_path Vector of SensorMeasurement objects representing the expected path
      * @param measured_path Vector of SensorMeasurement objects representing the measured path
      */
     virtual void print_task_clue_path(
            const std::vector<SensorMeasurement>& expected_path,
            const std::vector<SensorMeasurement>& measured_path) = 0;

    virtual void print_task_clue_goal(const char* clue_text) = 0;

    /**
     * @brief Clears the screen of any displayed clues
     */
    virtual void clear_all_task_clue() = 0;
};
