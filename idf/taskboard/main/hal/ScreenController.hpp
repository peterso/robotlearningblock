/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <hal/HardwareLowLevelController.hpp>
#include <hal/ClueScreenController.hpp>
#include <task/Task.hpp>

#include <M5Unified.h>

/**
 * @struct ScreenController

 * @brief Controller for managing display output on M5Stack hardware
 */
struct ScreenController :
    public ClueScreenController
{
    /**
     * @brief Constructs a new ScreenController object
     * @param hardware_low_level_controller Reference to hardware controller for display access
     */
    ScreenController(
            HardwareLowLevelController& hardware_low_level_controller)
        : display_(hardware_low_level_controller.m5_unified.Display)
    {
        clear();
        print("Robothon Task Board");
    }

    /**
     * @brief Clears the display and resets to initial state
     *
     * @param text_color Color for text output (default: TFT_WHITE)
     * @param background_color Color for background (default: TFT_BLACK)
     */
    void clear(
            int text_color = TFT_WHITE,
            int background_color = TFT_BLACK)
    {
        display_.setRotation(3);
        display_.setTextColor(TFT_WHITE, TFT_BLACK);
        display_.setTextSize(1);
        display_.setCursor(5, 0);

        display_.fillScreen(TFT_BLACK);

        current_line_ = 0;
        print_banner();
    }

    /**
     * @brief Prints a text line to the display
     *
     * @param text String to be displayed
     */
    void print(
            const std::string& text)
    {
        display_.setCursor(5, 7 + current_line_ * 10);
        display_.print(text.c_str());
        current_line_++;
    }

    /**
     * @brief Displays task status with visual progress indicators
     *
     * @param task_name Name of the task to display
     * @param task Reference to task object for status information
     * @param text_color Color for text output (default: TFT_WHITE)
     * @param background_color Color for background (default: TFT_BLACK)
     */
    void print_task_status(
            const std::string& task_name,
            const Task& task,
            int text_color = TFT_WHITE,
            int background_color = TFT_BLACK)
    {
        display_.setRotation(3);
        display_.setTextColor(text_color, background_color);
        display_.setTextSize(1);
        display_.setCursor(5, 0);

        display_.fillScreen(background_color);

        display_.setCursor(5, 0);
        display_.setTextSize(2);
        display_.print(task_name.c_str());
        display_.setTextSize(1);

        // Draw one circle per each task step
        constexpr uint32_t CIRCLE_RADIUS = 10;

        for (size_t i = 0; i < task.total_steps(); i++)
        {
            if (task.step_done(i))
            {
                display_.fillCircle(15 + ((CIRCLE_RADIUS + 3) * 2 * i), 30, CIRCLE_RADIUS, TFT_GREEN);
            }
            else
            {
                display_.drawCircle(15 + ((CIRCLE_RADIUS + 3) * 2 * i), 30, CIRCLE_RADIUS, TFT_RED);
            }
        }
    }

    /**
     * @brief Displays elapsed time for task execution
     *
     * @param time Time in seconds to display
     */
    void print_task_status_time(
            float time)
    {
        display_.setCursor(5, 50);
        display_.print("Time: ");
        display_.print(time);
        display_.print(" s");
    }

    /// Virtual method implementation
    void print_task_clue(
            const std::string& clue) override
    {
        display_.setCursor(5, 70);
        display_.print(clue.c_str());
    }

    /// Virtual method implementation
    void print_task_clue_analog(
            const float& current_value,
            const float& target_value) override
    {
        const int32_t w = display_.width();
        const int32_t h = display_.height();

        const int32_t item_offset = 20;
        const int32_t item_width = w - (2 * item_offset);
        const int32_t item_height = 40;
        const int32_t item_height_offset = h - item_height - 10;

        display_.fillRect(item_offset, item_height_offset, item_width, item_height, TFT_RED);

        const int32_t cursor_width = 6;

        // Draw current cursor
        display_.fillRect(item_offset + (item_width * current_value) - cursor_width / 2,
                item_height_offset, cursor_width, item_height, TFT_BLACK);

        // Draw target cursor
        display_.fillRect(item_offset + (item_width * target_value) - cursor_width / 2,
                item_height_offset, cursor_width, item_height, TFT_BLUE);
    }

    /// Virtual method implementation
    void clear_all_task_clue() override
    {
        display_.fillRect(5, 70, display_.width(), display_.height(), TFT_BLACK);
    }

private:

    /**
     * @brief Prints the title banner at the top of the display
     */
    void print_banner()
    {
        display_.setCursor(5, 0);
        display_.setTextSize(2);
        display_.print("Robothon Task Board");
        display_.setTextSize(1);
        current_line_++;
    }

    m5gfx::M5GFX& display_;            ///< Reference to M5Stack display hardware
    uint32_t current_line_ = 0;        ///< Current line position for text output
};
