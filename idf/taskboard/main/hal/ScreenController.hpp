/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <hal/HardwareLowLevelController.hpp>
#include <task/Task.hpp>

#include <M5Unified.h>

struct ScreenController
{
    ScreenController(HardwareLowLevelController & hardware_low_level_controller)
    : display_(hardware_low_level_controller.m5_unified.Display)
    {
        clear();
        print("Roboton Task Board");
    }

    void clear(int text_color = TFT_WHITE, int background_color = TFT_BLACK)
    {
        display_.setRotation(3);
        display_.setTextColor(TFT_WHITE, TFT_BLACK);
        display_.setTextSize(1);
        display_.setCursor(5, 0);

        display_.fillScreen(TFT_BLACK);

        current_line_ = 0;
        print_banner();
    }

    void print(const std::string & text)
    {
        display_.setCursor(5, 7 + current_line_ * 10);
        display_.print(text.c_str());
        current_line_++;
    }

    void print_task_status(const std::string & task_name, const Task & task, int text_color = TFT_WHITE, int background_color = TFT_BLACK)
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

    void print_task_status_time(float time)
    {
        display_.setCursor(5, 50);
        display_.print("Time: ");
        display_.print(time);
        display_.print(" s");
    }

    void print_task_clue(const std::string & clue)
    {
        display_.setCursor(5, 70);
        display_.print(clue.c_str());
    }

    void print_task_clue_analog(const float & current_value, const float & target_value)
    {
        const int32_t w = display_.width();
        const int32_t h = display_.height();

        const int32_t item_offset = 20;
        const int32_t item_width = w - (2 * item_offset);
        const int32_t item_height = 40;
        const int32_t item_height_offset = h - item_height - 10;

        display_.fillRect(item_offset, item_height_offset, item_width, item_height , TFT_RED);

        const int32_t cursor_width = 6;

        // Draw current cursor
        display_.fillRect(item_offset + (item_width * current_value) - cursor_width/2, item_height_offset, cursor_width, item_height, TFT_BLACK);

        // Draw target cursor
        display_.fillRect(item_offset + (item_width * target_value) - cursor_width/2, item_height_offset, cursor_width, item_height, TFT_BLUE);
    }

private:

    void print_banner()
    {
        display_.setCursor(5, 0);
        display_.setTextSize(2);
        display_.print("Roboton Task Board");
        display_.setTextSize(1);
        current_line_++;
    }

    m5gfx::M5GFX & display_;
    uint32_t current_line_ = 0;
};