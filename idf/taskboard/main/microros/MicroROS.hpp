/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <microros/MicroROSController.hpp>
#include <hal/TaskBoardDriver.hpp>
#include <task/TaskExecutor.hpp>

#include <esp_log.h>

struct MicroROSMainArgs
{
    MicroROSController & microros_controller;
    TaskBoardDriver & task_board_driver;
    TaskExecutor & task_executor;
};

constexpr uint8_t MANUALLY_CANCELLED_TASK = 0x01;

void microros_main(void *arg);
