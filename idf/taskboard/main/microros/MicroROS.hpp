/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <microros/MicroROSController.hpp>
#include <hal/TaskBoardDriver.hpp>
#include <task/TaskExecutor.hpp>

#include <esp_log.h>

/**
 * @struct MicroROSMainArgs

 * @brief Container for arguments passed to microros_main task
 */
struct MicroROSMainArgs
{
    MicroROSController& microros_controller;    ///< Reference to micro-ROS controller
    TaskBoardDriver& task_board_driver;         ///< Reference to task board hardware interface
    TaskExecutor& task_executor;                ///< Reference to task execution controller
    TaskHandle_t main_task_handle;              ///< Handle to main task
};

/// Notification bit indicating manual task cancellation from Core 0
constexpr uint8_t MANUALLY_CANCELLED_TASK = 0x01 << 0;
constexpr uint8_t MICROROS_CANCELLED_TASK = 0x01 << 1;
constexpr uint8_t MICROROS_TIMEOUT_TASK = 0x01 << 2;
constexpr uint8_t MICROROS_TASK_UPDATE_REQUIRED = 0x01 << 3;

/**
 * @brief Main function for MicroROS task execution
 *
 * @param arg Type erased pointer to MicroROSMainArgs structure
 */
void microros_main(
        void* arg);
