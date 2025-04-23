/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <hal/PbHubController.hpp>
#include <hal/PaHubToPbHubController.hpp>

/**
 * @struct HardwareLowLevelController
 *
 * @brief Aggregates low-level hardware controllers
 */
struct HardwareLowLevelController_v2023
{
    PbHubController& pb_hub_controller;    ///< Reference to PbHub I/O expansion controller
    m5::M5Unified& m5_unified;             ///< Reference to M5Stack core functionality controller
};

struct HardwareLowLevelController
{
    PaHubToPbHubController& pb_hub_controller_1;    ///< Reference to PbHub I/O expansion controller
    PaHubToPbHubController& pb_hub_controller_2;    ///< Reference to PbHub I/O expansion controller
    m5::M5Unified& m5_unified;                     ///< Reference to M5Stack core functionality controller
};