/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sdkconfig.h>

#include <hal/PbHubController.hpp>
#include <hal/PaHubToPbHubController.hpp>

/**
 * @struct HardwareLowLevelController
 *
 * @brief Aggregates low-level hardware controllers
 */
struct HardwareLowLevelController
{

#if CONFIG_M5STACK_CORE2
    PaHubToPbHubController& pb_hub_controller_1;    ///< Reference to PbHub I/O expansion controller
    PaHubToPbHubController& pb_hub_controller_2;    ///< Reference to PbHub I/O expansion controller
#else
    PbHubController& pb_hub_controller;    ///< Reference to PbHub I/O expansion controller
#endif

    m5::M5Unified& m5_unified;                     ///< Reference to M5Stack core functionality controller
};