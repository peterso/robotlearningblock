/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <hal/PbHubController.hpp>

/**
* @struct HardwareLowLevelController
*
* @brief Aggregates low-level hardware controllers
*/
struct HardwareLowLevelController
{
   PbHubController & pb_hub_controller;    ///< Reference to PbHub I/O expansion controller
   m5::M5Unified & m5_unified;             ///< Reference to M5Stack core functionality controller
};