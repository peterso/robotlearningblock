/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <hal/PbHubController.hpp>

struct HardwareLowLevelController
{
    PbHubController & pb_hub_controller;
    m5::M5Unified & m5_unified;
};