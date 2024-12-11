/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <cstdint>
#include <string>

// Firmware version
constexpr uint32_t FW_VERSION_MAJOR = 2;
constexpr uint32_t FW_VERSION_MINOR = 8;
constexpr uint32_t FW_VERSION_PATCH = 0;

// Firmware version string
const std::string FW_VERSION_STRING = std::to_string(FW_VERSION_MAJOR) + "." + std::to_string(FW_VERSION_MINOR) + "." +
        std::to_string(FW_VERSION_PATCH);
