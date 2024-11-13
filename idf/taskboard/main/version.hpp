/**
 * Roboton Task Board Firmware
 */

#include <cstdint>
#include <string>

// Firmware version
constexpr uint32_t FW_VERSION_MAJOR = 1;
constexpr uint32_t FW_VERSION_MINOR = 0;
constexpr uint32_t FW_VERSION_PATCH = 0;

// Firmware version string
const std::string FW_VERSION_STRING = std::to_string(FW_VERSION_MAJOR) + "." + std::to_string(FW_VERSION_MINOR) + "." + std::to_string(FW_VERSION_PATCH);