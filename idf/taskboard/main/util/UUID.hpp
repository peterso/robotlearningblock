/**
 * Robothon Task Board Firmware
 */

#pragma once

#include "esp_random.h"

/**
 * @brief Generate a random UUID
 *
 * @tparam T Type of the UUID
 */
template<typename T>
inline void uuid_generate(T& out)
{
    esp_fill_random(out, sizeof(T));
}

/**
 * @brief Get UUID as hex string
 */
template<typename T>
inline std::string uuid_to_string(const T& uuid)
{
    char str[2 * sizeof(T) + 1];
    for (size_t i = 0; i < sizeof(T); i++)
    {
        snprintf(&str[2 * i], 3, "%02x", ((uint8_t*)&uuid)[i]);
    }
    return std::string(str);
}
