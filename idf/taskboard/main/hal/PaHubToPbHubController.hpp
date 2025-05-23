/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <M5Unified.h>

#include <hal/PbHubController.hpp>

#include <esp_log.h>

#include <cstdint>

/**
 * @struct PaHubController
 * 
 * @brief Controller for I2C communication with PaHub device
 * 
 */
struct PaHubController
{
    const char* TAG = "PaHubController";    ///< Logging tag

    static constexpr uint32_t I2C_FREQ = 400000;        ///< I2C bus frequency in Hz
    static constexpr uint8_t PAHUB_I2C_ADDR = 0x70;   ///< Default I2C address for PaHub

    /**
     * @brief Constructs a new PaHubController object
     *
     * @details Initializes I2C communication with default address
     */
    PaHubController() noexcept
        : i2c_addr_(PAHUB_I2C_ADDR)
    {
        bool init = M5.Ex_I2C.begin();

        if (init)
        {
            ESP_LOGI(TAG, "PaHubController::PaHubController() on address %d", i2c_addr_);
        }
        else
        {
            ESP_LOGE(TAG, "PaHubController::PaHubController() failed");
        }
    }

    /**
     * @brief Selects the TCA9548A I2C multiplexer channel
     *
     * @param bus Target bus number
     *
     * @return true if operation successful, false otherwise
     */
    bool TCA9548A_select(uint8_t bus)
    {
        bool status = true;

        if (bus != last_channel_selected_)
        {
            status = status && M5.Ex_I2C.start(i2c_addr_, false, I2C_FREQ);
            status = status && M5.Ex_I2C.write(1 << bus);
            status = status && M5.Ex_I2C.stop();
            if (status)
            {
                last_channel_selected_ = bus;
                ESP_LOGI(TAG, "TCA9548A_select() bus %d selected", bus);
            }
        }

        return status;
    }

private:

    const uint8_t i2c_addr_ = 0;   ///< I2C address of PaHub device
    uint8_t last_channel_selected_ = -1;   ///< Last selected channel

};



/**
 * @struct PaHubToPbHubController
 *
 * @brief Controller for I2C communication with PbHub device through PaHub
 *
 */
struct PaHubToPbHubController :
    public PbHubController
{
    const char* TAG = "PaHubToPbHubController";    ///< Logging tag

    static constexpr uint8_t PBHUB_I2C_ADDR = 0x61;   ///< Default I2C address for PbHub

    /**
     * @brief Constructs a new PaHubToPbHubController object
     *
     * @details Initializes I2C communication with default address
     */
    PaHubToPbHubController(PaHubController& pa_hub_controller,uint8_t pb_hub_channel) noexcept
        :   i2c_addr_(PBHUB_I2C_ADDR),
            pa_hub_controller_(pa_hub_controller),
            pb_hub_channel_(pb_hub_channel)
    {
        bool init = M5.Ex_I2C.begin();

        if (init)
        {
            ESP_LOGI(TAG, "PaHubToPbHubController::PaHubToPbHubController() on address %d", i2c_addr_);
        }
        else
        {
            ESP_LOGE(TAG, "PaHubToPbHubController::PaHubToPbHubController() failed");
        }
    }

private:

    /**
     * @brief Performs I2C read operation
     *
     * @param channel Target channel
     * @param operation Operation to perform
     * @param data Buffer for read data
     * @param length Number of bytes to read
     *
     * @return true if operation successful, false otherwise
     */
    bool read_operation(
            const Channel channel,
            const Operation operation,
            uint8_t* data,
            const size_t length) override
    {
        bool status = true;
        
        status = status && pa_hub_controller_.TCA9548A_select(pb_hub_channel_);

        status = status && PbHubController::read_operation(channel, operation, data, length);

        if (!status)
        {
            ESP_LOGE(TAG, "PaHubToPbHubController::read_digital() failed");
        }

        return status;
    }

    /**
     * @brief Performs I2C write operation
     *
     * @param channel Target channel
     * @param operation Operation to perform
     * @param data Data to write
     * @param length Number of bytes to write
     *
     * @return true if operation successful, false otherwise
     */
    bool write_operation(
            const Channel channel,
            const Operation operation,
            const uint8_t* data,
            const size_t length) override
    {
        bool status = true;

        status = status && pa_hub_controller_.TCA9548A_select(pb_hub_channel_);

        status = status && PbHubController::write_operation(channel, operation, data, length);

        if (!status)
        {
            ESP_LOGE(TAG, "PaHubToPbHubController::write_digital() failed");
        }

        return status;
    }

    const uint8_t i2c_addr_ = 0;        ///< I2C address of PbHub device
    PaHubController& pa_hub_controller_;   ///< Reference to PaHub controller
    const uint8_t pb_hub_channel_ = 0;   ///< Channel of PaHub where PbHub is located
};
