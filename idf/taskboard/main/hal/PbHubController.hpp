/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <M5Unified.h>

#include <esp_log.h>

#include <cstdint>

/**
 * @struct PbHubController
 *
 * @brief Controller for I2C communication with PbHub device
 *
 */
struct PbHubController
{
    const char* TAG = "PbHubController";    ///< Logging tag

    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x61;   ///< Default I2C address for PbHub
    static constexpr uint32_t I2C_FREQ = 400000;        ///< I2C bus frequency in Hz

    /**
     * @enum Operation
     */
    enum class Operation : uint8_t
    {
        WRITE_IO0 = 0x00,       ///< Write to IO0 pin
        WRITE_IO1 = 0x01,       ///< Write to IO1 pin
        PWM_IO0 = 0x02,         ///< PWM output on IO0
        PWM_IO1 = 0x03,         ///< PWM output on IO1
        READ_IO0 = 0x04,        ///< Read from IO0 pin
        READ_IO1 = 0x05,        ///< Read from IO1 pin
        ANALOG_READ_IO0 = 0x06, ///< Analog read from IO0
    };

    /**
     * @enum Channel
     */
    enum class Channel : uint8_t
    {
        CHANNEL_0 = 0x40,   ///< Channel 0 base address
        CHANNEL_1 = 0x50,   ///< Channel 1 base address
        CHANNEL_2 = 0x60,   ///< Channel 2 base address
        CHANNEL_3 = 0x70,   ///< Channel 3 base address
        CHANNEL_4 = 0x80,   ///< Channel 4 base address
        CHANNEL_5 = 0xA0,   ///< Channel 5 base address
    };

    /**
     * @brief Constructs a new PbHubController object
     *
     * @details Initializes I2C communication with default address
     */
    PbHubController() noexcept
        : i2c_addr_(DEFAULT_I2C_ADDR)
    {
        bool init = M5.Ex_I2C.begin();

        if (init)
        {
            ESP_LOGI(TAG, "PbHubController::PbHubController() on address %d", i2c_addr_);
        }
        else
        {
            ESP_LOGE(TAG, "PbHubController::PbHubController() failed");
        }
    }

    /**
     * @brief Checks if PbHub is responding
     *
     * @return true if communication successful, false otherwise
     */
    bool check_status()
    {
        // Sample read on a register to check that I2C device is properly answering
        uint8_t value = 0;

        return read_operation(Channel::CHANNEL_0, Operation::READ_IO0, reinterpret_cast<uint8_t*>(&value),
                       sizeof(value));
    }

    /**
     * @brief Reads digital value from IO0 pin
     *
     * @param channel Channel to read from
     *
     * @return true if pin is high, false if low
     */
    bool read_digital_IO0(
            const Channel channel)
    {
        uint8_t value = 0;
        read_operation(channel, Operation::READ_IO0, reinterpret_cast<uint8_t*>(&value), sizeof(value));

        return value != 0;
    }

    /**
     * @brief Reads digital value from IO1 pin
     *
     * @param channel Channel to read from
     *
     * @return true if pin is high, false if low
     */
    bool read_digital_IO1(
            const Channel channel)
    {
        uint8_t value = 0;
        read_operation(channel, Operation::READ_IO1, reinterpret_cast<uint8_t*>(&value), sizeof(value));

        return value != 0;
    }

    /**
     * @brief Reads analog value from IO0 pin
     *
     * @param channel Channel to read from
     *
     * @return 16-bit analog reading
     */
    uint16_t read_analog_IO0(
            const Channel channel)
    {
        uint16_t value = 0;
        read_operation(channel, Operation::ANALOG_READ_IO0, reinterpret_cast<uint8_t*>(&value), sizeof(value));

        return value;
    }

    /**
     * @brief Writes digital value to IO0 pin
     *
     * @param channel Channel to write to
     * @param value true for high, false for low
     */
    void write_digital_IO0(
            const Channel channel,
            const bool value)
    {
        uint8_t data = value;
        write_operation(channel, Operation::WRITE_IO0, reinterpret_cast<uint8_t*>(&data), sizeof(data));
    }

    /**
     * @brief Writes digital value to IO1 pin
     *
     * @param channel Channel to write to
     * @param value true for high, false for low
     */
    void write_digital_IO1(
            const Channel channel,
            const bool value)
    {
        uint8_t data = value;
        write_operation(channel, Operation::WRITE_IO1, reinterpret_cast<uint8_t*>(&data), sizeof(data));
    }

    /**
     * @brief Writes PWM value to IO0 pin
     *
     * @param channel Channel to write to
     * @param value range 0 - 255
     */
     void write_PWM_IO0(
        const Channel channel,
        const uint8_t value)
    {
        uint8_t data = value;
        write_operation(channel, Operation::PWM_IO0, reinterpret_cast<uint8_t*>(&data), sizeof(data));
    }

    /**
    * @brief Writes PWM value to IO1 pin
    *
    * @param channel Channel to write to
    * @param value range 0 - 255
    */
    void write_PWM_IO1(
            const Channel channel,
            const uint8_t value)
    {
        uint8_t data = value;
        write_operation(channel, Operation::PWM_IO1, reinterpret_cast<uint8_t*>(&data), sizeof(data));
    }

protected:

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
    virtual bool read_operation(
            const Channel channel,
            const Operation operation,
            uint8_t* data,
            const size_t length)
    {
        bool status = true;
        status = status && M5.Ex_I2C.start(i2c_addr_, false, I2C_FREQ);
        status = status && M5.Ex_I2C.write(static_cast<uint8_t>(channel) + static_cast<uint8_t>(operation));
        status = status && M5.Ex_I2C.stop();

        status = status && M5.Ex_I2C.start(i2c_addr_, true, I2C_FREQ);

        for (size_t i = 0; i < length; i++)
        {
            status = status && M5.Ex_I2C.read(&data[i], 1);
        }

        status = status && M5.Ex_I2C.stop();

        if (!status)
        {
            ESP_LOGE(TAG, "PbHubController::read_digital() failed");
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
    virtual bool write_operation(
            const Channel channel,
            const Operation operation,
            const uint8_t* data,
            const size_t length)
    {
        bool status = true;
        status = status && M5.Ex_I2C.start(i2c_addr_, false, I2C_FREQ);
        status = status && M5.Ex_I2C.write(static_cast<uint8_t>(channel) + static_cast<uint8_t>(operation));
        status = status && M5.Ex_I2C.write(data, length);
        status = status && M5.Ex_I2C.stop();

        if (!status)
        {
            ESP_LOGE(TAG, "PbHubController::write_digital() failed");
        }

        return status;
    }

private:

    // const uint8_t i2c_addr_ = 0;   ///< I2C address of PbHub device
    const uint8_t i2c_addr_ = DEFAULT_I2C_ADDR;   ///< I2C address of PbHub device
};
