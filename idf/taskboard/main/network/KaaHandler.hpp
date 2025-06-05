/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <network/JSONHandler.hpp>
#include <hal/TaskBoardDriver.hpp>

#include <esp_mac.h>
#include <esp_log.h>

/**
 * @struct KaaHandler
 *
 * @brief Helper class for handling communication with Kaa IoT platform
 */
struct KaaHandler
{
    const char* TAG = "KaaHandler"; ///< Logging tag

    /**
     * @brief Constructs a new KaaHandler object
     *
     * @param task_board_driver Task board driver
     */
    KaaHandler(
            const TaskBoardDriver& task_board_driver)
        : task_board_driver_(task_board_driver)
    {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);

        char KAA_URI[80];
        
        sprintf(KAA_URI, "http://cloud.kaaiot.com/kpc/kp1/c1v9jqmgul2l1s47m6bg-v0/dcx/task_board_%01X%02X/json", (mac[4] & 0x0F), mac[5]);

        // Make HTTP request to get latest release
        esp_http_client_config_t config = {};
        config.url = KAA_URI;
        config.method = HTTP_METHOD_POST;
        config.buffer_size = 512;
        config.crt_bundle_attach = esp_crt_bundle_attach;
        config.is_async = false;
        config.timeout_ms = 1000;

        client_ = esp_http_client_init(&config);

        esp_http_client_set_header(client_, "Content-Type", "application/json");
    }

    /**
     * @brief Destructor
     */
    ~KaaHandler()
    {
        esp_http_client_cleanup(client_);
    }

    /**
     * @brief Sends Kaa telemetry
     */
    void send_telemetry()
    {
        JSONHandler json_handler;
        json_handler.add_taskboard_status_kaaiot(task_board_driver_);

        const char* status = json_handler.get_json_string();

        esp_http_client_set_post_field(client_, status, strlen(status));

        // Best effort approach, do not complain on failure
        esp_http_client_perform(client_);
    }

private:

    esp_http_client_handle_t client_;           ///< HTTP client handle
    const TaskBoardDriver& task_board_driver_;  ///< Task board driver
};
