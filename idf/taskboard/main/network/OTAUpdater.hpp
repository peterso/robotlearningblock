/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <version.hpp>

#include <esp_http_client.h>
#include <esp_crt_bundle.h>
#include <esp_https_ota.h>

#include <cJSON.h>

#include <string>

/**
 * @struct OTAUpdater
 *
 * @brief Class for handling OTA updates
 */
struct OTAUpdater
{
    const char* TAG = "OTAUpdater";     ///< Logging tag
    const char* OTA_URI = "https://api.github.com/repos/peterso/robotlearningblock/releases/latest";

    /**
     * @brief Constructs a new OTAUpdater object
     *
     * @param screen_controller Screen controller
     */
    OTAUpdater(
            ScreenController& screen_controller)
        : screen_controller_(screen_controller)
    {
        // Make HTTP request to get latest release
        esp_http_client_config_t config = {};
        config.url = OTA_URI;
        config.crt_bundle_attach = esp_crt_bundle_attach;
        config.timeout_ms = 10000;
        config.buffer_size = 512;
        config.event_handler = handle_http;
        config.user_data = this;

        // Preallocate memory for the response
        http_data_.reserve(3000);

        esp_http_client_handle_t client = esp_http_client_init(&config);

        esp_err_t err = esp_http_client_perform(client);
        esp_http_client_cleanup(client);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "HTTP GET readed = %d", http_data_.size());

            cJSON* root = cJSON_Parse(http_data_.c_str());

            // Extract the version ("name")
            cJSON* name = cJSON_GetObjectItem(root, "name");

            if (name && cJSON_IsString(name))
            {
                ESP_LOGI(TAG, "Latest release: %s", name->valuestring);
                latest_version_ = name->valuestring;
            }
            else
            {
                ESP_LOGE(TAG, "Version not found");
            }

            // Extract the "assets" array
            cJSON* assets = cJSON_GetObjectItem(root, "assets");

            if (assets && cJSON_IsArray(assets))
            {
                cJSON* asset;
                cJSON_ArrayForEach(asset, assets) {
                    cJSON* asset_name = cJSON_GetObjectItem(asset, "name");

                    if (asset_name && cJSON_IsString(asset_name) &&
                            strcmp(asset_name->valuestring, "taskboard.bin") == 0)
                    {
                        // Found the asset named "firmware.bin"
                        ESP_LOGI(TAG, "Firmware asset found");
                        cJSON* browser_download_url = cJSON_GetObjectItem(asset, "browser_download_url");

                        if (browser_download_url && cJSON_IsString(browser_download_url))
                        {
                            ESP_LOGI(TAG, "Firmware URL: %s", browser_download_url->valuestring);
                            firmware_url_ = browser_download_url->valuestring;
                        }

                        break;
                    }
                }
            }
            else
            {
                ESP_LOGE(TAG, "Assets not found");
            }

            // Clean up
            cJSON_Delete(root);
        }
        else
        {
            ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
        }

    }

    /**
     * @brief Check if the firmware can be updated
     *
     * @return true if the firmware can be updated, false otherwise
     */
    bool can_update()
    {
        // latest_version_ is vX.Y.Z get the version numbers
        std::string version = latest_version_.substr(1);
        uint8_t major = std::stoi(version.substr(0, version.find('.')));
        version = version.substr(version.find('.') + 1);
        uint8_t minor = std::stoi(version.substr(0, version.find('.')));
        version = version.substr(version.find('.') + 1);
        uint8_t patch = std::stoi(version);

        // Check if the latest version is greater than the current version
        const bool newer_version =
                major > FW_VERSION_MAJOR ||
                (major == FW_VERSION_MAJOR && minor > FW_VERSION_MINOR) ||
                (major == FW_VERSION_MAJOR && minor == FW_VERSION_MINOR && patch > FW_VERSION_PATCH);

        // Check if there is a firmware URL
        const bool has_firmware_url = !firmware_url_.empty();

        return newer_version && has_firmware_url;
    }

    /**
     * @brief Get the latest version
     *
     * @return Latest version
     */
    const std::string& get_latest_version() const
    {
        return latest_version_;
    }

    /**
     * @brief Update firmware
     *
     * @note This function will restart the device
     */
    void update()
    {
        if (!can_update())
        {
            return;
        }

        esp_http_client_config_t config = {};
        config.url = firmware_url_.c_str();
        config.crt_bundle_attach = esp_crt_bundle_attach;
        config.keep_alive_enable = true;
        config.buffer_size_tx = 5 * 1024;

        esp_https_ota_config_t ota_config = {};
        ota_config.http_config = &config;

        ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &ota_event_handler, this));

        esp_err_t ret = esp_https_ota(&ota_config);

        if (ret == ESP_OK)
        {
            esp_restart();
        }

        // Shall not reach this point
        while (true)
        {
            ESP_LOGE(TAG, "Firmware update failed");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        }
    }

private:

    /**
     * @brief Handle HTTP events
     *
     * @param evt HTTP event
     */
    static esp_err_t handle_http(
            esp_http_client_event_t* evt)
    {
        OTAUpdater* updater = (OTAUpdater*)evt->user_data;

        switch (evt->event_id){
            case HTTP_EVENT_ON_DATA:
            {
                updater->http_data_ += std::string((char*)evt->data, evt->data_len);
                break;
            }
            default:
                break;
        }

        return ESP_OK;
    }

    /**
     * @brief HTTPS OTA Handler
     *
     * @param arg Argument
     * @param event_base Event base
     * @param event_id Event ID
     * @param event_data Event data
     */
    static void ota_event_handler(
            void* arg,
            esp_event_base_t event_base,
            int32_t event_id,
            void* event_data)
    {
        OTAUpdater* updater = (OTAUpdater*)arg;

        if (event_base == ESP_HTTPS_OTA_EVENT)
        {
            updater->screen_controller_.clear();

            switch (event_id){
                case ESP_HTTPS_OTA_START:
                    ESP_LOGI(updater->TAG, "OTA started");
                    updater->screen_controller_.print("-> OTA started");
                    break;
                case ESP_HTTPS_OTA_CONNECTED:
                    ESP_LOGI(updater->TAG, "Connected to server");
                    updater->screen_controller_.print("-> Connected to server");
                    break;
                case ESP_HTTPS_OTA_GET_IMG_DESC:
                    ESP_LOGI(updater->TAG, "Reading Image Description");
                    updater->screen_controller_.print("-> Reading Image Description");
                    break;
                case ESP_HTTPS_OTA_VERIFY_CHIP_ID:
                    ESP_LOGI(updater->TAG, "Verifying chip id of new image: %d", *(esp_chip_id_t*)event_data);
                    updater->screen_controller_.print("-> Verifying chip id");
                    break;
                case ESP_HTTPS_OTA_DECRYPT_CB:
                    ESP_LOGI(updater->TAG, "Callback to decrypt function");
                    updater->screen_controller_.print("-> Decrypting");
                    break;
                case ESP_HTTPS_OTA_WRITE_FLASH:
                    ESP_LOGD(updater->TAG, "Writing to flash: %d written", *(int*)event_data);
                    updater->screen_controller_.print("-> Writing to flash: " + std::to_string(
                                *(int*)event_data) + " bytes");
                    break;
                case ESP_HTTPS_OTA_UPDATE_BOOT_PARTITION:
                    ESP_LOGI(updater->TAG, "Boot partition updated. Next Partition: %d",
                            *(esp_partition_subtype_t*)event_data);
                    updater->screen_controller_.print("-> Boot partition updated");
                    break;
                case ESP_HTTPS_OTA_FINISH:
                    ESP_LOGI(updater->TAG, "OTA finish");
                    updater->screen_controller_.print("-> OTA finish");
                    break;
                case ESP_HTTPS_OTA_ABORT:
                    ESP_LOGI(updater->TAG, "OTA abort");
                    updater->screen_controller_.print("-> OTA abort");
                    break;
            }
        }
    }

    std::string latest_version_;      ///< Latest version
    std::string firmware_url_;        ///< URL to download firmware
    std::string http_data_;           ///< HTTP response data

    ScreenController& screen_controller_;  ///< Screen controller
};
