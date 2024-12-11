/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <network/HTTPServer.hpp>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>

#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_softap.h>

#include <string>

#include <network/webpages/web_wifiprovisioning.h>

// Maximum number of Wi-Fi connection retries
constexpr int WIFI_CONNECTION_MAX_RETRIES = 3;

/**
 * @struct WifiManager
 *
 * @brief Manages Wi-Fi provisioning and connection
 */
struct WifiManager
{
    const char* TAG = "WifiManager";                    ///< Logging tag

    /// @brief Event group for Wi-Fi events
    static constexpr int WIFI_CONNECTED_EVENT = BIT0;

    /**
     * @brief Constructs a new WifiManager object
     *
     * @param server HTTP server
     * @param provisioning_ssid SSID for provisioning
     * @param reset_provisioning Reset provisioning
     */
    WifiManager(
            HTTPServer& server,
            const std::string& provisioning_ssid,
            bool reset_provisioning = false)
        : server_(server)
        , provisioning_ssid_(provisioning_ssid)
    {
        // Create a wifi event group
        wifi_event_group_ = xEventGroupCreate();

        // Register our event handler for Wi-Fi, IP and Provisioning related events
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &WifiManager::event_handler,
                this));
        ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID,
                &WifiManager::event_handler, this));
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiManager::event_handler, this));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &WifiManager::event_handler, this));

        // Initialize Wi-Fi including netif with default config
        esp_netif_create_default_wifi_sta();
        esp_netif_create_default_wifi_ap();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        // Configuration for the provisioning manager
        wifi_prov_mgr_config_t config = {};
        config.scheme = wifi_prov_scheme_softap;
        config.scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE;

        ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

        // Reset provisioning if requested
        if (reset_provisioning)
        {
            ESP_LOGI(TAG, "Reset Provisioning");
            wifi_prov_mgr_reset_provisioning();
        }

        // Check if device is provisioned
        provisioned_ = false;
        ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned_));

        if (!provisioned_)
        {
            // Register a / handler for the provisioning app
            httpd_uri_t uri = {
                .uri      = "/",
                .method   = HTTP_GET,
                .handler  = WifiManager::serve_provisioning_page,
                .user_ctx = NULL
            };
            httpd_register_uri_handler(server.get_handle(), &uri);
            wifi_prov_scheme_softap_set_httpd_handle(&server.get_handle());

            ESP_LOGI(TAG, "Starting provisioning webapp");
            ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_0, NULL,
                    provisioning_ssid.c_str(), NULL));
        }
        else
        {
            wifi_config_t wifi_config;
            esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
            ssid_ = (char*)wifi_config.sta.ssid;

            ESP_LOGI(TAG, "Already provisioned, connecting to %s", ssid_.c_str());

            wifi_prov_mgr_deinit();

            // Start Wi-Fi in station mode
            ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
            ESP_ERROR_CHECK(esp_wifi_start());
        }
    }

    /**
     * @brief Waits for connection to the AP
     */
    void wait_for_connection()
    {
        ESP_LOGI(TAG, "Waiting for connection to the AP");

        xEventGroupWaitBits(wifi_event_group_, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);

        wifi_config_t wifi_config;
        esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
        ssid_ = (char*)wifi_config.sta.ssid;

        provisioned_ = true;

        // Clean up
        esp_event_handler_unregister(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &WifiManager::event_handler);
        esp_event_handler_unregister(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &WifiManager::event_handler);
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiManager::event_handler);
        esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &WifiManager::event_handler);
        vEventGroupDelete(wifi_event_group_);

        // Remove URI handler
        httpd_unregister_uri_handler(server_.get_handle(), "/", HTTP_GET);
    }

    /**
     * @brief Gets the IP address
     *
     * @return IP address
     */
    const std::string& get_ip() const
    {
        return ip_;
    }

    /**
     * @brief Gets the SSID
     *
     * @return SSID
     */
    const std::string& get_ssid() const
    {
        return ssid_;
    }

    /**
     * @brief Gets the provisioning SSID
     *
     * @return Provisioning SSID
     */
    const std::string& get_provisioning_ssid() const
    {
        return provisioning_ssid_;
    }

    /**
     * @brief Checks if the device is provisioned
     *
     * @return true if provisioned, false otherwise
     */
    bool is_provisioned() const
    {
        return provisioned_;
    }

private:

    /**
     * @brief Serves the provisioning page
     *
     * @param req HTTP request
     * @return ESP_OK
     */
    static esp_err_t serve_provisioning_page(
            httpd_req_t* req)
    {
        // Ensure that this becomes other thing when provisioned
        httpd_resp_send(req, (char*)_wifiprovisioning_html, _wifiprovisioning_html_len);

        return ESP_OK;
    }

    /**
     * @brief Event handler for Wi-Fi, IP and Provisioning related events
     *
     * @param arg Argument
     * @param event_base Event base
     * @param event_id Event ID
     * @param event_data Event data
     */
    static void event_handler(
            void* arg,
            esp_event_base_t event_base,
            int32_t event_id,
            void* event_data)
    {
        WifiManager* wifi_manager = (WifiManager*)arg;

        if (event_base == WIFI_PROV_EVENT)
        {
            switch (event_id){
                case WIFI_PROV_START:
                    ESP_LOGI(wifi_manager->TAG, "Provisioning started");
                    break;
                case WIFI_PROV_CRED_RECV: {
                    wifi_sta_config_t* wifi_sta_cfg = (wifi_sta_config_t*)event_data;
                    ESP_LOGI(wifi_manager->TAG, "Received Wi-Fi credentials"
                            "\n\tSSID     : %s\n\tPassword : %s",
                            (const char*) wifi_sta_cfg->ssid,
                            (const char*) wifi_sta_cfg->password);
                    break;
                }
                case WIFI_PROV_CRED_FAIL: {
                    wifi_prov_sta_fail_reason_t* reason = (wifi_prov_sta_fail_reason_t*)event_data;
                    ESP_LOGE(wifi_manager->TAG, "Provisioning failed!\n\tReason : %s"
                            "\n\tPlease reset to factory and retry provisioning",
                            (*reason == WIFI_PROV_STA_AUTH_ERROR) ?
                            "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
                    wifi_manager->retries_++;

                    if (wifi_manager->retries_ >= WIFI_CONNECTION_MAX_RETRIES)
                    {
                        ESP_LOGI(wifi_manager->TAG,
                                "Failed to connect with provisioned AP, reseting provisioned credentials");
                        wifi_prov_mgr_reset_sm_state_on_failure();
                        wifi_manager->retries_ = 0;
                    }

                    break;
                }
                case WIFI_PROV_CRED_SUCCESS:
                    ESP_LOGI(wifi_manager->TAG, "Provisioning successful");
                    wifi_manager->retries_ = 0;

                    break;
                case WIFI_PROV_END:
                    /* De-initialize manager once provisioning is finished */
                    wifi_prov_mgr_deinit();
                    break;
                default:
                    break;
            }
        }
        else if (event_base == WIFI_EVENT)
        {
            switch (event_id){
                case WIFI_EVENT_STA_START:
                    esp_wifi_connect();
                    break;
                case WIFI_EVENT_STA_DISCONNECTED:
                    ESP_LOGI(wifi_manager->TAG, "Disconnected. Connecting to the AP again...");
                    wifi_manager->retries_++;

                    if (wifi_manager->retries_ >= WIFI_CONNECTION_MAX_RETRIES)
                    {
                        ESP_LOGI(wifi_manager->TAG,
                                "Failed to connect with provisioned AP, reseting provisioned credentials");
                        wifi_prov_mgr_reset_provisioning();
                        esp_restart();
                    }

                    esp_wifi_connect();
                    break;
                case WIFI_EVENT_AP_STACONNECTED:
                    ESP_LOGI(wifi_manager->TAG, "SoftAP transport: Connected!");
                    break;
                case WIFI_EVENT_AP_STADISCONNECTED:
                    ESP_LOGI(wifi_manager->TAG, "SoftAP transport: Disconnected!");
                    break;
                default:
                    break;
            }
        }
        else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
        {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(wifi_manager->TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
            char buffer[40];
            char* ip = esp_ip4addr_ntoa(&event->ip_info.ip, buffer, sizeof(buffer));
            wifi_manager->ip_ = ip;
            xEventGroupSetBits(wifi_manager->wifi_event_group_, WIFI_CONNECTED_EVENT);
        }
    }

    EventGroupHandle_t wifi_event_group_;        ///< Event group for Wi-Fi events
    bool provisioned_;                           ///< Provisioned flag
    uint32_t retries_ = 0;                       ///< Number of retries
    HTTPServer& server_;                         ///< HTTP server
    std::string ssid_;                           ///< SSID
    std::string ip_;                             ///< IP address
    std::string provisioning_ssid_;              ///< Provisioning SSID
};
