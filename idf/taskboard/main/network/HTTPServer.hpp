/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <task/TaskExecutor.hpp>
#include <hal/TaskBoardDriver.hpp>
#include <hal/NonVolatileStorage.hpp>
#include <microros/MicroROSController.hpp>
#include <network/JSONHandler.hpp>

#include <esp_log.h>
#include <esp_http_server.h>

#include <freertos/task.h>
#include <freertos/queue.h>

#include <string>
#include <vector>

#include <network/webpages/web_index.h>

/**
 * @struct WebSocketServer
 *
 * @brief Websocket server for handling task board API, web interfaces
 */
struct WebSocketServer
{
    const char* TAG = "WebSocketServer";        ///< Logging tag

    /**
     * @brief Constructs a new WebSocketServer object
     */
    WebSocketServer()
    {
        client_mutex_ = xSemaphoreCreateMutex();
    }

    /**
     * @brief Sends data to all clients
     *
     * @param data Data to send
     * @param len Length of the data
     */
    void send_to_all_clients(
            const uint8_t* data,
            const size_t len)
    {
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = (uint8_t*)data;
        ws_pkt.len = len;
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;

        if (xSemaphoreTake(client_mutex_, portMAX_DELAY) == pdTRUE)
        {
            for (auto const& client : clients)
            {
                if (ESP_OK != httpd_ws_send_frame_async(client.hd, client.client_fd, &ws_pkt))
                {
                    // Remove client if sending failed
                    xSemaphoreGive(client_mutex_);
                    remove_client(client.hd, client.client_fd);

                    return;
                }
            }

            xSemaphoreGive(client_mutex_);
        }
    }

    /**
     * @brief Websocket handler
     *
     * @param req HTTP request
     *
     * @return esp_err_t
     */
    static esp_err_t ws_handler(
            httpd_req_t* req)
    {
        WebSocketServer* server = (WebSocketServer*)req->user_ctx;

        if (req->method == HTTP_GET)
        {
            ESP_LOGI(server->TAG, "Handshake done, the new connection was opened");
            server->add_client(req->handle, httpd_req_to_sockfd(req));

            return ESP_OK;
        }

        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

        // Receive frame
        esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);

        if (ret != ESP_OK)
        {
            ESP_LOGE(server->TAG, "httpd_ws_recv_frame failed with %d, removing client", ret);
            server->remove_client(req->handle, httpd_req_to_sockfd(req));

            return ret;
        }

        // If it's a PING, respond with PONG
        if (ws_pkt.type == HTTPD_WS_TYPE_PING)
        {
            ws_pkt.type = HTTPD_WS_TYPE_PONG;

            return httpd_ws_send_frame(req, &ws_pkt);
        }

        // If it's a CLOSE frame, remove the client
        if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
        {
            ESP_LOGI(server->TAG, "WebSocket close frame received");
            server->remove_client(req->handle, httpd_req_to_sockfd(req));

            return ESP_OK;
        }

        return ESP_OK;
    }

private:

    /**
     * @brief Adds a new client to the server
     *
     * @param hd HTTPD handle
     * @param client_fd Client file descriptor
     */
    struct ClientInfo
    {
        httpd_handle_t hd;
        int client_fd;
    };

    SemaphoreHandle_t client_mutex_ = NULL;    ///< Mutex for thread safety
    std::vector<ClientInfo> clients;           ///< List of clients

private:

    /**
     * @brief Adds a new client to the server
     *
     * @param hd HTTPD handle
     * @param client_fd Client file descriptor
     */
    void add_client(
            httpd_handle_t hd,
            int client_fd)
    {
        if (xSemaphoreTake(client_mutex_, portMAX_DELAY) == pdTRUE)
        {

            ClientInfo new_client;
            new_client.hd = hd;
            new_client.client_fd = client_fd;

            clients.push_back(new_client);

            xSemaphoreGive(client_mutex_);
        }
    }

    /**
     * @brief Removes a client from the server
     *
     * @param hd HTTPD handle
     * @param client_fd Client file descriptor
     */
    void remove_client(
            httpd_handle_t hd,
            int client_fd)
    {
        if (xSemaphoreTake(client_mutex_, portMAX_DELAY) == pdTRUE)
        {
            for (auto it = clients.begin(); it != clients.end(); it++)
            {
                if (it->hd == hd && it->client_fd == client_fd)
                {
                    clients.erase(it);
                    break;
                }
            }

            xSemaphoreGive(client_mutex_);
        }
    }

    struct WSDisconnectHandlerArg
    {
        httpd_handle_t* server;
        WebSocketServer* ws_server;
    };

    static void ws_disconnect_handler(
            void* arg,
            esp_event_base_t event_base,
            int32_t event_id,
            void* event_data)
    {
        WSDisconnectHandlerArg* ws_arg = (WSDisconnectHandlerArg*) arg;
        WebSocketServer* ws_server = ws_arg->ws_server;
        httpd_handle_t* server = ws_arg->server;

        struct httpd_data* hd = (struct httpd_data*) *server;
        int sockfd = (int) event_data;

        ws_server->remove_client(hd, sockfd);
    }

};

/**
 * @struct HTTPServer
 *
 * @brief HTTP server for handling task board API, web interfaces
 */
struct HTTPServer
{
    const char* TAG = "HTTPServer";     ///< Logging tag

    /**
     * @brief Constructs a new HTTPServer object
     *
     * @param task_board_driver Reference to the task board driver
     * @param task_executor Reference to the task executor
     * @param micro_ros_controller Reference to the micro-ROS controller
     * @param non_volatile_storage Reference to the non-volatile storage
     */
    HTTPServer(
            TaskBoardDriver& task_board_driver,
            TaskExecutor& task_executor,
            MicroROSController& micro_ros_controller,
            NonVolatileStorage& non_volatile_storage)
        : task_board_driver_(task_board_driver)
        , task_executor_(task_executor)
        , micro_ros_controller_(micro_ros_controller)
        , non_volatile_storage_(non_volatile_storage)
    {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.lru_purge_enable = true;
        config.core_id = 1;
        config.max_uri_handlers = 20;

        if (httpd_start(&server_, &config) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start file server!");
        }
    }

    /**
     * @brief Gets the HTTP server handle
     *
     * @return HTTP server handle
     */
    httpd_handle_t& get_handle()
    {
        return server_;
    }

    /**
     * @brief Sends data to all clients
     */
    void send_data_to_clients(
            uint8_t* data,
            size_t len)
    {
        ws_server_.send_to_all_clients(data, len);
    }

    /**
     * @brief Initializes the task board API
     */
    void initialize_taskboard_api()
    {
        httpd_uri_t get_index_uri = {};
        get_index_uri.uri      = "/";
        get_index_uri.method   = HTTP_GET;
        get_index_uri.handler  = HTTPServer::index_handler;
        get_index_uri.user_ctx = this;
        get_index_uri.is_websocket = false;

        httpd_uri_t get_taskboard_status_uri = {};
        get_taskboard_status_uri.uri      = "/taskboard_status";
        get_taskboard_status_uri.method   = HTTP_GET;
        get_taskboard_status_uri.handler  = HTTPServer::taskboard_status;
        get_taskboard_status_uri.user_ctx = this;

        httpd_uri_t get_system_status_uri = {};
        get_system_status_uri.uri      = "/system_status";
        get_system_status_uri.method   = HTTP_GET;
        get_system_status_uri.handler  = HTTPServer::system_status;
        get_system_status_uri.user_ctx = this;

        httpd_uri_t get_task_status_uri = {};
        get_task_status_uri.uri      = "/task_status";
        get_task_status_uri.method   = HTTP_GET;
        get_task_status_uri.handler  = HTTPServer::task_status;
        get_task_status_uri.user_ctx = this;

        httpd_uri_t get_logs_uri = {};
        get_logs_uri.uri      = "/leaderboard";
        get_logs_uri.method   = HTTP_GET;
        get_logs_uri.handler  = HTTPServer::logs_handler;
        get_logs_uri.user_ctx = this;

        httpd_uri_t options_uri = {};
        options_uri.uri      = "*";
        options_uri.method   = HTTP_OPTIONS;
        options_uri.handler  = HTTPServer::options_handler;
        options_uri.user_ctx = this;

        httpd_uri_t post_microros_uri = {};
        post_microros_uri.uri      = "/microros";
        post_microros_uri.method   = HTTP_POST;
        post_microros_uri.handler  = HTTPServer::microros_handler;
        post_microros_uri.user_ctx = this;

        httpd_uri_t options_micro_ros_uri = {};
        options_micro_ros_uri.uri      = "/microros";
        options_micro_ros_uri.method   = HTTP_OPTIONS;
        options_micro_ros_uri.handler  = HTTPServer::options_handler;
        options_micro_ros_uri.user_ctx = this;

        httpd_uri_t clear_logs_url = {};
        clear_logs_url.uri      = "/clear_logs";
        clear_logs_url.method   = HTTP_POST;
        clear_logs_url.handler  = HTTPServer::clear_logs;
        clear_logs_url.user_ctx = this;

        httpd_uri_t options_clear_logs_url = {};
        options_clear_logs_url.uri      = "/clear_logs";
        options_clear_logs_url.method   = HTTP_OPTIONS;
        options_clear_logs_url.handler  = HTTPServer::options_handler;
        options_clear_logs_url.user_ctx = this;

        httpd_register_uri_handler(server_, &get_index_uri);
        httpd_register_uri_handler(server_, &get_taskboard_status_uri);
        httpd_register_uri_handler(server_, &get_task_status_uri);
        httpd_register_uri_handler(server_, &get_system_status_uri);
        httpd_register_uri_handler(server_, &get_logs_uri);
        httpd_register_uri_handler(server_, &options_uri);
        httpd_register_uri_handler(server_, &post_microros_uri);
        httpd_register_uri_handler(server_, &options_micro_ros_uri);
        httpd_register_uri_handler(server_, &clear_logs_url);
        httpd_register_uri_handler(server_, &options_clear_logs_url);

        // Web sockets
        httpd_uri_t uri_ws = {};
        uri_ws.uri      = "/ws";
        uri_ws.method   = HTTP_GET;
        uri_ws.handler  = WebSocketServer::ws_handler;
        uri_ws.user_ctx = &ws_server_;
        uri_ws.is_websocket = true;

        httpd_register_uri_handler(server_, &uri_ws);

        ESP_LOGI(TAG, "Taskboard API initialized");
    }

private:

    /**
     * @brief Handler index
     *
     * @param req HTTP request
     */
    static esp_err_t index_handler(
            httpd_req_t* req)
    {
        // Add CORS headers
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // Allow all origins
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

        // Send index.html
        httpd_resp_send(req, (char*) _index_html, _index_html_len);

        return ESP_OK;
    }

    /**
     * @brief Handler for logs
     *
     * @param req HTTP request
     */
    static esp_err_t logs_handler(
            httpd_req_t* req)
    {
        // Add CORS headers
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // Allow all origins
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

        // Open file for reading
        FILE* file = fopen(NonVolatileStorage::log_file().c_str(), "r");

        if (file == NULL)
        {
            httpd_resp_send(req, "No logs available", -1);

            return ESP_OK;
        }

        // Read file and send it
        char buffer[128];

        while (fgets(buffer, sizeof(buffer), file) != NULL)
        {
            httpd_resp_send_chunk(req, buffer, strlen(buffer));
        }

        httpd_resp_send_chunk(req, NULL, 0);

        fclose(file);

        return ESP_OK;
    }

    /**
     * @brief Handler for task board status
     *
     * @param req HTTP request
     */
    static esp_err_t taskboard_status(
            httpd_req_t* req)
    {
        // Add CORS headers
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // Allow all origins
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

        // Generate JSON response
        HTTPServer* server = (HTTPServer*)req->user_ctx;
        JSONHandler json_handler(server->task_board_driver_.get_unique_id());

        json_handler.add_taskboard_status(server->task_board_driver_);

        char* status = json_handler.get_json_string();

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, status, strlen(status));

        return ESP_OK;
    }

    /**
     * @brief Handler for get system status
     *
     * @param req HTTP request
     */
    static esp_err_t system_status(
            httpd_req_t* req)
    {
        // Add CORS headers
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // Allow all origins
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

        // Generate JSON response
        HTTPServer* server = (HTTPServer*)req->user_ctx;
        JSONHandler json_handler(server->task_board_driver_.get_unique_id());

        // Add microros info
        json_handler.add_microros_info(server->micro_ros_controller_);

        // Add FreeRTOS status
        json_handler.add_freertos_summary();

        char* status = json_handler.get_json_string();

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, status, strlen(status));

        return ESP_OK;
    }

    /**
     * @brief Handler for task status
     *
     * @param req HTTP request
     */
    static esp_err_t task_status(
            httpd_req_t* req)
    {
        // Add CORS headers
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // Allow all origins
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

        // Generate JSON response
        HTTPServer* server = (HTTPServer*)req->user_ctx;
        JSONHandler json_handler(server->task_board_driver_.get_unique_id());

        // Add task status if there is a task running
        server->task_executor_.execute_operation_on_task(portMAX_DELAY,
                [&json_handler](Task* task, Task* precondition)
                {
                    if (task != nullptr)
                    {
                        json_handler.add_task_status(*task, precondition);
                    }
                });

        char* status = json_handler.get_json_string();

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, status, strlen(status));

        return ESP_OK;
    }

    /**
     * @brief Handler for options requests
     *
     * @param req HTTP request
     */
    static esp_err_t options_handler(
            httpd_req_t* req)
    {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Content-Length");
        httpd_resp_set_hdr(req, "Access-Control-Max-Age", "3600");

        // For OPTIONS requests, we need to send a 204 No Content response
        httpd_resp_set_status(req, "204 No Content");
        httpd_resp_send(req, NULL, 0);

        return ESP_OK;
    }

    /**
     * @brief Handler for setting micro-ROS agent address
     *
     * @param req HTTP request
     */
    static esp_err_t microros_handler(
            httpd_req_t* req)
    {
        // Set CORS headers consistently
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Content-Length");
        httpd_resp_set_type(req, "application/json");  // Add content type header

        HTTPServer* server = (HTTPServer*)req->user_ctx;

        char content[100];
        const size_t recv_size = req->content_len < sizeof(content) ? req->content_len : sizeof(content);
        int ret = httpd_req_recv(req, content, recv_size);

        if (ret <= 0)
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
            {
                httpd_resp_send_408(req);
            }

            return ESP_FAIL;
        }

        cJSON* json = cJSON_Parse(content);

        if (json == NULL)
        {
            const char* error_response = "{\"error\": \"Invalid JSON\"}";
            httpd_resp_send(req, error_response, strlen(error_response));

            return ESP_OK;
        }

        cJSON* agent_ip = cJSON_GetObjectItem(json, "agent_ip");
        cJSON* agent_port = cJSON_GetObjectItem(json, "agent_port");

        if (agent_ip == NULL || agent_port == NULL)
        {
            const char* error_response = "{\"error\": \"Missing required fields\"}";
            httpd_resp_send(req, error_response, strlen(error_response));
            cJSON_Delete(json);

            return ESP_OK;
        }

        if (!cJSON_IsString(agent_ip) || !cJSON_IsString(agent_port))
        {
            const char* error_response = "{\"error\": \"Invalid field types\"}";
            httpd_resp_send(req, error_response, strlen(error_response));
            cJSON_Delete(json);

            return ESP_OK;
        }

        server->micro_ros_controller_.set_agent_ip(agent_ip->valuestring, agent_port->valuestring);

        // Send success response
        const char* success_response = "{\"status\": \"success\"}";
        httpd_resp_send(req, success_response, strlen(success_response));

        cJSON_Delete(json);

        return ESP_OK;
    }

    /**
     * @brief Handler for clearing logs
     *
     * @param req HTTP request
     */
    static esp_err_t clear_logs(
            httpd_req_t* req)
    {
        // Set CORS headers consistently
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Content-Length");
        httpd_resp_set_type(req, "application/json");  // Add content type header

        HTTPServer* server = (HTTPServer*)req->user_ctx;
        server->non_volatile_storage_.clear_log();

        httpd_resp_send(req, "{\"status\": \"success\"}", -1);

        return ESP_OK;
    }

private:

    httpd_handle_t server_;                         ///< HTTP server handle
    TaskBoardDriver& task_board_driver_;            ///< Reference to the task board driver
    TaskExecutor& task_executor_;                   ///< Reference to the task executor
    MicroROSController& micro_ros_controller_;      ///< Reference to the micro-ROS controller
    NonVolatileStorage& non_volatile_storage_;      ///< Reference to the non-volatile storage
    WebSocketServer ws_server_;                     ///< Web socket server
};
