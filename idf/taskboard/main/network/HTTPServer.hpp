/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <task/TaskExecutor.hpp>
#include <hal/TaskBoardDriver.hpp>
#include <hal/NonVolatileStorage.hpp>
#include <microros/MicroROSController.hpp>
#include <network/JSONHandler.hpp>

#include <esp_log.h>
#include <esp_http_server.h>

#include <string>

#include <network/webpages/web_index.h>

struct HTTPServer
{
    const char *TAG = "HTTPServer";

    HTTPServer(TaskBoardDriver & task_board_driver, TaskExecutor & task_executor, MicroROSController & micro_ros_controller, NonVolatileStorage & non_volatile_storage)
    : task_board_driver_(task_board_driver)
    , task_executor_(task_executor)
    , micro_ros_controller_(micro_ros_controller)
    , non_volatile_storage_(non_volatile_storage)
    {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.lru_purge_enable = true;
        config.core_id = 1;
        config.max_uri_handlers = 20;

        if (httpd_start(&server_, &config) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start file server!");
        }
    }

    httpd_handle_t & get_handle()
    {
        return server_;
    }

    void initialize_taskboard_api()
    {
        httpd_uri_t get_index_uri = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = HTTPServer::index_handler,
            .user_ctx = this
        };


        httpd_uri_t get_api_uri = {
            .uri      = "/api",
            .method   = HTTP_GET,
            .handler  = HTTPServer::api_handler,
            .user_ctx = this
        };

        httpd_uri_t get_logs_uri = {
            .uri      = "/logs",
            .method   = HTTP_GET,
            .handler  = HTTPServer::logs_handler,
            .user_ctx = this
        };

        httpd_uri_t options_uri = {
            .uri      = "*",
            .method   = HTTP_OPTIONS,
            .handler  = HTTPServer::options_handler,
            .user_ctx = this
        };

        httpd_uri_t post_microros_uri = {
            .uri      = "/microros",
            .method   = HTTP_POST,
            .handler  = HTTPServer::microros_handler,
            .user_ctx = this
        };

        httpd_uri_t options_micro_ros_uri = {
            .uri      = "/microros",
            .method   = HTTP_OPTIONS,
            .handler  = HTTPServer::options_handler,
            .user_ctx = this
        };

        httpd_uri_t clear_logs_url = {
            .uri      = "/clear_logs",
            .method   = HTTP_POST,
            .handler  = HTTPServer::clear_logs,
            .user_ctx = this
        };

        httpd_uri_t options_clear_logs_url = {
            .uri      = "/clear_logs",
            .method   = HTTP_OPTIONS,
            .handler  = HTTPServer::options_handler,
            .user_ctx = this
        };

        httpd_register_uri_handler(server_, &get_index_uri);
        httpd_register_uri_handler(server_, &get_api_uri);
        httpd_register_uri_handler(server_, &get_logs_uri);
        httpd_register_uri_handler(server_, &options_uri);
        httpd_register_uri_handler(server_, &post_microros_uri);
        httpd_register_uri_handler(server_, &options_micro_ros_uri);
        httpd_register_uri_handler(server_, &clear_logs_url);
        httpd_register_uri_handler(server_, &options_clear_logs_url);

        ESP_LOGI(TAG, "Taskboard API initialized");
    }

    static esp_err_t index_handler(httpd_req_t *req)
    {
        // Add CORS headers
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // Allow all origins
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

        // Send index.html
        httpd_resp_send(req, (char*) _index_html, _index_html_len);

        return ESP_OK;
    }

    static esp_err_t logs_handler(httpd_req_t *req)
    {
        // Add CORS headers
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // Allow all origins
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

        // Open file for reading
        FILE * file = fopen(NonVolatileStorage::log_file().c_str(), "r");

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

    static esp_err_t api_handler(httpd_req_t *req)
    {
         // Add CORS headers
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // Allow all origins
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

        // Generate JSON response
        HTTPServer * server = (HTTPServer *)req->user_ctx;
        JSONHandler json_handler(server->task_board_driver_.get_unique_id());

        for (size_t i = 0; i < server->task_board_driver_.get_sensor_count(); i++)
        {
            const SensorReader * sensor = server->task_board_driver_.get_sensor(i);

            if (sensor != nullptr)
            {
                json_handler.add_sensor_measure(sensor->name(), sensor->read());
            }
        }

        // Add microros info
        json_handler.add_microros_info(server->micro_ros_controller_);

        // Add task status if there is a task
        const Task * current_task = server->task_executor_.current_task();
        if (nullptr != current_task)
        {
            json_handler.add_task_status(*current_task);
        }

        char * status = json_handler.get_json_string();

        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, status, strlen(status));

        return ESP_OK;
    }

    static esp_err_t options_handler(httpd_req_t *req)
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

    static esp_err_t microros_handler(httpd_req_t *req)
    {
        // Set CORS headers consistently
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Content-Length");
        httpd_resp_set_type(req, "application/json");  // Add content type header

        HTTPServer * server = (HTTPServer *)req->user_ctx;

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

        cJSON * json = cJSON_Parse(content);

        if (json == NULL)
        {
            const char* error_response = "{\"error\": \"Invalid JSON\"}";
            httpd_resp_send(req, error_response, strlen(error_response));
            return ESP_OK;
        }

        cJSON * agent_ip = cJSON_GetObjectItem(json, "agent_ip");
        cJSON * agent_port = cJSON_GetObjectItem(json, "agent_port");

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

    static esp_err_t clear_logs(httpd_req_t *req)
    {
        // Set CORS headers consistently
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Content-Length");
        httpd_resp_set_type(req, "application/json");  // Add content type header

        HTTPServer * server = (HTTPServer *)req->user_ctx;
        server->non_volatile_storage_.clear_log();

       httpd_resp_send(req, "{\"status\": \"success\"}", -1);

        return ESP_OK;
    }

private:
    httpd_handle_t server_;
    TaskBoardDriver & task_board_driver_;
    TaskExecutor & task_executor_;
    MicroROSController & micro_ros_controller_;
    NonVolatileStorage & non_volatile_storage_;
};
