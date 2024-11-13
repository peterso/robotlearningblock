/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <microros/MicroROSTypes.hpp>

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/init_options.h>
#include <rmw_microros/timing.h>
#include <rmw_microros/ping.h>

#include <nvs_flash.h>
#include <esp_log.h>

#include <string>
#include <cstdint>
#include <functional>

struct MicroROSController
{
    const char * TAG = "MicroROSController";

    using HandleGoal = std::function<rcl_ret_t(rclc_action_goal_handle_t *)>;
    using HandleCancel = std::function<bool(rclc_action_goal_handle_t *)>;

    MicroROSController()
    {
        allocator_ = rcl_get_default_allocator();

        init_options_ = rcl_get_zero_initialized_init_options();
        if(RCL_RET_OK != rcl_init_options_init(&init_options_, allocator_))
        {
            ESP_LOGI(TAG, "Failed to init options");
        }
        rwm_options_ = rcl_init_options_get_rmw_init_options(&init_options_);

        // Open NVM
        if(ESP_OK != nvs_open("microros", NVS_READWRITE, &nvm_handle_))
        {
            ESP_LOGE(TAG, "Failed to open NVM");
        }

        // Configure micro-ROS Agent
        load_agent_address_from_nvm();
        rmw_uros_options_set_udp_address(agent_ip_.c_str(), agent_port_.c_str(), rwm_options_);

        ESP_LOGI(TAG, "MicroROSController initialized on %s:%s", agent_ip_.c_str(), agent_port_.c_str());

        // Configure timed operations
        ping_agent_operation_ = new TimedOperation(500, [this](){
            this->check_agent_status();
        });

    }

    ~MicroROSController()
    {
        nvs_close(nvm_handle_);

        // TODO(pgarrido): Free memory if this object is destroyed somehow
    }

    void set_agent_ip(const char * ip, const char * port)
    {
        ESP_LOGI(TAG, "Setting agent address to %s:%s", ip, port);

        agent_ip_ = ip;
        agent_port_ = port;

        store_agent_address_to_nvm();

        rmw_uros_options_set_udp_address(agent_ip_.c_str(), agent_port_.c_str(), rwm_options_);
    }

    const std::string & get_agent_ip() const
    {
        return agent_ip_;
    }

    const std::string & get_agent_port() const
    {
        return agent_port_;
    }

    bool is_agent_connected() const
    {
        return state_ == State::AGENT_CONNECTED;
    }

    void publish_board_status(const MicroROSTypes::TaskBoardStatus & status)
    {
        if(state_ == State::AGENT_CONNECTED)
        {
            if(RCL_RET_OK != rcl_publish(&publisher_, &status.get_microros_msg(), NULL))
            {
                ESP_LOGE(TAG, "Failed to publish");
            }
        }
    }

    void update()
    {
        switch (state_) {
        case State::AGENT_AVAILABLE:
            state_ = (true == create_entities()) ? State::AGENT_CONNECTED : State::WAITING_AGENT;
            if (state_ == State::WAITING_AGENT) {
                ESP_LOGE(TAG, "Failed to create entities");
                destroy_entities();
            }
            else
            {
                ESP_LOGI(TAG, "Entities created");
            }
            break;
        case State::AGENT_CONNECTED:
            rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
            break;
        case State::AGENT_DISCONNECTED:
            ESP_LOGI(TAG, "Agent disconnected, destroying entities");
            destroy_entities();
            state_ = State::WAITING_AGENT;
            break;
        case State::WAITING_AGENT:
        default:
            break;
        }

        ping_agent_operation_->update();
    }

    void set_handle_goal(HandleGoal handle_goal)
    {
        handle_goal_ = handle_goal;
    }

    void set_handle_cancel(HandleCancel handle_cancel)
    {
        handle_cancel_ = handle_cancel;
    }

    void publish_feedback(const MicroROSTypes::FeedbackMessage & feedback, const rclc_action_goal_handle_t * goal_handle)
    {
        if(state_ == State::AGENT_CONNECTED)
        {
            if(RCL_RET_OK != rclc_action_publish_feedback((rclc_action_goal_handle_t*) goal_handle, (void*) &feedback.get_microros_msg()))
            {
                ESP_LOGE(TAG, "Failed to publish feedback");
            }
        }
    }

    void publish_goal_result(const MicroROSTypes::ResultMessage & result, const rclc_action_goal_handle_t * goal_handle, rcl_action_goal_state_t state)
    {
        if(state_ == State::AGENT_CONNECTED)
        {
            if(RCL_RET_OK != rclc_action_send_result((rclc_action_goal_handle_t*) goal_handle, state, (void*) &result.get_microros_msg()))
            {
                ESP_LOGE(TAG, "Failed to publish result");
            }
        }
    }

    void publish_task_status(const MicroROSTypes::TaskStatus & task_status)
    {
        if(state_ == State::AGENT_CONNECTED)
        {
            if(RCL_RET_OK != rcl_publish(&task_status_publisher_, &task_status.get_microros_msg(), NULL))
            {
                ESP_LOGE(TAG, "Failed to publish task status");
            }
        }
    }

private:

    HandleGoal handle_goal_;

    static rcl_ret_t handle_goal(rclc_action_goal_handle_t * goal_handle, void * context)
    {
        MicroROSController * node = static_cast<MicroROSController*>(context);
        return node->handle_goal_(goal_handle);

        ESP_LOGI(node->TAG, "Received goal");

        return RCL_RET_ACTION_GOAL_REJECTED;
    }

    HandleCancel handle_cancel_;

    static bool handle_cancel(rclc_action_goal_handle_t * goal_handle, void * context)
    {
        MicroROSController * node = static_cast<MicroROSController*>(context);
        return node->handle_cancel_(goal_handle);
    }

    bool create_entities()
    {
        // create init_options
        if(RCL_RET_OK != rclc_support_init_with_options(&support_, 0, NULL, &init_options_, &allocator_))
        {
            ESP_LOGI(TAG, "Failed to init support");
            return false;
        }

        // create node
        if(RCL_RET_OK != rclc_node_init_default(&node_, "roboton_taskboard", "", &support_))
        {
            ESP_LOGI(TAG, "Failed to init node");
            return false;
        }

        // create publisher
        if(RCL_RET_OK != rclc_publisher_init_best_effort(
            &publisher_,
            &node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(roboton_taskboard_msgs, msg, TaskBoardStatus),
            "roboton_taskboard_status"))
        {
            ESP_LOGI(TAG, "Failed to init publisher");
            return false;
        }

        // create task status publisher
        if(RCL_RET_OK != rclc_publisher_init_best_effort(
            &task_status_publisher_,
            &node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(roboton_taskboard_msgs, msg, TaskStatus),
            "roboton_task_status"))
        {
            ESP_LOGI(TAG, "Failed to init task status publisher");
            return false;
        }

        // Create action service
        if(RCL_RET_OK != rclc_action_server_init_default(
            &action_server_,
            &node_,
            &support_,
            ROSIDL_GET_ACTION_TYPE_SUPPORT(roboton_taskboard_msgs, ExecuteTask),
            "taskboard_execute_task"))
        {
            ESP_LOGI(TAG, "Failed to init action server");
            return false;
        }


        // create executor
        if(RCL_RET_OK != rclc_executor_init(&executor_, &support_.context, 1, &allocator_))
        {
            ESP_LOGI(TAG, "Failed to init executor");
            return false;
        }

        if(RCL_RET_OK != rclc_executor_add_action_server(
            &executor_,
            &action_server_,
            1,
            const_cast<void *>(static_cast<const void*>(&action_goal_request_.get_microros_msg())),
            sizeof(roboton_taskboard_msgs__action__ExecuteTask_SendGoal_Request),
            handle_goal,
            handle_cancel,
            this))
        {
            ESP_LOGI(TAG, "Failed to add action server");
            return false;
        }

        return true;
    }

    void destroy_entities()
    {
        rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support_.context);
        (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

        if(RCL_RET_OK != rcl_publisher_fini(&publisher_, &node_))
        {
            ESP_LOGE(TAG, "Failed to fini publisher");
        }

        if(RCL_RET_OK != rcl_publisher_fini(&task_status_publisher_, &node_))
        {
            ESP_LOGE(TAG, "Failed to fini task status publisher");
        }

        if(RCL_RET_OK != rclc_action_server_fini(&action_server_, &node_))
        {
            ESP_LOGE(TAG, "Failed to fini action server");
        }

        if(RCL_RET_OK != rclc_executor_fini(&executor_))
        {
            ESP_LOGE(TAG, "Failed to fini executor");
        }

        if(RCL_RET_OK != rcl_node_fini(&node_))
        {
            ESP_LOGE(TAG, "Failed to fini node");
        }

        if(RCL_RET_OK != rclc_support_fini(&support_))
        {
            ESP_LOGE(TAG, "Failed to fini support");
        }
    }

    void load_agent_address_from_nvm()
    {
        size_t agentaddress_len = 0;
        switch (nvs_get_str(nvm_handle_, "agent", NULL, &agentaddress_len))
        {
            case ESP_OK:
            {
                char buffer[100];
                nvs_get_str(nvm_handle_, "agent", buffer, &agentaddress_len);
                std::string agent_address(buffer);
                ESP_LOGI(TAG, "NVM agent address: %s", agent_address.c_str());
                // Split by :
                size_t pos = agent_address.find(":");
                if (pos != std::string::npos)
                {
                    agent_ip_ = agent_address.substr(0, pos);
                    agent_port_ = agent_address.substr(pos + 1);
                }
            }
            break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGI(TAG, "NVM agent address not found");
                break;
        }
    }

    void store_agent_address_to_nvm()
    {
        if(ESP_OK != nvs_set_str(nvm_handle_, "agent", (agent_ip_ + ":" + agent_port_).c_str()))
        {
            ESP_LOGE(TAG, "Failed to store in NVM");
        }
        else
        {
            if(ESP_OK != nvs_commit(nvm_handle_))
            {
                ESP_LOGE(TAG, "Failed to commit NVM");
            }
        }
    }

    void check_agent_status()
    {
        switch (state_)
        {
        case State::WAITING_AGENT:
            state_ = (RMW_RET_OK == rmw_uros_ping_agent_options(100, 10, rwm_options_)) ? State::AGENT_AVAILABLE : State::WAITING_AGENT;
            break;
        case State::AGENT_CONNECTED:
            state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 10)) ? State::AGENT_CONNECTED : State::AGENT_DISCONNECTED;
        default:
            break;
        }
    }

    std::string agent_ip_ = CONFIG_MICRO_ROS_AGENT_IP;
    std::string agent_port_ = CONFIG_MICRO_ROS_AGENT_PORT;

    nvs_handle_t nvm_handle_;

    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rcl_publisher_t publisher_;
    rcl_publisher_t task_status_publisher_;
    rclc_action_server_t action_server_;
    rclc_executor_t executor_;

    static constexpr size_t MAX_TASK_NAME_SIZE = 64;
    static constexpr size_t MAX_SENSOR_NAME_SIZE = 64;
    static constexpr size_t MAX_STEPS_PER_TASK = 15;
    MicroROSTypes::SendGoalRequest action_goal_request_ = {MAX_TASK_NAME_SIZE, MAX_SENSOR_NAME_SIZE, MAX_STEPS_PER_TASK};

    enum class State {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    };

    State state_ = State::WAITING_AGENT;

    rcl_init_options_t init_options_;
    rmw_init_options_t * rwm_options_;

    TimedOperation * ping_agent_operation_;
};
