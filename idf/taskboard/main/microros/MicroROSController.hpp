/**
 * Robothon Task Board Firmware
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
#include <rmw_microros/time_sync.h>

#include <nvs_flash.h>
#include <esp_log.h>

#include <string>
#include <cstdint>
#include <functional>

// Ping operation configuration
constexpr uint32_t PING_AGENT_TIMEOUT_MS = 200;
constexpr uint32_t PING_AGENT_RETRIES = 10;
constexpr uint32_t PING_ON_WAIT_AGENT_TIMEOUT_MS = 50;
constexpr uint32_t PING_ON_WAIT_AGENT_RETRIES = 5;

/**
 * @struct MicroROSController
 *
 * @brief Controller for managing micro-ROS communication with ROS2
 */
struct MicroROSController
{
    const char* TAG = "MicroROSController";         ///< Logging tag

    /// @brief Function pointer for handling incoming action goals
    using HandleGoal = std::function<rcl_ret_t(rclc_action_goal_handle_t*)>;

    /// @brief Function pointer for handling incoming action goal cancellations
    using HandleCancel = std::function<bool (rclc_action_goal_handle_t*)>;

    /**
     * @brief Constructs a new MicroROSController object
     */
    MicroROSController()
    {
        allocator_ = rcl_get_default_allocator();

        init_options_ = rcl_get_zero_initialized_init_options();

        if (RCL_RET_OK != rcl_init_options_init(&init_options_, allocator_))
        {
            ESP_LOGI(TAG, "Failed to init options");
        }

        rwm_options_ = rcl_init_options_get_rmw_init_options(&init_options_);

        // Open NVM for agent address storage
        if (ESP_OK != nvs_open("microros", NVS_READWRITE, &nvm_handle_))
        {
            ESP_LOGE(TAG, "Failed to open NVM");
        }

        // Configure micro-ROS Agent
        load_agent_address_from_nvm();
        rmw_uros_options_set_udp_address(agent_ip_.c_str(), agent_port_.c_str(), rwm_options_);

        ESP_LOGI(TAG, "MicroROSController initialized on %s:%s", agent_ip_.c_str(), agent_port_.c_str());

        // Configure timed operations
        ping_agent_operation_ = new TimedOperation(500, [this]()
                        {
                            this->check_agent_status();
                        });

    }

    /**
     * @brief Destroys the MicroROSController object
     */
    ~MicroROSController()
    {
        nvs_close(nvm_handle_);

        // TODO(pgarrido): Free memory if this object is destroyed somehow
    }

    /**
     * @brief Sets the agent address for micro-ROS communication
     *
     * @param ip IP address of the agent
     * @param port Port of the agent
     */
    void set_agent_ip(
            const char* ip,
            const char* port)
    {
        ESP_LOGI(TAG, "Setting agent address to %s:%s", ip, port);

        agent_ip_ = ip;
        agent_port_ = port;

        store_agent_address_to_nvm();

        rmw_uros_options_set_udp_address(agent_ip_.c_str(), agent_port_.c_str(), rwm_options_);
    }

    /**
     * @brief Gets the agent address for micro-ROS communication
     *
     * @return IP address of the agent
     */
    const std::string& get_agent_ip() const
    {
        return agent_ip_;
    }

    /**
     * @brief Gets the agent port for micro-ROS communication
     *
     * @return Port of the agent
     */
    const std::string& get_agent_port() const
    {
        return agent_port_;
    }

    /**
     * @brief Checks the status of the micro-ROS agent
     *
     * @return true if agent is available, false otherwise
     */
    bool is_agent_connected() const
    {
        return state_ == State::AGENT_CONNECTED;
    }

    /**
     * @brief Publish the current task board status to ROS
     *
     * @param status Task board status message
     */
    void publish_taskboard_status(
            const MicroROSTypes::TaskBoardStatus& status)
    {
        if (state_ == State::AGENT_CONNECTED)
        {
            if (RCL_RET_OK != rcl_publish(&taskboard_status_publisher_, &status.get_microros_msg(), NULL))
            {
                ESP_LOGE(TAG, "Failed to publish");
            }
        }
    }

    /**
     * @brief Publish the current task status to ROS
     *
     * @param task_status Task status message
     */
    void publish_task_status(
            const MicroROSTypes::TaskStatus& task_status)
    {
        if (state_ == State::AGENT_CONNECTED)
        {
            if (RCL_RET_OK != rcl_publish(&task_status_publisher_, &task_status.get_microros_msg(), NULL))
            {
                ESP_LOGE(TAG, "Failed to publish task status");
            }
        }
    }

    /**
     * @brief Sets the function pointer for handling incoming action goals
     */
    void set_handle_goal(
            HandleGoal handle_goal)
    {
        handle_goal_ = handle_goal;
    }

    /**
     * @brief Sets the function pointer for handling incoming action goal cancellations
     */
    void set_handle_cancel(
            HandleCancel handle_cancel)
    {
        handle_cancel_ = handle_cancel;
    }

    /**
     * @brief Publish feedback on a ROS 2 action goal
     *
     * @param feedback Feedback message to publish
     * @param goal_handle Handle to the goal
     */
    void publish_feedback(
            const MicroROSTypes::FeedbackMessage& feedback,
            const rclc_action_goal_handle_t* goal_handle)
    {
        if (state_ == State::AGENT_CONNECTED)
        {
            if (RCL_RET_OK !=
                    rclc_action_publish_feedback((rclc_action_goal_handle_t*) goal_handle,
                    (void*) &feedback.get_microros_msg()))
            {
                ESP_LOGE(TAG, "Failed to publish feedback");
            }
        }
    }

    /**
     * @brief Publish result on a ROS 2 action goal
     *
     * @param result Result message to publish
     * @param goal_handle Handle to the goal
     * @param state Current goal state
     */
    void publish_goal_result(
            const MicroROSTypes::ResultMessage& result,
            const rclc_action_goal_handle_t* goal_handle,
            rcl_action_goal_state_t state)
    {
        if (state_ == State::AGENT_CONNECTED)
        {
            if (RCL_RET_OK !=
                    rclc_action_send_result((rclc_action_goal_handle_t*) goal_handle, state,
                    (void*) &result.get_microros_msg()))
            {
                ESP_LOGE(TAG, "Failed to publish result");
            }
        }
    }

    /**
     * @brief Updates the micro-ROS controller state
     *
     * @details This function should be called periodically to update the controller state
     */
    void update()
    {
        switch (state_){
            case State::AGENT_AVAILABLE:
                state_ = (true == create_entities()) ? State::AGENT_CONNECTED : State::WAITING_AGENT;

                if (state_ == State::WAITING_AGENT)
                {
                    ESP_LOGE(TAG, "Failed to create entities");
                    destroy_entities();
                }
                else
                {
                    ESP_LOGI(TAG, "Entities created");
                    sync_time_from_agent();
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

private:

    /// @brief Internall goal handling function reference
    HandleGoal handle_goal_;

    /// @brief Handle goal function dispatcher
    static rcl_ret_t handle_goal(
            rclc_action_goal_handle_t* goal_handle,
            void* context)
    {
        MicroROSController* node = static_cast<MicroROSController*>(context);

        ESP_LOGI(node->TAG, "Received goal");

        return node->handle_goal_(goal_handle);
    }

    /// @brief Internall goal cancellation function reference
    HandleCancel handle_cancel_;

    /// @brief Handle goal cancellation function dispatcher
    static bool handle_cancel(
            rclc_action_goal_handle_t* goal_handle,
            void* context)
    {
        MicroROSController* node = static_cast<MicroROSController*>(context);

        ESP_LOGI(node->TAG, "Received cancel");

        return node->handle_cancel_(goal_handle);
    }

    /**
     * @brief Create micro-ROS entities
     */
    bool create_entities()
    {
        // create init_options
        if (RCL_RET_OK != rclc_support_init_with_options(&support_, 0, NULL, &init_options_, &allocator_))
        {
            ESP_LOGI(TAG, "Failed to init support");

            return false;
        }

        // create node
        if (RCL_RET_OK != rclc_node_init_default(&node_, "robothon_taskboard", "", &support_))
        {
            ESP_LOGI(TAG, "Failed to init node");

            return false;
        }

        // create publisher
        if (RCL_RET_OK != rclc_publisher_init_best_effort(
                    &taskboard_status_publisher_,
                    &node_,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(robothon_taskboard_msgs, msg, TaskBoardStatus),
                    "robothon_taskboard_status"))
        {
            ESP_LOGI(TAG, "Failed to init publisher");

            return false;
        }

        // create task status publisher
        if (RCL_RET_OK != rclc_publisher_init_best_effort(
                    &task_status_publisher_,
                    &node_,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(robothon_taskboard_msgs, msg, TaskStatus),
                    "robothon_task_status"))
        {
            ESP_LOGI(TAG, "Failed to init task status publisher");

            return false;
        }

        // Create action service
        if (RCL_RET_OK != rclc_action_server_init_default(
                    &action_server_,
                    &node_,
                    &support_,
                    ROSIDL_GET_ACTION_TYPE_SUPPORT(robothon_taskboard_msgs, ExecuteTask),
                    "taskboard_execute_task"))
        {
            ESP_LOGI(TAG, "Failed to init action server");

            return false;
        }

        // create executor
        if (RCL_RET_OK != rclc_executor_init(&executor_, &support_.context, 1, &allocator_))
        {
            ESP_LOGI(TAG, "Failed to init executor");

            return false;
        }

        if (RCL_RET_OK != rclc_executor_add_action_server(
                    &executor_,
                    &action_server_,
                    1,
                    const_cast<void*>(static_cast<const void*>(&action_goal_request_.get_microros_msg())),
                    sizeof(robothon_taskboard_msgs__action__ExecuteTask_SendGoal_Request),
                    handle_goal,
                    handle_cancel,
                    this))
        {
            ESP_LOGI(TAG, "Failed to add action server");

            return false;
        }

        return true;
    }

    /**
     * @brief Syncronize time with the micro-ROS agent
     */
    void sync_time_from_agent()
    {
        if (RCL_RET_OK != rmw_uros_sync_session(500))
        {
            ESP_LOGE(TAG, "Failed to sync time");
        }
    }

    /**
     * @brief Destroy micro-ROS entities
     */
    void destroy_entities()
    {
        rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support_.context);
        (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

        if (RCL_RET_OK != rcl_publisher_fini(&taskboard_status_publisher_, &node_))
        {
            ESP_LOGE(TAG, "Failed to fini publisher");
        }

        if (RCL_RET_OK != rcl_publisher_fini(&task_status_publisher_, &node_))
        {
            ESP_LOGE(TAG, "Failed to fini task status publisher");
        }

        if (RCL_RET_OK != rclc_action_server_fini(&action_server_, &node_))
        {
            ESP_LOGE(TAG, "Failed to fini action server");
        }

        if (RCL_RET_OK != rclc_executor_fini(&executor_))
        {
            ESP_LOGE(TAG, "Failed to fini executor");
        }

        if (RCL_RET_OK != rcl_node_fini(&node_))
        {
            ESP_LOGE(TAG, "Failed to fini node");
        }

        if (RCL_RET_OK != rclc_support_fini(&support_))
        {
            ESP_LOGE(TAG, "Failed to fini support");
        }
    }

    /**
     * @brief Load agent address from Non Volatile Memory
     */
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

    /**
     * @brief Store agent address to Non Volatile Memory
     */
    void store_agent_address_to_nvm()
    {
        if (ESP_OK != nvs_set_str(nvm_handle_, "agent", (agent_ip_ + ":" + agent_port_).c_str()))
        {
            ESP_LOGE(TAG, "Failed to store in NVM");
        }
        else
        {
            if (ESP_OK != nvs_commit(nvm_handle_))
            {
                ESP_LOGE(TAG, "Failed to commit NVM");
            }
        }
    }

    /**
     * @brief Check the status of the micro-ROS agent by means of a ping operation
     */
    void check_agent_status()
    {
        switch (state_)
        {
            case State::WAITING_AGENT:
                state_ =
                        (RMW_RET_OK ==
                        rmw_uros_ping_agent_options(PING_ON_WAIT_AGENT_TIMEOUT_MS, PING_ON_WAIT_AGENT_RETRIES,
                        rwm_options_)) ? State::AGENT_AVAILABLE : State::WAITING_AGENT;
                break;
            case State::AGENT_CONNECTED:
                state_ =
                        (RMW_RET_OK ==
                        rmw_uros_ping_agent(PING_AGENT_TIMEOUT_MS,
                        PING_AGENT_RETRIES)) ? State::AGENT_CONNECTED : State::AGENT_DISCONNECTED;
            default:
                break;
        }
    }

    std::string agent_ip_ = CONFIG_MICRO_ROS_AGENT_IP;              ///< IP address of the micro-ROS agent
    std::string agent_port_ = CONFIG_MICRO_ROS_AGENT_PORT;          ///< Port of the micro-ROS agent

    nvs_handle_t nvm_handle_;                                       ///< Non Volatile Memory handle

    // micro-ROS entities
    rcl_init_options_t init_options_;                               ///< Initialization options for micro-ROS entities
    rmw_init_options_t* rwm_options_;                               ///< RMW initialization options for micro-ROS entities
    rcl_allocator_t allocator_;                                     ///< Allocator for micro-ROS entities
    rclc_support_t support_;                                        ///< Support structure for micro-ROS entities
    rcl_node_t node_;                                               ///< micro-ROS node
    rcl_publisher_t taskboard_status_publisher_;                    ///< Task board status publisher
    rcl_publisher_t task_status_publisher_;                         ///< Task status publisher
    rclc_action_server_t action_server_;                            ///< Action server for task execution
    rclc_executor_t executor_;                                      ///< micro-ROS executor

    // Action goal request memory
    static constexpr size_t MAX_TASK_NAME_SIZE = 64;                ///< Maximum size of received a task name
    static constexpr size_t MAX_SENSOR_NAME_SIZE = 64;              ///< Maximum size of a received sensor name
    static constexpr size_t MAX_STEPS_PER_TASK = 15;                ///< Maximum number of steps per task

    /// @brief Action goal request structure
    MicroROSTypes::SendGoalRequest action_goal_request_ =
    {MAX_TASK_NAME_SIZE, MAX_SENSOR_NAME_SIZE, MAX_STEPS_PER_TASK};

    /**
     * @brief Enumeration representing the possible states of the MicroROSController.
     */
    enum class State
    {
        WAITING_AGENT,          ///< Waiting for the agent to be available
        AGENT_AVAILABLE,        ///< Agent is available
        AGENT_CONNECTED,        ///< Agent is connected
        AGENT_DISCONNECTED      ///< Agent is disconnected
    };

    State state_ = State::WAITING_AGENT;        ///< Current state of the controller

    TimedOperation* ping_agent_operation_;      ///< Timed operation for pinging the agent
};
