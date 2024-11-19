/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <task/Task.hpp>

#include <esp_spiffs.h>
#include <esp_log.h>

#include <cstring>

/**
 * @struct NonVolatileStorage
 *
 * @brief Manages persistent storage of task execution data using SPIFFS
 */
struct NonVolatileStorage
{
    const char* TAG = "NonVolatileStorage";   ///< Logging tag

    /**
     * @brief Constructs a new NonVolatileStorage object
     * @details Initializes SPIFFS filesystem and creates/validates the CSV log file
     *
     * @param device_id Unique identifier for the device
     */
    NonVolatileStorage(
            const std::string& device_id)
        : device_id_(device_id)
    {
        ESP_LOGI(TAG, "Initializing SPIFFS");

        esp_vfs_spiffs_conf_t conf = {
            .base_path = "/spiffs",
            .partition_label = NULL,
            .max_files = 5,
            .format_if_mount_failed = true
        };

        esp_err_t ret = esp_vfs_spiffs_register(&conf);

        if (ret != ESP_OK)
        {
            if (ret == ESP_FAIL)
            {
                ESP_LOGE(TAG, "Failed to mount or format filesystem");
            }
            else if (ret == ESP_ERR_NOT_FOUND)
            {
                ESP_LOGE(TAG, "Failed to find SPIFFS partition");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
            }

            return;
        }

        size_t total = 0, used = 0;
        ret = esp_spiffs_info(NULL, &total, &used);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        }
        else
        {
            ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
        }

        initialize_csv_file();
    }

    /**
     * @brief Destructor that cleanly unmounts SPIFFS
     */
    ~NonVolatileStorage()
    {
        esp_vfs_spiffs_unregister(NULL);
        ESP_LOGI(TAG, "SPIFFS unmounted");
    }

    /**
     * @brief Adds a new task execution record to the log file
     * @details Creates a CSV entry with task name, execution time, and execution mode
     *
     * @param task Reference to completed task
     * @param is_human true if executed by human, false if by robot
     */
    void add_new_register(
            const Task& task,
            bool is_human)
    {
        FILE* file = fopen(NonVolatileStorage::log_file().c_str(), "a");

        if (file == NULL)
        {
            ESP_LOGE(TAG, "File not properly initialized");

            return;
        }

        // Write CSV line: timestamp, task name, is_human
        fprintf(file, "%s,%lld,%d,%s\n",
                task.name().c_str(),
                task.elapsed_time(),
                is_human ? 1 : 0,
                task.unique_id().c_str());

        // Ensure data is written to file
        fflush(file);

        ESP_LOGI(TAG, "Added new register: %s, %lld, %d",
                task.name().c_str(), task.elapsed_time(), is_human ? 1 : 0);

        fclose(file);
    }

    /**
     * @brief Gets the path to the log file
     * @return Reference to log file path string
     */
    static const std::string& log_file()
    {
        static const std::string log_file = "/spiffs/taskboard.csv";

        return log_file;
    }

    /**
     * @brief Clears all records from the log file
     * @details Reinitializes the file with headers while maintaining device ID
     */
    void clear_log()
    {
        FILE* file = fopen(NonVolatileStorage::log_file().c_str(), "w");

        if (file == NULL)
        {
            ESP_LOGE(TAG, "Failed to open file for clearing");

            return;
        }

        // Write headers
        fprintf(file, "# DEVICE_ID=%s\n", device_id_.c_str());
        fprintf(file, "task_name,timestamp,is_human\n");

        fflush(file);
        fclose(file);
    }

private:

    /**
     * @brief Initializes or validates the CSV log file
     *
     * @details Creates a new file with headers if it doesn't exist or has incorrect format.
     *          Otherwise opens existing file in append mode.
     */
    void initialize_csv_file()
    {
        // First try to open file for reading to check if it exists and has header
        FILE* file = fopen(NonVolatileStorage::log_file().c_str(), "r");
        bool create_new_file = false;

        if (file == NULL)
        {
            // File doesn't exist, need to create new
            create_new_file = true;
            ESP_LOGI(TAG, "File doesn't exist");
        }
        else
        {
            // Check if file has correct header
            char first_line[64];

            if (fgets(first_line, sizeof(first_line), file) == NULL)
            {
                create_new_file = true; // File is empty
                ESP_LOGI(TAG, "File is empty");
            }
            else
            {
                if (strncmp(first_line, "# DEVICE_ID=", 12) != 0)
                {
                    create_new_file = true; // Incorrect header
                    ESP_LOGI(TAG, "Incorrect header");
                }
            }

            fclose(file);
        }

        // Open file in appropriate mode
        if (create_new_file)
        {
            ESP_LOGI(TAG, "Creating new CSV file");
            file = fopen(NonVolatileStorage::log_file().c_str(), "w"); // New file with write mode

            if (file == NULL)
            {
                ESP_LOGE(TAG, "Failed to create new file");

                return;
            }

            // Write headers
            fprintf(file, "# DEVICE_ID=%s\n", device_id_.c_str());
            fprintf(file, "task_name,timestamp,is_human\n");
        }
        else
        {
            ESP_LOGI(TAG, "Opening existing CSV file for append");
            file = fopen(NonVolatileStorage::log_file().c_str(), "a"); // Append mode for existing file

            if (file == NULL)
            {
                ESP_LOGE(TAG, "Failed to open existing file for append");

                return;
            }
        }

        fflush(file);
        fclose(file);
    }

    const std::string& device_id_;    ///< Reference to device's unique identifier
};
