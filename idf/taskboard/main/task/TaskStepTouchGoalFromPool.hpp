/**
 * Robothon Task Board Firmware
 */

#pragma once

#include <sensor/Sensor.hpp>
#include <task/TaskStep.hpp>
#include <task/TaskStepTraceShapeManagePool.hpp>

#include <esp_log.h>

/**
 * @struct TaskStepTouchGoalFromPool
 *
 * @brief
 *
 * @details
 */
struct TaskStepTouchGoalFromPool :
    public TaskStep
{
    const char* TAG = "TaskStepTouchGoalFromPool";    ///< Logging tag
    static constexpr uint64_t TIMEOUT_LIMIT = 8000000L;    ///< Timeout limit in microseconds (8 seconds)

    /**
     * @brief Enumeration of touch goals
     */
    enum TouchGoalType
    {
        TapAGoal,
        TapBGoal,
        DoubleTapAGoal,
        DoubleTapBGoal,
        LongPressAGoal,
        LongPressBackgroundGoal,
        LongPressBGoal,
        TapBackgroundGoal,
        DoubleTapBackgroundGoal,
        DragFromAtoBGoal,
        DragFromBtoAGoal,
        DragFromAtoBackgroundGoal,
        DragFromBtoBackgroundGoal,
        DragFromBackgroundtoAGoal,
        DragFromBackgroundtoBGoal,
        SwipeUpGoal,
        SwipeDownGoal,
        SwipeLeftGoal,
        SwipeRightGoal
        };

    /**
     * @brief Constructs a new TaskStepTouchGoalFromPool object
     *
     * @param sensor Reference to the sensor to monitor
     * @param goal_pool Vector of touch goals to select from
     */
    TaskStepTouchGoalFromPool(
            SensorReader& sensor,
            std::vector<TouchGoalType>* goal_pool)
        : TaskStep(sensor),
            goal_pool_(goal_pool)
    {
        TaskStep::type_ = Type::TOUCH_GOAL_FROM_POOL;
    }

    /// Virtual method implementation
    void initialize_step() const override
    {
        // Select a random goal from the pool
        if (goal_pool_->empty())
        {
            ESP_LOGE(TAG, "Goal pool is empty");
            return;
        }
        
        goal_ = goal_pool_->at(0);
        goal_pool_->erase(goal_pool_->begin());

        initial_time_ = esp_timer_get_time();
        last_touching_ = false;
        tap_count_ = 0;

        measured_path_.clear();
    }

    /// Virtual method implementation
    bool success() const override
    {
        auto sensor_value_aux = sensor_.read();
        SensorMeasurement::Vector3 sensor_value = SensorMeasurement::Vector3{
            sensor_value_aux.get_vector3().x * 320.0f / 100.0f,
            sensor_value_aux.get_vector3().y * 240.0f / 100.0f,
            sensor_value_aux.get_vector3().z};
        bool pressing_screen = (sensor_value.z != 0);

        if (pressing_screen)
        {
            // Save the value
            measured_path_.push_back(sensor_value);

            // Update the timer for the first tap
            if (tap_count_ == 0)
            {
                first_tap_start_time_ = esp_timer_get_time();
                first_tap_duration_ = 0;
                ESP_LOGI(TAG, "Step started");
            }
            else if (tap_count_ == 1)
            {
                first_tap_duration_ = esp_timer_get_time() - first_tap_start_time_;
            }

            // Update the tap count
            if (!last_touching_)
            {
                tap_count_++;
                ESP_LOGI(TAG, "Tap count: %d", tap_count_);
            }
        }

        last_touching_ = pressing_screen;

        // Check for success conditions
        if (esp_timer_get_time() - initial_time_ > TIMEOUT_LIMIT)
        {
            // The time limit has been reached
            ESP_LOGI(TAG, "Time limit reached");
            return true;
        }
        else if (!pressing_screen)
        {
            // Has the tap count been reached?
            int tap_count_limit =  (goal_ == DoubleTapAGoal ||
                                    goal_ == DoubleTapBGoal ||
                                    goal_ == DoubleTapBackgroundGoal) ? 2 : 1;
                    
            if(tap_count_ >= tap_count_limit)
            {
                ESP_LOGI(TAG, "Step finished");
                return true;
            }
        }
        else if (first_tap_duration_ > 2000000)
        {
            // A long press has been detected
            ESP_LOGI(TAG, "Long press detected");
            tap_count_ = 1;
            return true;
        }

        return false;
    }

    bool is_inside_button_area(
            const SensorMeasurement& point,
            uint8_t button) const
    {
        static constexpr int32_t BUTTON_WIDTH = 80;
        static constexpr int32_t BUTTON_HEIGHT = 80;
        static constexpr int32_t BUTTON_Y = 220 - BUTTON_HEIGHT;
        static constexpr int32_t BUTTON_A_X = 40;
        static constexpr int32_t BUTTON_B_X = 320 - BUTTON_WIDTH -40;

        switch (button)
        {
            case 0:
                return (point.get_vector3().x >= BUTTON_A_X &&
                        point.get_vector3().x <= BUTTON_A_X + BUTTON_WIDTH &&
                        point.get_vector3().y >= BUTTON_Y &&
                        point.get_vector3().y <= BUTTON_Y + BUTTON_HEIGHT);
            case 1:
                return (point.get_vector3().x >= BUTTON_B_X &&
                        point.get_vector3().x <= BUTTON_B_X + BUTTON_WIDTH &&
                        point.get_vector3().y >= BUTTON_Y &&
                        point.get_vector3().y <= BUTTON_Y + BUTTON_HEIGHT);
            default:
                ESP_LOGE(TAG, "Invalid button index");
                return false;
        }
    }

    /// Virtual method implementation
    float score() const override
    {
        float score = 0.0f;
        if (measured_path_.empty())
        {
            return score;
        }

        switch (goal_)
        {
        case TapAGoal:
            // Check if all points are in the button A area
            for (const auto& point : measured_path_)
            {
                if (!is_inside_button_area(point, 0))
                {
                    return 0.0f;
                }
            }
            // Check if count_taps is 1
            if (tap_count_ != 1)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case TapBGoal:
            // Check if all points are in the button B area
            for (const auto& point : measured_path_)
            {
                if (!is_inside_button_area(point, 1))
                {
                    return 0.0f;
                }
            }
            // Check if count_taps is 1
            if (tap_count_ != 1)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case DoubleTapAGoal:
            // Check if all points are in the button A area and there are two taps
            for (const auto& point : measured_path_)
            {
                if (!is_inside_button_area(point, 0))
                {
                    return 0.0f;
                }
            }
            if (tap_count_ != 2)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case DoubleTapBGoal:
            // Check if all points are in the button A area and there are two taps
            for (const auto& point : measured_path_)
            {
                if (!is_inside_button_area(point, 1))
                {
                    return 0.0f;
                }
            }
            if (tap_count_ != 2)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case LongPressAGoal:
            // Check if all points are in the button A area and there is a long press
            for (const auto& point : measured_path_)
            {
                if (!is_inside_button_area(point, 0))
                {
                    return 0.0f;
                }
            }
            if (tap_count_ != 1 || first_tap_duration_ < 1000)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case LongPressBackgroundGoal:
            // Check if all points are in the background area and there is a long press (check the duration of the tap with the z coordinate)
            for (const auto& point : measured_path_)
            {
                if (is_inside_button_area(point, 0) ||
                    is_inside_button_area(point, 1)) 
                {
                    return 0.0f;
                }
            }
            if (tap_count_ != 1 || first_tap_duration_ < 1000)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case LongPressBGoal:
            // Check if all points are in the button A area and there is a long press
            for (const auto& point : measured_path_)
            {
                if (!is_inside_button_area(point, 1))
                {
                    return 0.0f;
                }
            }
            if (tap_count_ != 1 || first_tap_duration_ < 1000)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case TapBackgroundGoal:
            // Check if all points are in the background area
            for (const auto& point : measured_path_)
            {
                if (is_inside_button_area(point, 0) ||
                    is_inside_button_area(point, 1)) 
                {
                    return 0.0f;
                }
            }
            // Check if count_taps is 1
            if (tap_count_ != 1)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case DoubleTapBackgroundGoal:
            // Check if all points are in the background area
            for (const auto& point : measured_path_)
            {
                if (is_inside_button_area(point, 0) ||
                    is_inside_button_area(point, 1)) 
                {
                    return 0.0f;
                }
            }
            // Check if count_taps is 2
            if (tap_count_ != 2)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case DragFromAtoBGoal:
            // Check if the starting point is in the button A area and the ending point is in the button B area
            if (!is_inside_button_area(measured_path_[0], 0) ||
                !is_inside_button_area(measured_path_.back(), 1))
            {
                ESP_LOGI(TAG, "DragFromAtoBGoal: not in button area");
                return 0.0f;
            }
            // Check if the path is continuous
            if (tap_count_ > 1)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case DragFromBtoAGoal:
            // Check if the starting point is in the button B area and the ending point is in the button A area
            if (!is_inside_button_area(measured_path_[0], 1) ||
                !is_inside_button_area(measured_path_.back(), 0))
            {
                ESP_LOGI(TAG, "DragFromBtoAGoal: not in button area");
                return 0.0f;
            }
            // Check if the path is continuous
            if (tap_count_ > 1)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case DragFromAtoBackgroundGoal:
            // Check if the starting point is in the button A area and the ending point is in the background area
            if (!is_inside_button_area(measured_path_[0], 0) ||
                (is_inside_button_area(measured_path_.back(), 0) ||
                 is_inside_button_area(measured_path_.back(), 1)))
            {
                ESP_LOGI(TAG, "DragFromAtoBackgroundGoal: not in button area");
                return 0.0f;
            }
            // Check if the path is continuous
            if (tap_count_ > 1)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case DragFromBtoBackgroundGoal:
            // Check if the starting point is in the button B area and the ending point is in the background area
            if (!is_inside_button_area(measured_path_[0], 1) ||
                (is_inside_button_area(measured_path_.back(), 0) ||
                 is_inside_button_area(measured_path_.back(), 1)))
            {
                ESP_LOGI(TAG, "DragFromBtoBackgroundGoal: not in button area");
                return 0.0f;
            }
            // Check if the path is continuous
            if (tap_count_ > 1)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case DragFromBackgroundtoAGoal:
            // Check if the starting point is in the background area and the ending point is in the button A area
            if ((is_inside_button_area(measured_path_[0], 0) ||
                 is_inside_button_area(measured_path_[0], 1)) ||
                !is_inside_button_area(measured_path_.back(), 0))
            {
                ESP_LOGI(TAG, "DragFromBackgroundtoAGoal: not in button area");
                return 0.0f;
            }
            // Check if the path is continuous
            if (tap_count_ > 1)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case DragFromBackgroundtoBGoal:
            // Check if the starting point is in the background area and the ending point is in the button B area
            if ((is_inside_button_area(measured_path_[0], 0) ||
                 is_inside_button_area(measured_path_[0], 1)) ||
                !is_inside_button_area(measured_path_.back(), 1))
            {
                ESP_LOGI(TAG, "DragFromBackgroundtoBGoal: not in button area");
                return 0.0f;
            }
            // Check if the path is continuous
            if (tap_count_ > 1)
            {
                return 0.0f;
            }
            score = 100.0f;
            break;
        case SwipeUpGoal:
            // Check if the starting point is significantly lower than the ending point
            if (measured_path_[0].get_vector3().y > measured_path_.back().get_vector3().y - 20)
            {
                score = 100.0f;
            }
            break;
        case SwipeDownGoal:
            // Check if the starting point is significantly higher than the ending point
            if (measured_path_[0].get_vector3().y < measured_path_.back().get_vector3().y + 20)
            {
                score = 100.0f;
            }
            break;
        case SwipeLeftGoal:
            // Check if the starting point is significantly to the right of the ending point
            if (measured_path_[0].get_vector3().x > measured_path_.back().get_vector3().x + 20)
            {
                score = 100.0f;
            }
            break;
        case SwipeRightGoal:
            // Check if the starting point is significantly to the left of the ending point
            if (measured_path_[0].get_vector3().x < measured_path_.back().get_vector3().x - 20)
            {
                score = 100.0f;
            }
            break;
        default:
            ESP_LOGE(TAG, "Unknown goal type");
            score = 0.0f;
            break;
        }

        return score;
    }

    /// Virtual method implementation
    SensorMeasurement expected_value() const override
    {
        return SensorMeasurement();
    }

protected:

    /// Virtual method implementation
    void show_clue_implementation(
            ClueScreenController& screen_controller) const override
    {
        if (!success())
        {
            const char* clue = nullptr;
            switch(goal_)
            {
                case TouchGoalType::TapAGoal:
                    clue = "Tap A";
                    break;
                case TouchGoalType::TapBGoal:
                    clue = "Tap B";
                    break;
                case TouchGoalType::DoubleTapAGoal:
                    clue = "Double Tap A";
                    break;
                case TouchGoalType::DoubleTapBGoal:
                    clue = "Double Tap B";
                    break;
                case TouchGoalType::LongPressAGoal:
                    clue = "Long Press A";
                    break;
                case TouchGoalType::LongPressBackgroundGoal:
                    clue = "Long Press Background";
                    break;
                case TouchGoalType::LongPressBGoal:
                    clue = "Long Press B";
                    break;
                case TouchGoalType::TapBackgroundGoal:
                    clue = "Tap Background";
                    break;
                case TouchGoalType::DoubleTapBackgroundGoal:
                    clue = "Double Tap Background";
                    break;
                case TouchGoalType::DragFromAtoBGoal:
                    clue = "Drag from A to B";
                    break;
                case TouchGoalType::DragFromBtoAGoal:
                    clue = "Drag from B to A";
                    break;
                case TouchGoalType::DragFromAtoBackgroundGoal:
                    clue = "Drag from A to Background";
                    break;
                case TouchGoalType::DragFromBtoBackgroundGoal:
                    clue = "Drag from B to Background";
                    break;
                case TouchGoalType::DragFromBackgroundtoAGoal:
                    clue = "Drag from Background to A";
                    break;
                case TouchGoalType::DragFromBackgroundtoBGoal:
                    clue = "Drag from Background to B";
                    break;
                case TouchGoalType::SwipeUpGoal:
                    clue = "Swipe Up";
                    break;
                case TouchGoalType::SwipeDownGoal:
                    clue = "Swipe Down";
                    break;
                case TouchGoalType::SwipeLeftGoal:
                    clue = "Swipe Left";
                    break;
                case TouchGoalType::SwipeRightGoal:
                    clue = "Swipe Right";
                    break;
            }
            screen_controller.print_task_clue(sensor_.name());
            screen_controller.print_task_clue_goal(clue);
        }
        else
        {
            reset_clue();
        }
    }

    mutable TouchGoalType goal_;                                     ///< Shape type to be traced
    mutable std::vector<SensorMeasurement>measured_path_ = {};  ///< Path measured
    mutable int64_t initial_time_ = 0;                               ///< Time when the step started
    mutable int tap_count_ = 0;                                  ///< Number of taps detected
    mutable int32_t first_tap_duration_ = 0;                      ///< Longest tap duration detected
    mutable int32_t first_tap_start_time_ = 0;                    ///< Start time of the longest tap
    mutable bool last_touching_ = false;                            ///< Flag to indicate if the last touch was a tap

    std::vector<TouchGoalType>* goal_pool_;    ///< Vector of shapes to select from
};

/**
 * @struct TaskStepTouchGoalSetPool
 * 
 * @brief Implementation of TaskStep that empties a shape pool and fills it with a vector of shapes
 */
struct TaskStepTouchGoalSetPool :
    public TaskStepAuto
{
    const char* TAG = "TaskStepTouchGoalSetPool";    ///< Logging tag

    /**
     * @brief Constructs a new TaskStepTouchGoalSetPool object
     *
     * @param name Name identifier for the task step
     * @param goal_pool Vector of shapes to be filled
     * @param goal_vector Vector of shapes to fill the pool with
     */
    TaskStepTouchGoalSetPool(
            std::string name,
            std::vector<TaskStepTouchGoalFromPool::TouchGoalType>* goal_pool,
            std::vector<TaskStepTouchGoalFromPool::TouchGoalType>* goal_vector)
        : TaskStepAuto(name)
        , goal_pool_(goal_pool)
        , goal_vector_(goal_vector)
    {
        TaskStepAuto::type_ = Type::TRACE_SHAPE_MANAGE_POOL;
    }

    /// Virtual method implementation
    void initialize_step() const override
    {
        goal_pool_->clear();
        goal_pool_->shrink_to_fit();

        goal_pool_->insert(goal_pool_->end(), goal_vector_->begin(), goal_vector_->end());
        esp_shuffle(goal_pool_);
    }

    /// Virtual method implementation
    bool success() const override
    {
        return true;
    }

    /// Virtual method implementation
    float score() const override
    {
        return -1.0f;
    }

private:

    /// Virtual method implementation
    void show_clue_implementation(
            ClueScreenController& screen_controller) const override
    {
        screen_controller.print_task_clue("Goal pool filled");
    }

    std::vector<TaskStepTouchGoalFromPool::TouchGoalType>* goal_pool_;    ///< Vector of shapes to be filled
    const std::vector<TaskStepTouchGoalFromPool::TouchGoalType>* goal_vector_;    ///< Vector of shapes to fill the pool with
};