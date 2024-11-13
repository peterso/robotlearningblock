/**
 * Roboton Task Board Firmware
 */

#pragma once

#include <builtin_interfaces/msg/time.h>

#include <esp_log.h>
#include <esp_timer.h>

/**
* @brief Converts microseconds to ROS Time message
*
* @details Converts a microsecond timestamp into a ROS2 builtin_interfaces Time message,
*          splitting it into seconds and nanoseconds components
*
* @param usec Time in microseconds
*
* @return ROS Time message equivalent
*/
inline builtin_interfaces__msg__Time usec_to_microros(const int64_t usec)
{
   builtin_interfaces__msg__Time time;
   time.sec = usec / 1000000;
   time.nanosec = (usec % 1000000) * 1000;
   return time;
}

/**
* @struct TimedOperation
*
* @brief Executes a callback function periodically
*/
struct TimedOperation
{
   /// @brief Function type for periodic callback
   using Callback = std::function<void()>;

   /**
    * @brief Constructs a new TimedOperation object
    * @param period_ms Period in milliseconds between callback executions
    * @param callback Function to execute periodically
    */
   TimedOperation(const uint32_t period_ms, const Callback& callback)
   : callback_(callback),
     period_ms_(period_ms),
     last_time_(esp_timer_get_time())
   {}

   /**
    * @brief Updates the timer and executes callback if period has elapsed
    */
   void update()
   {
       const uint32_t current_time = esp_timer_get_time();
       if(current_time - last_time_ > period_ms_ * 1000)
       {
           callback_();
           last_time_ = current_time;
       }
   }

private:
   Callback callback_;              ///< Function to execute periodically
   const uint32_t period_ms_;       ///< Period between executions in milliseconds
   uint32_t last_time_;             ///< Last execution time in microseconds
};