/**
 * Robothon Task Board Firmware
 */

#include <network/JSONHandler.hpp>

uint8_t JSONHandler::memory_buffer_[JSON_BUFFER_SIZE];
uint32_t JSONHandler::memory_buffer_index_;
SemaphoreHandle_t JSONHandler::mutex_;
bool JSONHandler::mutex_initialized_;
