#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

// Log levels
#define LOG_LEVEL_DEBUG  0
#define LOG_LEVEL_INFO   1
#define LOG_LEVEL_WARN   2
#define LOG_LEVEL_ERROR  3

// Default log level (can be overridden in project settings)
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DEBUG
#endif

// Logging macros
#define LOG_DEBUG(fmt, ...)   if (LOG_LEVEL <= LOG_LEVEL_DEBUG)  log_print("DEBUG", fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)    if (LOG_LEVEL <= LOG_LEVEL_INFO)   log_print("INFO", fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)    if (LOG_LEVEL <= LOG_LEVEL_WARN)   log_print("WARN", fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)   if (LOG_LEVEL <= LOG_LEVEL_ERROR)  log_print("ERROR", fmt, ##__VA_ARGS__)

// Core logging function declaration
void log_print(const char *level, const char *format, ...);

#endif // LOGGER_H
