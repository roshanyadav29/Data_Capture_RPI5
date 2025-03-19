#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>

// Log levels
typedef enum {
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR
} LogLevel;

// Logging functions
void log_set_level(LogLevel level);
void log_debug(const char* format, ...);
void log_info(const char* format, ...);
void log_warning(const char* format, ...);
void log_error(const char* format, ...);

// Time functions
unsigned long long get_time_ms(void);
void delay_ms(unsigned int ms);

// String utility functions
char* trim_string(char* str);

#endif // UTILS_H