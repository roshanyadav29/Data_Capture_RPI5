#include "../include/utils.h"
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>
#include <unistd.h>

// Default log level
static LogLevel current_log_level = LOG_LEVEL_INFO;

// Set the log level
void log_set_level(LogLevel level) {
    current_log_level = level;
}

// Log with timestamp and level
static void log_message(LogLevel level, const char* level_str, const char* format, va_list args) {
    // Only log if current level is less than or equal to the message's level
    if (level < current_log_level) {
        return;
    }
    
    // Get current time
    time_t now;
    struct tm *tm_info;
    char timestamp[20];
    
    time(&now);
    tm_info = localtime(&now);
    strftime(timestamp, 20, "%Y-%m-%d %H:%M:%S", tm_info);
    
    // Print log header
    fprintf(stderr, "[%s] %s: ", timestamp, level_str);
    
    // Print the actual message
    vfprintf(stderr, format, args);
    
    // Newline
    fprintf(stderr, "\n");
}

// Debug level log
void log_debug(const char* format, ...) {
    va_list args;
    va_start(args, format);
    log_message(LOG_LEVEL_DEBUG, "DEBUG", format, args);
    va_end(args);
}

// Info level log
void log_info(const char* format, ...) {
    va_list args;
    va_start(args, format);
    log_message(LOG_LEVEL_INFO, "INFO", format, args);
    va_end(args);
}

// Warning level log
void log_warning(const char* format, ...) {
    va_list args;
    va_start(args, format);
    log_message(LOG_LEVEL_WARNING, "WARNING", format, args);
    va_end(args);
}

// Error level log
void log_error(const char* format, ...) {
    va_list args;
    va_start(args, format);
    log_message(LOG_LEVEL_ERROR, "ERROR", format, args);
    va_end(args);
}

// Get current time in milliseconds
unsigned long long get_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
}

// Delay for specified milliseconds
void delay_ms(unsigned int ms) {
    usleep(ms * 1000);
}

// Trim whitespace from a string
char* trim_string(char* str) {
    if (!str) return NULL;
    
    // Trim leading space
    while(isspace((unsigned char)*str)) str++;
    
    // Trim trailing space
    if (*str == 0) return str;
    
    char* end = str + strlen(str) - 1;
    while(end > str && isspace((unsigned char)*end)) end--;
    
    end[1] = '\0';
    
    return str;
}