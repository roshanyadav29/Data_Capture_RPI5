#include "../include/utils.h"
#include <stdio.h>
#include <time.h>
#include <unistd.h>

void log_error(const char *message) {
    time_t now;
    struct tm *tm_info;
    char timestamp[64];
    
    time(&now);
    tm_info = localtime(&now);
    strftime(timestamp, 64, "%Y-%m-%d %H:%M:%S", tm_info);
    
    fprintf(stderr, "[%s] ERROR: %s\n", timestamp, message);
}

void log_info(const char *message) {
    time_t now;
    struct tm *tm_info;
    char timestamp[64];
    
    time(&now);
    tm_info = localtime(&now);
    strftime(timestamp, 64, "%Y-%m-%d %H:%M:%S", tm_info);
    
    printf("[%s] INFO: %s\n", timestamp, message);
}

void delay_ms(unsigned int milliseconds) {
    usleep(milliseconds * 1000);
}