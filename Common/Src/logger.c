#include <logger.h>
#include <stdio.h>
#include <stdarg.h>

// Core logging function
void log_print(const char *level, const char *format, ...)
{
    char buffer[256];
    va_list args;

    // Start the variadic argument list
    va_start(args, format);

    // Format the log message with the specified level
    int len = snprintf(buffer, sizeof(buffer), "[%s]: ", level);
    vsnprintf(buffer + len, sizeof(buffer) - len, format, args);

    // End the variadic argument list
    va_end(args);

    // Output the log message
    printf("%s\n", buffer);  // Example: SWO or UART
}
