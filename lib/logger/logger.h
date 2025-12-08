#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

// Log levels
enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR
};

class Logger {
public:
    // Initialize logger
    static void begin(unsigned long baudRate = 115200);
    
    // Set minimum log level
    static void setLogLevel(LogLevel level);
    
    // Log methods
    static void debug(const String& tag, const String& message);
    static void info(const String& tag, const String& message);
    static void warning(const String& tag, const String& message);
    static void error(const String& tag, const String& message);
    
    // Log with formatted values
    static void debugf(const String& tag, const char* format, ...);
    static void infof(const String& tag, const char* format, ...);
    static void warningf(const String& tag, const char* format, ...);
    static void errorf(const String& tag, const char* format, ...);

private:
    static LogLevel currentLogLevel;
    static bool initialized;
    
    static void log(LogLevel level, const String& tag, const String& message);
    static const char* getLevelString(LogLevel level);
    static unsigned long getTimestamp();
};

#endif // LOGGER_H
