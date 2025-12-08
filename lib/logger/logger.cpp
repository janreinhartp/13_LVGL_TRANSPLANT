#include "logger.h"

LogLevel Logger::currentLogLevel = LogLevel::INFO;
bool Logger::initialized = false;

void Logger::begin(unsigned long baudRate) {
    if (!initialized) {
        Serial.begin(baudRate);
        initialized = true;
        info("Logger", "Logger initialized");
    }
}

void Logger::setLogLevel(LogLevel level) {
    currentLogLevel = level;
    infof("Logger", "Log level set to %s", getLevelString(level));
}

void Logger::debug(const String& tag, const String& message) {
    log(LogLevel::DEBUG, tag, message);
}

void Logger::info(const String& tag, const String& message) {
    log(LogLevel::INFO, tag, message);
}

void Logger::warning(const String& tag, const String& message) {
    log(LogLevel::WARNING, tag, message);
}

void Logger::error(const String& tag, const String& message) {
    log(LogLevel::ERROR, tag, message);
}

void Logger::debugf(const String& tag, const char* format, ...) {
    if (static_cast<int>(currentLogLevel) > static_cast<int>(LogLevel::DEBUG)) return;
    
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    log(LogLevel::DEBUG, tag, String(buffer));
}

void Logger::infof(const String& tag, const char* format, ...) {
    if (static_cast<int>(currentLogLevel) > static_cast<int>(LogLevel::INFO)) return;
    
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    log(LogLevel::INFO, tag, String(buffer));
}

void Logger::warningf(const String& tag, const char* format, ...) {
    if (static_cast<int>(currentLogLevel) > static_cast<int>(LogLevel::WARNING)) return;
    
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    log(LogLevel::WARNING, tag, String(buffer));
}

void Logger::errorf(const String& tag, const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    log(LogLevel::ERROR, tag, String(buffer));
}

void Logger::log(LogLevel level, const String& tag, const String& message) {
    if (static_cast<int>(level) < static_cast<int>(currentLogLevel)) {
        return;
    }
    
    if (!initialized) {
        begin();
    }
    
    // Format: [timestamp] [LEVEL] [TAG] message
    Serial.printf("[%lu] [%s] [%s] %s\n", 
                  getTimestamp(), 
                  getLevelString(level), 
                  tag.c_str(), 
                  message.c_str());
}

const char* Logger::getLevelString(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG:   return "DEBUG";
        case LogLevel::INFO:    return "INFO ";
        case LogLevel::WARNING: return "WARN ";
        case LogLevel::ERROR:   return "ERROR";
        default:                return "UNKNOWN";
    }
}

unsigned long Logger::getTimestamp() {
    return millis();
}
