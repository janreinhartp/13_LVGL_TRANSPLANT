#include "system_controller.h"
#include "logger.h"

const char* SystemController::TAG = "SystemController";

SystemController::SystemController(ESPNowController& espnow, UIController& ui)
    : espnowCtrl(espnow), uiCtrl(ui), panelHandle(NULL), touchHandle(NULL) {
}

bool SystemController::begin() {
    Logger::begin(115200);
    Logger::info("SystemController", "Starting system initialization...");
    
    // Initialize display
    if (!initDisplay()) {
        Logger::error("SystemController", "Display initialization failed");
        return false;
    }
    
    // Initialize LVGL and UI
    if (!initLVGL()) {
        Logger::error("SystemController", "LVGL initialization failed");
        return false;
    }
    
    // Initialize ESP-NOW communication
    if (!initCommunication()) {
        Logger::error("SystemController", "Communication initialization failed");
        return false;
    }
    
    Logger::info("SystemController", "System initialization completed successfully");
    return true;
}

bool SystemController::initDisplay() {
    Logger::info("SystemController", "Initializing display hardware...");
    
    // Initialize the GT911 touch screen controller
    touchHandle = touch_gt911_init();
    if (touchHandle == NULL) {
        Logger::error("SystemController", "Touch controller initialization failed");
        return false;
    }
    Logger::debug("SystemController", "Touch controller initialized");
    
    // Initialize the Waveshare ESP32-S3 RGB LCD hardware
    panelHandle = waveshare_esp32_s3_rgb_lcd_init();
    if (panelHandle == NULL) {
        Logger::error("SystemController", "LCD panel initialization failed");
        return false;
    }
    Logger::debug("SystemController", "LCD panel initialized");
    
    // Turn on the LCD backlight
    wavesahre_rgb_lcd_bl_on();
    Logger::debug("SystemController", "LCD backlight enabled");
    
    Logger::info("SystemController", "Display hardware initialized");
    return true;
}

bool SystemController::initLVGL() {
    Logger::info("SystemController", "Initializing LVGL...");
    
    // Initialize LVGL with the panel and touch handles
    if (lvgl_port_init(panelHandle, touchHandle) != ESP_OK) {
        Logger::error("SystemController", "LVGL port initialization failed");
        return false;
    }
    Logger::debug("SystemController", "LVGL port initialized");
    
    // Lock the mutex because LVGL APIs are not thread-safe
    if (lvgl_port_lock(-1)) {
        ui_init(); // Custom UI initialization
        lvgl_port_unlock();
        Logger::info("SystemController", "UI initialized");
    } else {
        Logger::error("SystemController", "Failed to lock LVGL mutex");
        return false;
    }
    
    return true;
}

bool SystemController::initCommunication() {
    Logger::info("SystemController", "Initializing ESP-NOW communication...");
    
    // Initialize ESP-NOW controller
    if (!espnowCtrl.begin()) {
        Logger::error("SystemController", "ESP-NOW initialization failed");
        return false;
    }
    
    // Register broadcast peer
    if (!espnowCtrl.registerBroadcastPeer()) {
        Logger::error("SystemController", "Failed to register broadcast peer");
        return false;
    }
    
    Logger::info("SystemController", "Communication initialized");
    return true;
}
