#ifndef SYSTEM_CONTROLLER_H
#define SYSTEM_CONTROLLER_H

#include <Arduino.h>
#include "lvgl_port.h"
#include "ui.h"
#include "espnow_controller.h"
#include "ui_controller.h"

class SystemController {
public:
    // Constructor
    SystemController(ESPNowController& espnow, UIController& ui);
    
    // Initialize all system components
    bool begin();
    
    // Initialize display and touch
    bool initDisplay();
    
    // Initialize LVGL and UI
    bool initLVGL();
    
    // Initialize ESP-NOW communication
    bool initCommunication();
    
    // Get panel handle
    esp_lcd_panel_handle_t getPanelHandle() const { return panelHandle; }
    
    // Get touch handle
    esp_lcd_touch_handle_t getTouchHandle() const { return touchHandle; }

private:
    ESPNowController& espnowCtrl;
    UIController& uiCtrl;
    
    esp_lcd_panel_handle_t panelHandle;
    esp_lcd_touch_handle_t touchHandle;
    
    static const char* TAG;
};

#endif // SYSTEM_CONTROLLER_H
