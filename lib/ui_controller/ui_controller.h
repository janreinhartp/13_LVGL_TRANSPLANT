#ifndef UI_CONTROLLER_H
#define UI_CONTROLLER_H

#include "lvgl.h"
#include "ui.h"
#include "ui_config.h"
#include "espnow_controller.h"

class UIController {
public:
    // Constructor
    UIController();
    
    // Update all UI elements with sensor data
    void updateUI(const SensorData& data);
    
    // Update individual sections
    void updateEngineGauges(const SensorData& data);
    void updateGeneratorGauges(const SensorData& data);
    void updateGeneralInfo(const SensorData& data);
    
private:
    // Helper function for updating label and arc widgets
    void updateGauge(lv_obj_t* label, lv_obj_t* arc, float value, 
                     int precision, int scaleFactor = UIScaleFactor::NONE);
    
    // Overloaded helper with value range for proper arc mapping
    void updateGauge(lv_obj_t* label, lv_obj_t* arc, float value, 
                     int precision, const UIValueRange& valueRange, int scaleFactor = UIScaleFactor::NONE);
};

#endif // UI_CONTROLLER_H
