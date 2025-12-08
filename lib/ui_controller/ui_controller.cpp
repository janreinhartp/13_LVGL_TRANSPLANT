#include "ui_controller.h"
#include "logger.h"

UIController::UIController() {
    // Constructor - no initialization needed for now
}

void UIController::updateUI(const SensorData& data) {
    Logger::debug("UIController", "Updating UI with new sensor data");
    
    // Update all sections
    updateEngineGauges(data);
    updateGeneratorGauges(data);
    updateGeneralInfo(data);
}

void UIController::updateEngineGauges(const SensorData& data) {
    Logger::debug("UIController", "Updating engine gauges");
    
    // Water Inlet Temperature
    updateGauge(ui_txtWaterTempIn, ui_arcWaterTempIn, 
                data.waterTempIn, UIPrecision::TEMPERATURE, UIValueRanges::WATER_TEMP_IN);
    
    // Water Outlet Temperature
    updateGauge(ui_txtWaterTempOutlet, ui_arcWaterTempIn1, 
                data.waterTempOut, UIPrecision::TEMPERATURE, UIValueRanges::WATER_TEMP_OUT);
    
    // Water Flow Rate
    updateGauge(ui_txtWaterFlowRate, ui_arcWaterFlowRate, 
                data.waterFlowRate, UIPrecision::FLOW_RATE, UIValueRanges::WATER_FLOW_RATE);
    
    // Air Intake Temperature
    updateGauge(ui_txtAirTempIntake, ui_arcAirTempIntake, 
                data.airIntakeTemp, UIPrecision::TEMPERATURE, UIValueRanges::AIR_TEMP_INTAKE);
    
    // Exhaust Temperature
    updateGauge(ui_txtAirTempExhaust, ui_arcAirTempExhaust, 
                data.exhaustTemp, UIPrecision::TEMPERATURE, UIValueRanges::AIR_TEMP_EXHAUST);
    
    // Air Flow Rate Intake
    updateGauge(ui_txtAirFlowIntake, ui_arcAirFlowIntake, 
                data.massFlowIntake, UIPrecision::FLOW_RATE, UIValueRanges::AIR_FLOW_INTAKE);
    
    // Air Flow Rate Exhaust
    updateGauge(ui_txtAirFlowExhaust, ui_arcAirFlowExhaust, 
                data.massFlowExhaust, UIPrecision::FLOW_RATE, UIValueRanges::AIR_FLOW_EXHAUST);
    
    // Engine RPM
    updateGauge(ui_txtEngineRpm, ui_arcEngineRpm, 
                data.rpm, UIPrecision::RPM, UIValueRanges::ENGINE_RPM);
}

void UIController::updateGeneratorGauges(const SensorData& data) {
    Logger::debug("UIController", "Updating generator gauges");
    
    // Voltage
    updateGauge(ui_txtVoltage, ui_arcVoltage, 
                data.voltage, UIPrecision::VOLTAGE, UIValueRanges::VOLTAGE);
    
    // Ampere (Current)
    updateGauge(ui_txtAmpere, ui_arcAmpere, 
                data.ampere, UIPrecision::CURRENT, UIValueRanges::CURRENT);
    
    // Power (calculated)
    float power = data.ampere * data.voltage;
    updateGauge(ui_txtPower, ui_arcPower, 
                power, UIPrecision::POWER, UIValueRanges::POWER);
    
    // Generator RPM
    updateGauge(ui_txtEngineRpm1, ui_arcEngineRpm1, 
                data.rpm, UIPrecision::RPM, UIValueRanges::ENGINE_RPM);
    
    // Load Cell
    updateGauge(ui_txtLoadCell, ui_arcLoadCell, 
                data.loadCell, UIPrecision::LOAD_CELL, UIValueRanges::LOAD_CELL);
    
    // Gas Flow Rate
    updateGauge(ui_txtGasFlowRate, ui_arcGasFlowRate, 
                data.gasFlowRate, UIPrecision::GAS_FLOW, UIValueRanges::GAS_FLOW_RATE);
}

void UIController::updateGeneralInfo(const SensorData& data) {
    // Reserved for future general information displays
}

void UIController::updateGauge(lv_obj_t* label, lv_obj_t* arc, float value, 
                               int precision, int scaleFactor) {
    // Update label with formatted value
    lv_label_set_text(label, String(value, precision).c_str());
    
    // Scale and map value to 0-100 range for arc
    float scaledValue = value / scaleFactor;
    int mappedValue = map((int)scaledValue, 0, 100, 0, 100);
    
    // Clamp value to 0-100 range
    if (mappedValue < 0) mappedValue = 0;
    if (mappedValue > 100) mappedValue = 100;
    
    // Update arc with mapped value
    lv_arc_set_value(arc, mappedValue);
}

void UIController::updateGauge(lv_obj_t* label, lv_obj_t* arc, float value, 
                               int precision, const UIValueRange& valueRange, int scaleFactor) {
    // Update label with formatted value
    lv_label_set_text(label, String(value, precision).c_str());
    
    // Apply scale factor first
    float scaledValue = value / scaleFactor;
    
    // Map the scaled value from its actual range to 0-100 for the arc
    int mappedValue = map((int)scaledValue, 
                          (int)valueRange.min, (int)valueRange.max, 
                          UIArcRange::MIN, UIArcRange::MAX);
    
    // Clamp value to arc range
    if (mappedValue < UIArcRange::MIN) mappedValue = UIArcRange::MIN;
    if (mappedValue > UIArcRange::MAX) mappedValue = UIArcRange::MAX;
    
    // Update arc with mapped value
    lv_arc_set_value(arc, mappedValue);
}
