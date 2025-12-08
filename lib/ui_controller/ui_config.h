#ifndef UI_CONFIG_H
#define UI_CONFIG_H

// UI Configuration for sensor displays and gauges

// Precision settings (decimal places)
struct UIPrecision {
    static constexpr int TEMPERATURE = 1;
    static constexpr int FLOW_RATE = 1;
    static constexpr int VOLTAGE = 2;
    static constexpr int CURRENT = 2;
    static constexpr int POWER = 1;
    static constexpr int LOAD_CELL = 1;
    static constexpr int GAS_FLOW = 1;
    static constexpr int RPM = 0;
};

// Value range for mapping to arc (0-100)
struct UIValueRange {
    float min;
    float max;
    
    constexpr UIValueRange(float minVal, float maxVal) : min(minVal), max(maxVal) {}
};

// Value ranges for each sensor (actual data ranges)
struct UIValueRanges {
    // Temperature ranges (°C)
    static constexpr UIValueRange WATER_TEMP_IN = {0, 100};
    static constexpr UIValueRange WATER_TEMP_OUT = {0, 100};
    static constexpr UIValueRange AIR_TEMP_INTAKE = {0, 100};
    static constexpr UIValueRange AIR_TEMP_EXHAUST = {0, 500};
    
    // Flow rate ranges (L/min or similar)
    static constexpr UIValueRange WATER_FLOW_RATE = {0, 300};
    static constexpr UIValueRange AIR_FLOW_INTAKE = {0, 1500};
    static constexpr UIValueRange AIR_FLOW_EXHAUST = {0, 1500};
    static constexpr UIValueRange GAS_FLOW_RATE = {0, 30};
    
    // RPM range (after scaling by 100)
    static constexpr UIValueRange ENGINE_RPM = {0, 3000};
    
    // Electrical ranges
    static constexpr UIValueRange VOLTAGE = {0, 300};      // 0-30V
    static constexpr UIValueRange CURRENT = {0, 30};     // 0-30A
    static constexpr UIValueRange POWER = {0, 5500};      // 0-5500W
    static constexpr UIValueRange LOAD_CELL = {0, 100};    // 0-50 kg
};

// Arc display range (fixed 0-100 for all gauges)
struct UIArcRange {
    static constexpr int MIN = 0;
    static constexpr int MAX = 100;
};

// Scale factors for arc values
struct UIScaleFactor {
    static constexpr int NONE = 1;
    static constexpr int DIVIDE_BY_100 = 100;
};

#endif // UI_CONFIG_H
