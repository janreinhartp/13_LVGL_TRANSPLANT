/*
* Ported LVGL 8.4 and display the official demo interface.
*/
#include <Arduino.h>

#include "lvgl_port.h"       // LVGL porting functions for integration
// #include <demos/lv_demos.h>        // LVGL demo headers
#include "ui.h"

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

typedef struct SensorData
{
    float waterTempIn;
    float waterTempOut;
    float waterFlowRate;
    float airIntakeTemp;
    float massFlowIntake;
    float exhaustTemp;
    float massFlowExhaust;
    int rpm;
    float gasFlowRate;
    float voltage;
    float ampere;
    float loadCell;
    char dateTime[25];
} SensorData;

SensorData sensorData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&sensorData, incomingData, sizeof(sensorData));
    // Also print to serial for debugging
    Serial.println("Dummy Sensor Data Received:");
    Serial.print("waterTempIn: ");
    Serial.println(sensorData.waterTempIn);
    Serial.print("waterTempOut: ");
    Serial.println(sensorData.waterTempOut);
    Serial.print("waterFlowRate: ");
    Serial.println(sensorData.waterFlowRate);
    Serial.print("airIntakeTemp: ");
    Serial.println(sensorData.airIntakeTemp);
    Serial.print("massFlowIntake: ");
    Serial.println(sensorData.massFlowIntake);
    Serial.print("exhaustTemp: ");
    Serial.println(sensorData.exhaustTemp);
    Serial.print("massFlowExhaust: ");
    Serial.println(sensorData.massFlowExhaust);
    Serial.print("rpm: ");
    Serial.println(sensorData.rpm);
    Serial.print("gasFlowRate: ");
    Serial.println(sensorData.gasFlowRate);
    Serial.print("voltage: ");
    Serial.println(sensorData.voltage);
    Serial.print("ampere: ");
    Serial.println(sensorData.ampere);
    Serial.print("loadCell: ");
    Serial.println(sensorData.loadCell);
    Serial.print("dateTime: ");
    Serial.println(sensorData.dateTime);
    Serial.println("---");

    lv_label_set_text(ui_txtWaterTempIn, String(sensorData.waterTempIn, 1).c_str());
    lv_label_set_text(ui_txtWaterTempOutlet, String(sensorData.waterTempOut, 1).c_str());
    lv_label_set_text(ui_txtWaterFlowRate, String(sensorData.waterFlowRate, 1).c_str());
    lv_label_set_text(ui_txtAirTempIntake, String(sensorData.airIntakeTemp, 1).c_str());
    lv_label_set_text(ui_txtAirTempExhaust, String(sensorData.exhaustTemp, 1).c_str());
    lv_label_set_text(ui_txtAirFlowIntake, String(sensorData.massFlowIntake, 1).c_str());
    lv_label_set_text(ui_txtAirFlowExhaust, String(sensorData.massFlowExhaust, 1).c_str());
    lv_label_set_text(ui_txtEngineRpm, String(sensorData.rpm).c_str());

    lv_arc_set_value(ui_arcWaterTempIn, (int)sensorData.waterTempIn);
    lv_arc_set_value(ui_arcWaterTempIn1, (int)sensorData.waterTempOut);
    lv_arc_set_value(ui_arcWaterFlowRate, (int)sensorData.waterFlowRate);
    lv_arc_set_value(ui_arcAirTempIntake, (int)sensorData.airIntakeTemp);
    lv_arc_set_value(ui_arcAirTempExhaust, (int)sensorData.exhaustTemp);
    lv_arc_set_value(ui_arcAirFlowIntake, (int)sensorData.massFlowIntake);
    lv_arc_set_value(ui_arcAirFlowExhaust, (int)sensorData.massFlowExhaust);
    lv_arc_set_value(ui_arcEngineRpm, sensorData.rpm / 100);  
}

static const char* TAG = "MyModule";

void setup() {
    static esp_lcd_panel_handle_t panel_handle = NULL; // Declare a handle for the LCD panel
    static esp_lcd_touch_handle_t tp_handle = NULL;    // Declare a handle for the touch panel

    // Initialize the GT911 touch screen controller
    tp_handle = touch_gt911_init();  
    
    // Initialize the Waveshare ESP32-S3 RGB LCD hardware
    panel_handle = waveshare_esp32_s3_rgb_lcd_init(); 

    // Turn on the LCD backlight
    wavesahre_rgb_lcd_bl_on();   

    // Initialize LVGL with the panel and touch handles
    ESP_ERROR_CHECK(lvgl_port_init(panel_handle, tp_handle));

    ESP_LOGI(TAG, "Display LVGL demos");
    // ESP_LOGI(TAG, "LVGL version: %s", lv_version_info().version);

    // Lock the mutex because LVGL APIs are not thread-safe
    if (lvgl_port_lock(-1)) {
        // Uncomment and run the desired demo functions here
        // lv_demo_stress();  // Stress test demo
        // lv_demo_benchmark(); // Benchmark demo
        // lv_demo_music();     // Music demo
        // lv_demo_widgets();    // Widgets demo
        ui_init();          // Custom UI initialization
        // Release the mutex after the demo execution
        lvgl_port_unlock();
    }

    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  // put your main code here, to run repeatedly:

}
