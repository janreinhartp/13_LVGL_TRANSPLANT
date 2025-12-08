#include <Arduino.h>

// Controller includes
#include "espnow_controller.h"
#include "ui_controller.h"
#include "system_controller.h"
#include "logger.h"

// Global controller instances
ESPNowController espnowController;
UIController uiController;
SystemController systemController(espnowController, uiController);

// Callback function for handling received sensor data
void onSensorDataReceived(const SensorData& data) {
  // Update UI with received sensor data
  uiController.updateUI(data);
}

void setup()
{
  // Initialize all system components
  if (!systemController.begin())
  {
    Logger::error("Main", "System initialization failed, rebooting...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP.restart();
  }

  // Set callback for received data
  espnowController.setDataReceivedCallback(onSensorDataReceived);
  Logger::info("Main", "Application started successfully");
}

void loop()
{
  // put your main code here, to run repeatedly:
  // delay(1000);
  // readMacAddress();
}