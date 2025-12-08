#include "espnow_controller.h"
#include "logger.h"

// Initialize static instance pointer
ESPNowController* ESPNowController::instance = nullptr;

ESPNowController::ESPNowController() 
    : dataCallback(nullptr) {
    // Set broadcast address (FF:FF:FF:FF:FF:FF)
    memset(broadcastAddr, 0xFF, 6);
    memset(&sensorData, 0, sizeof(sensorData));
    memset(&broadcastPeer, 0, sizeof(broadcastPeer));
    
    // Set static instance for callback
    instance = this;
}

bool ESPNowController::begin() {
    Logger::info("ESPNowController", "Initializing ESP-NOW...");
    
    // Initialize WiFi in station mode
    WiFi.mode(WIFI_STA);
    Logger::debug("ESPNowController", "WiFi set to Station mode");
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Logger::error("ESPNowController", "ESP-NOW initialization failed");
        return false;
    }
    
    // Register receive callback
    esp_now_register_recv_cb(onDataRecvStatic);
    Logger::debug("ESPNowController", "Receive callback registered");
    
    Logger::info("ESPNowController", "ESP-NOW initialized successfully");
    return true;
}

bool ESPNowController::registerBroadcastPeer() {
    Logger::debug("ESPNowController", "Configuring broadcast peer...");
    
    // Configure broadcast peer
    broadcastPeer.channel = 0;                         // 0 = any channel
    broadcastPeer.encrypt = false;                     // No encryption for broadcast
    memcpy(broadcastPeer.peer_addr, broadcastAddr, 6); // Copy broadcast address
    
    // Register peer
    if (esp_now_add_peer(&broadcastPeer) != ESP_OK) {
        Logger::error("ESPNowController", "Failed to register broadcast peer");
        return false;
    }
    
    Logger::info("ESPNowController", "Broadcast peer registered successfully");
    return true;
}

void ESPNowController::setDataReceivedCallback(DataReceivedCallback callback) {
    dataCallback = callback;
}

const SensorData& ESPNowController::getSensorData() const {
    return sensorData;
}

String ESPNowController::getMacAddress() {
    uint8_t baseMac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
    
    if (ret == ESP_OK) {
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
                 baseMac[0], baseMac[1], baseMac[2],
                 baseMac[3], baseMac[4], baseMac[5]);
        Logger::infof("ESPNowController", "MAC Address: %s", macStr);
        return String(macStr);
    } else {
        Logger::error("ESPNowController", "Failed to read MAC address");
        return String("");
    }
}

bool ESPNowController::sendData(const uint8_t* macAddr, const void* data, size_t len) {
    esp_err_t result = esp_now_send(macAddr, (uint8_t*)data, len);
    return (result == ESP_OK);
}

void ESPNowController::onDataRecvStatic(const esp_now_recv_info_t *mac, const uint8_t *data, int len) {
    if (instance != nullptr) {
        // Copy received data to sensor data structure
        if (len == sizeof(SensorData)) {
            memcpy(&instance->sensorData, data, sizeof(SensorData));
            
            // Log received data in tabulated form
            Logger::info("ESPNowController", "========== Sensor Data Received ==========");
            Logger::infof("ESPNowController", "%-25s | %8.2f °C", "Water Temp In", instance->sensorData.waterTempIn);
            Logger::infof("ESPNowController", "%-25s | %8.2f °C", "Water Temp Out", instance->sensorData.waterTempOut);
            Logger::infof("ESPNowController", "%-25s | %8.2f L/min", "Water Flow Rate", instance->sensorData.waterFlowRate);
            Logger::infof("ESPNowController", "%-25s | %8.2f °C", "Air Intake Temp", instance->sensorData.airIntakeTemp);
            Logger::infof("ESPNowController", "%-25s | %8.2f kg/h", "Mass Flow Intake", instance->sensorData.massFlowIntake);
            Logger::infof("ESPNowController", "%-25s | %8.2f °C", "Exhaust Temp", instance->sensorData.exhaustTemp);
            Logger::infof("ESPNowController", "%-25s | %8.2f kg/h", "Mass Flow Exhaust", instance->sensorData.massFlowExhaust);
            Logger::infof("ESPNowController", "%-25s | %8d RPM", "Engine RPM", instance->sensorData.rpm);
            Logger::infof("ESPNowController", "%-25s | %8.2f L/min", "Gas Flow Rate", instance->sensorData.gasFlowRate);
            Logger::infof("ESPNowController", "%-25s | %8.2f V", "Voltage", instance->sensorData.voltage);
            Logger::infof("ESPNowController", "%-25s | %8.2f A", "Ampere", instance->sensorData.ampere);
            Logger::infof("ESPNowController", "%-25s | %8.2f kg", "Load Cell", instance->sensorData.loadCell);
            Logger::infof("ESPNowController", "%-25s | %s", "Date/Time", instance->sensorData.dateTime);
            Logger::info("ESPNowController", "==========================================");
            
            // Call user callback if registered
            if (instance->dataCallback != nullptr) {
                instance->dataCallback(instance->sensorData);
            }
        } else {
            Logger::errorf("ESPNowController", "Data size mismatch: %d bytes (expected %d)", 
                          len, sizeof(SensorData));
        }
    }
}
