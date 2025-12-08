#ifndef ESPNOW_CONTROLLER_H
#define ESPNOW_CONTROLLER_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Structure for sensor data received via ESP-NOW
struct SensorData {
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
};

// Callback function type for data reception
typedef void (*DataReceivedCallback)(const SensorData& data);

class ESPNowController {
public:
    // Constructor
    ESPNowController();
    
    // Initialize ESP-NOW
    bool begin();
    
    // Register broadcast peer
    bool registerBroadcastPeer();
    
    // Set callback for received data
    void setDataReceivedCallback(DataReceivedCallback callback);
    
    // Get the latest sensor data
    const SensorData& getSensorData() const;
    
    // Read and return MAC address
    String getMacAddress();
    
    // Send data via ESP-NOW
    bool sendData(const uint8_t* macAddr, const void* data, size_t len);

private:
    SensorData sensorData;
    DataReceivedCallback dataCallback;
    uint8_t broadcastAddr[6];
    esp_now_peer_info_t broadcastPeer;
    
    // Static callback wrapper for ESP-NOW
    static void onDataRecvStatic(const esp_now_recv_info_t *mac, const uint8_t *data, int len);
    
    // Instance pointer for static callback
    static ESPNowController* instance;
};

#endif // ESPNOW_CONTROLLER_H
