/*
This code receives data from the remote

Data is sent using the ESP-NOW Protocol
*/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "wireless.h"

joy_message joyData;

esp_now_peer_info_t peerInfo;

//TODO replace this address with the address of the remote 
uint8_t broadcastAddress[] = {0x44, 0x17, 0x93, 0x5D, 0xA7, 0x10};

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&joyData, incomingData, sizeof(joyData));
}

void wirelessSetup(void){

// Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //Tell the microcontroller which functions to call when
  //data is sent or received
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  

  // Register peer to send to
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

    // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // ESP-NOW Setup Complete

  //default the joystick values
  joyData.joyX = 512;
  joyData.joyY = 512;
  joyData.rightPressed = false;
  joyData.downPressed = false;
  joyData.leftPressed = false;
  joyData.upPressed = false;
  joyData.selPressed = false;

  //default the odometry values
  // odom_data.millis = 0;
  // odom_data.pathDistance = 0;
  // odom_data.x = 5;
  // odom_data.y = 0;
  // odom_data.theta = 0;
  // odom_data.velL = 0;
  // odom_data.velR = 0;
}

//sends the currently stored odometry data to the remote
bool sendOdometry(){
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &odom_data, sizeof(odom_data));
  return result == ESP_OK;
}