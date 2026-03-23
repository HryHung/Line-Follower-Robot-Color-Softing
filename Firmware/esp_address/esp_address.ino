#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  
  // Quan trọng: Phải để chế độ STA để lấy đúng MAC dùng cho ESP-NOW
  WiFi.mode(WIFI_MODE_STA);
  delay(100);

  Serial.println("\n\n==================================");
  Serial.print("Dia chi MAC mac dinh: ");
  Serial.println(WiFi.macAddress());
                                                                                                                                        
  // Lấy MAC dạng mảng byte để copy vào code
  uint8_t mac[6];
  WiFi.macAddress(mac);

  Serial.printf("{0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}\n", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println("==================================\n");
}

void loop() {
  // Không làm gì cả
}