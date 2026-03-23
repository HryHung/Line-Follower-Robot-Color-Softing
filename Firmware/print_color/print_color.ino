#include <Wire.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

// Giá trị đã calib
float redCalib[3]  = {47.8, 20.9, 14.9};
float blueCalib[3] = {38.0, 45.6, 38.6};
float unkCalib[3]  = {26.0, 24.0, 16.8};

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!tcs.begin()) {
    Serial.println("ERROR: TCS34725 NOT FOUND!");
    while (1);
  }

  Serial.println("READY - color detection...");
}

void loop() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // In giá trị đo được
  Serial.printf("Measured: R=%u G=%u B=%u\n", r, g, b);

  // Chuyển sang float
  float fR = r;
  float fG = g;
  float fB = b;

  // Tính khoảng cách Euclidean đến từng màu
  float dRed = distance(fR, fG, fB, redCalib);
  float dBlue = distance(fR, fG, fB, blueCalib);
  float dUnk = distance(fR, fG, fB, unkCalib);

  // Xác định màu gần nhất
  float minDist = dRed;
  String color = "RED";

  if (dBlue < minDist) {
    minDist = dBlue;
    color = "BLUE";
  }
  if (dUnk < minDist) {
    minDist = dUnk;
    color = "UNKNOWN";
  }

  Serial.printf("Detected color: %s\n\n", color.c_str());

  delay(500);
}

// Hàm tính khoảng cách Euclidean
float distance(float r, float g, float b, float calib[3]) {
  return sqrt(pow(r - calib[0], 2) + pow(g - calib[1], 2) + pow(b - calib[2], 2));
} 
