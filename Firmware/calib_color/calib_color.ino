#include <Wire.h>
#include <Adafruit_TCS34725.h>


Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

// Lưu trung bình: chỉ 3 giá trị R,G,B
float calibRed[3];
float calibBlue[3];
float calibUnk[3];   // NEW

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!tcs.begin()) {
    Serial.println("ERROR: TCS34725 NOT FOUND!");
    while (1);
  }

  Serial.println("READY - press r / b / u / p");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'r') {
      Serial.println("Calibrating RED...");
      calibrateColor(calibRed);
      Serial.println("→ RED calibrated!");
    }

    if (cmd == 'b') {
      Serial.println("Calibrating BLUE...");
      calibrateColor(calibBlue);
      Serial.println("→ BLUE calibrated!");
    }

    if (cmd == 'u') {
      Serial.println("Calibrating UNKNOWN...");
      calibrateColor(calibUnk);
      Serial.println("→ UNKNOWN calibrated!");
    }

    if (cmd == 'p') {
      printCalib();
    }
  }
}

void calibrateColor(float *arr) {
  uint32_t sumR = 0, sumG = 0, sumB = 0;
  uint16_t r, g, b, c;

  for (int i = 0; i < 50; i++) {
    tcs.getRawData(&r, &g, &b, &c);

    sumR += r;
    sumG += g;
    sumB += b;

    Serial.printf("%d) R=%u G=%u B=%u\n", i + 1, r, g, b);
    delay(100);
  }

  arr[0] = sumR / 50.0;
  arr[1] = sumG / 50.0;
  arr[2] = sumB / 50.0;
}

void printCalib() {
  Serial.println("\n=== CALIBRATION RESULT ===");

  Serial.printf("float redCalib[3]  = {%.1f, %.1f, %.1f};\n",
                calibRed[0], calibRed[1], calibRed[2]);

  Serial.printf("float blueCalib[3] = {%.1f, %.1f, %.1f};\n",
                calibBlue[0], calibBlue[1], calibBlue[2]);

  Serial.printf("float unkCalib[3]  = {%.1f, %.1f, %.1f};\n",
                calibUnk[0], calibUnk[1], calibUnk[2]);

  Serial.println("================================\n");
}
