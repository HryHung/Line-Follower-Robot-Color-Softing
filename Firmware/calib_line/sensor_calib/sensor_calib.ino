// ESP32 TRCT5000 Calibration + Normalized Read + Position
// Arduino IDE (ESP32)
// Pins: S1..S5 on ADC1: GPIO32,33,34,35,36

#include <Preferences.h>

#define N_SENSORS 5
const int pins[N_SENSORS] = {36, 39, 34, 35, 32};

Preferences prefs;

int rawMin[N_SENSORS];
int rawMax[N_SENSORS];

// THAY ĐỔI: Biến tạm toàn cục để lưu giá trị calib
int whiteVal[N_SENSORS];
int blackVal[N_SENSORS];

const int AVG_SAMPLES = 20; // số mẫu trung bình
const int SAMPLE_DELAY_MS = 5;

// ---------------- helpers ----------------
int readAvg(int pin) {
  long sum = 0;
  for (int i = 0; i < AVG_SAMPLES; i++) {
    sum += analogRead(pin);
    delay(SAMPLE_DELAY_MS);
  }
  return (int)(sum / AVG_SAMPLES);
}

int clampMap(int x, int in_min, int in_max, int out_min, int out_max) {
  if (in_min == in_max) return (out_min + out_max) / 2;
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  long v = (long)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return (int)v;
}

// ---------------- calibration routines ----------------
void initDefaults() {
  for (int i = 0; i < N_SENSORS; i++) {
    rawMin[i] = 0;
    rawMax[i] = 4095;
    // Khởi tạo luôn giá trị tạm
    whiteVal[i] = 0;
    blackVal[i] = 4095;
  }
}

void saveCalibration() {
  prefs.begin("trctcal", false);
  for (int i = 0; i < N_SENSORS; i++) {
    prefs.putInt((String("min") + i).c_str(), rawMin[i]);
    prefs.putInt((String("max") + i).c_str(), rawMax[i]);
  }
  prefs.end();
  Serial.println("Calibration saved to flash.");
}

bool loadCalibration() {
  prefs.begin("trctcal", true);
  int test = prefs.getInt("min0", -1);
  prefs.end();
  if (test == -1) return false;

  prefs.begin("trctcal", true);
  for (int i = 0; i < N_SENSORS; i++) {
    rawMin[i] = prefs.getInt((String("min") + i).c_str(), 0);
    rawMax[i] = prefs.getInt((String("max") + i).c_str(), 4095);
  }
  prefs.end();
  return true;
}

void printCalibration() {
  Serial.println("Calibration ranges (min..max):");
  for (int i = 0; i < N_SENSORS; i++) {
    Serial.printf("S%d: %d .. %d\n", i + 1, rawMin[i], rawMax[i]);
  }
}

// THAY ĐỔI: Hàm mới để in giá trị calib ra dạng mảng C
void printCalibrationForCopy() {
  Serial.print("int rawMin[");
  Serial.print(N_SENSORS);
  Serial.print("] = {");
  for (int i = 0; i < N_SENSORS; i++) {
    Serial.print(rawMin[i]);
    if (i < N_SENSORS - 1) Serial.print(", ");
  }
  Serial.println("};");

  Serial.print("int rawMax[");
  Serial.print(N_SENSORS);
  Serial.print("] = {");
  for (int i = 0; i < N_SENSORS; i++) {
    Serial.print(rawMax[i]);
    if (i < N_SENSORS - 1) Serial.print(", ");
  }
  Serial.println("};");
}

// THAY ĐỔI: Tách hàm - chỉ Calib WHITE
void calibrateWhite() {
  Serial.println("\n=== Calibrating WHITE ===");
  Serial.println("Place all sensors on WHITE (background).");
  Serial.println("Reading WHITE... move sensors slowly across white area.");

  for (int i = 0; i < N_SENSORS; i++) whiteVal[i] = 4095;

  for (int t = 0; t < 20; t++) {
    Serial.print("WHITE: ");
    for (int i = 0; i < N_SENSORS; i++) {
      int v = readAvg(pins[i]);
      if (v < whiteVal[i]) whiteVal[i] = v;
      Serial.printf("%4d ", v);
    }
    Serial.println();
  }
  Serial.println("WHITE reading finished. (Gõ 'b' để calib black)");
}

// THAY ĐỔI: Tách hàm - chỉ Calib BLACK và lưu
void calibrateBlack() {
  Serial.println("\n=== Calibrating BLACK ===");
  Serial.println("Place sensors on BLACK (line).");
  Serial.println("Reading BLACK... move sensors slowly across black area.");

  for (int i = 0; i < N_SENSORS; i++) blackVal[i] = 0;

  for (int t = 0; t < 20; t++) {
    Serial.print("BLACK: ");
    for (int i = 0; i < N_SENSORS; i++) {
      int v = readAvg(pins[i]);
      if (v > blackVal[i]) blackVal[i] = v;
      Serial.printf("%4d ", v);
    }
    Serial.println();
  }
  Serial.println("BLACK reading finished.");

  // --- Áp dụng và Lưu calibration ---
  Serial.println("Applying and saving calibration...");
  for (int i = 0; i < N_SENSORS; i++) {
    // Dùng giá trị từ whiteVal và blackVal đã lưu
    rawMin[i] = whiteVal[i] - 20; if (rawMin[i] < 0) rawMin[i] = 0;
    rawMax[i] = blackVal[i] + 20; if (rawMax[i] > 4095) rawMax[i] = 4095;
    if (rawMax[i] <= rawMin[i]) {
      rawMin[i] = 0;
      rawMax[i] = 4095;
    }
  }

  printCalibration();
  saveCalibration(); // Tự động lưu
  Serial.println("Calibration FINISHED and saved.");
}


// ---------------- read calibrated and compute position ----------------
int readCalibrated(int index) {
  int raw = readAvg(pins[index]);
  int norm = clampMap(raw, rawMin[index], rawMax[index], 0, 1000);
  return norm;
}

long computePositionRaw() {
  long weights[N_SENSORS] = { -2000, -1000, 0, 1000, 2000};
  long sumVal = 0;
  long sumWeight = 0;
  for (int i = 0; i < N_SENSORS; i++) {
    int v = readCalibrated(i);
    sumVal += (long)v * weights[i];
    sumWeight += v;
  }
  if (sumWeight < 50) return 1000000; // LOST
  return sumVal / sumWeight;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  delay(100);

  for (int i = 0; i < N_SENSORS; i++) pinMode(pins[i], INPUT);

  Serial.println("=== TRCT5000 Calibration + Position ===");
  if (!loadCalibration()) {
    Serial.println("No saved calibration found. Using defaults.");
    initDefaults();
  } else {
    Serial.println("Loaded calibration from flash.");
    printCalibration();
  }

  // THAY ĐỔI: Cập nhật danh sách lệnh
  Serial.println("\nCommands:");
  Serial.println("  w - calibrate WHITE");
  Serial.println("  b - calibrate BLACK (and save)");
  Serial.println("  l - load calibration");
  Serial.println("  p - print calibration as C-Array (để copy)");
  Serial.println("  r - read continuous normalized values + position");
  Serial.println("  n - print normalized reads (lệnh 'p' cũ)");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    // THAY ĐỔI: Cập nhật các lệnh
    if (cmd == 'w') calibrateWhite();
    else if (cmd == 'b') calibrateBlack();
    else if (cmd == 'l') {
      if (loadCalibration()) {
        Serial.println("Loaded calibration.");
        printCalibration();
      } else Serial.println("No calibration in flash.");
    }
    // THAY ĐỔI: 'p' gọi hàm mới
    else if (cmd == 'p') {
      printCalibrationForCopy();
    }
    // THAY ĐỔI: Thêm 'n' (normalized) để giữ lại chức năng của 'p' cũ
    else if (cmd == 'n') {
      Serial.println("Normalized readings (0..1000):");
      for (int i = 0; i < N_SENSORS; i++) Serial.printf("S%d=%d\t", i + 1, readCalibrated(i));
      Serial.println();
    }
    else if (cmd == 'r') {
      Serial.println("Streaming normalized reads + position (press any key to stop)...");
      while (Serial.available() == 0) {
        long pos = computePositionRaw();
        if (pos == 1000000) Serial.println("LOST");
        else {
          float posF = pos / 1000.0;
          for (int i = 0; i < N_SENSORS; i++) Serial.printf("%4d ", readCalibrated(i));
          Serial.printf(" | pos=%.3f\n", posF);
        }
        delay(80);
      }
      while (Serial.available()) Serial.read();
      Serial.println("Stopped streaming.");
    }
  }
  delay(10);
}