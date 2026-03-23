#define S1 36
#define S2 39
#define S3 34
#define S4 35
#define S5 32

// Giá trị calibration
int rawMin[5] = {44, 43, 34, 35, 28};
int rawMax[5] = {2821, 2753, 2138, 2328, 2600};

int weight[5] = {-20, -10, 0, 10, 20};

int clampMap(int x, int in_min, int in_max, int out_min, int out_max) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  long v = (long)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return (int)v;
}

int lineErr_Cal(int sensor[5]) {
    long sumWeight = 0;
    long sumValue = 0;

    for (int i = 0; i < 5; i++) {
        sumWeight += (long)sensor[i] * weight[i];
        sumValue  += sensor[i];
    }

    if (sumValue == 0) return 0; // tránh chia 0

    return sumWeight / sumValue;  // giá trị sai số nằm ~ [-2…2]
}


void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  Serial.println("=== TRCT5000 ===");
}


void loop() {
    static unsigned long lastTime = 0; 
    
    if (millis() - lastTime >= 100) { 
        int s[5];
        s[0] = analogRead(S1);
        s[1] = analogRead(S2);
        s[2] = analogRead(S3);
        s[3] = analogRead(S4);
        s[4] = analogRead(S5);

        int norm[5];
        for (int i = 0; i < 5; i++) {
            norm[i] = clampMap(s[i], rawMin[i], rawMax[i], 0, 1000);
        }

        // --- TÍNH SAI SỐ LINE ---
        int lineErr = lineErr_Cal(norm);

        // --- IN KẾT QUẢ ---
        Serial.print("Normalized: ");
        for (int i = 0; i < 5; i++) {
            Serial.print(norm[i]);
            Serial.print("\t");
        }

        Serial.print(" | Err = ");
        Serial.println(lineErr);

        lastTime = millis(); 
    }
}

