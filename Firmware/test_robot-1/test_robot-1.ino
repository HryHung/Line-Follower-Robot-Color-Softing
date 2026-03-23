/*
 * ESP32 - BỘ ĐIỀU KHIỂN ROBOT BÁM LINE
 * =====================================
 * HỆ THỐNG ĐIỀU KHIỂN XẾP TẦNG (CASCADE CONTROL)
 *
 * 1. Vòng ngoài (PD - Bám line):
 * - Đọc 5 cảm biến line -> tính 'err_line'.
 * - Bộ PD tính toán 'steering_adjustment' dựa trên 'err_line'.
 *
 * 2. Vòng trong (PI - Tốc độ):
 * - rpm_set_1 = base_rpm + steering_adjustment
 * - rpm_set_2 = base_rpm - steering_adjustment
 * - Hai bộ PI độc lập điều khiển tốc độ từng động cơ bám theo
 * rpm_set_1 và rpm_set_2 bằng cách sử dụng encoder.
 *
 * Yêu cầu Serial Output: rpm, err_line, norm_sensor(s1->s5)
 */

#include <Arduino.h>

// ===== PIN MAP (ĐỘNG CƠ) =====
// --- Động cơ 1 (Motor A - Trái) ---
const int PIN_AIN1 = 14;
const int PIN_AIN2 = 27;
const int PIN_PWMA = 25;
const int PIN_ENC_A_1 = 16;
const int PIN_ENC_B_1 = 17;

// --- Động cơ 2 (Motor B - Phải) ---
const int PIN_BIN1 = 13;
const int PIN_BIN2 = 5;
const int PIN_PWMB = 26;
const int PIN_ENC_A_2 = 19;
const int PIN_ENC_B_2 = 18;

// --- Chung ---
const int PIN_STBY = 4;

// ===== PIN MAP (CẢM BIẾN LINE) =====
const int S1 = 36;
const int S2 = 39;
const int S3 = 34;
const int S4 = 35;
const int S5 = 32;

// ===== PWM (LEDC) =====
const int PWM_FREQ = 20000;  // 20 kHz
const int PWM_RES  = 10;     // 10-bit (0..1023)
const double PWM_MAX = (1<<PWM_RES) - 1; // 1023

// ===== ENCODER / TỐC ĐỘ =====
const int   CPR = 390;             // Tỉ số xung
const float SPEED_DT_MS = 100.0;   // chu kỳ điều khiển (ms)

// ===== CẢM BIẾN LINE =====
int rawMin[5] = {65, 69, 69, 80, 88};     // CALIBRATION MIN
int rawMax[5] = {2672, 2823, 2873, 3025, 3014}; // CALIBRATION MAX
int weight[5] = {-20, -10, 0, 10, 20}; // Trọng số
int norm_sensor[5]; // Mảng lưu giá trị normalized
double err_line = 0.0;
double last_err_line = 0.0;

// ===== BỘ ĐIỀU KHIỂN TỐC ĐỘ (VÒNG TRONG - PI) =====
double rpm_meas_1 = 0.0; // RPM đo Motor 1
double rpm_meas_2 = 0.0; // RPM đo Motor 2

// Tham số PI (Tune riêng cho mỗi động cơ nếu cần)
double Kp_1 = 0.2;
double Ki_1 = 11;
double integ_1 = 0.0;

double Kp_2 = 0.2;
double Ki_2 = 10.9;
double integ_2 = 0.0;

// ===== BỘ ĐIỀU KHIỂN BÁM LINE (VÒNG NGOÀI - PD) =====
double base_rpm = 80.0;  // Tốc độ cơ sở (RPM)
// !!! BẮT BUỘC PHẢI TUNE 2 HẰNG SỐ NÀY !!!
double Kp_line = 4.0;    // Gain P cho line
double Kd_line = 0.8;    // Gain D cho line

// ===== Biến encoder =====
volatile long encCount1 = 0;
long lastEnc1 = 0;
volatile long encCount2 = 0;
long lastEnc2 = 0;

unsigned long lastSpeedMs = 0;

// ===== Nhập lệnh Serial =====
String inLine; bool haveLine = false;

// ---------- ISR: Động cơ 1 ----------
void IRAM_ATTR encISR1() {
  if (digitalRead(PIN_ENC_B_1)) encCount1++;
  else                          encCount1--;
}

// ---------- ISR: Động cơ 2 ----------
void IRAM_ATTR encISR2() {
  if (digitalRead(PIN_ENC_B_2)) encCount2++;
  else                          encCount2--;
}

// ---------- Điều khiển TB6612 - Động cơ 1 ----------
static inline void motor1Brake() {
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); ledcWrite(PIN_PWMA, 0);
}
static inline void motor1Drive(int signed_pwm) {
  int mag = constrain(abs(signed_pwm), 0, (int)PWM_MAX);
  if      (signed_pwm > 0) { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW); ledcWrite(PIN_PWMA, mag); }
  else if (signed_pwm < 0) { digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); ledcWrite(PIN_PWMA, mag); }
  else { motor1Brake(); }
}

// ---------- Điều khiển TB6612 - Động cơ 2 ----------
static inline void motor2Brake() {
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW); ledcWrite(PIN_PWMB, 0);
}
static inline void motor2Drive(int signed_pwm) {
  int mag = constrain(abs(signed_pwm), 0, (int)PWM_MAX);
  if      (signed_pwm > 0) { digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW); ledcWrite(PIN_PWMB, mag); }
  else if (signed_pwm < 0) { digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, HIGH); ledcWrite(PIN_PWMB, mag); }
  else { motor2Brake(); }
}

// ---------- Điều khiển TB6612 - Chung ----------
static inline void motorStandby(bool en) { digitalWrite(PIN_STBY, en ? HIGH : LOW); }
static inline void motorBrake() {
  motor1Brake();
  motor2Brake();
}

// ---------- HÀM CẢM BIẾN LINE (Từ code 2) ----------
int clampMap(int x, int in_min, int in_max, int out_min, int out_max) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  long v = (long)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return (int)v;
}

double lineErr_Cal(int sensor[5]) {
    long sumWeight = 0;
    long sumValue = 0;

    for (int i = 0; i < 5; i++) {
        sumWeight += (long)sensor[i] * weight[i];
        sumValue  += sensor[i];
    }

    if (sumValue == 0) {
      // Mất line (tất cả đều là 0)
      // Giữ sai số cuối cùng
      return last_err_line; 
    }

    return (double)sumWeight / sumValue;
}

//=========================================================
//                       SETUP
//=========================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Cài đặt ADC
  analogReadResolution(12);

  // STBY chung
  pinMode(PIN_STBY, OUTPUT);

  // Motor 1 Pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_ENC_A_1, INPUT); // Cần pull-up ngoài
  pinMode(PIN_ENC_B_1, INPUT); // Cần pull-up ngoài

  // Motor 2 Pins
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_ENC_A_2, INPUT); // Cần pull-up ngoài
  pinMode(PIN_ENC_B_2, INPUT); // Cần pull-up ngoài

  // API 3.x: Tự động setup và attach
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);

  // Gắn ngắt
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_1), encISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_2), encISR2, RISING);

  motorStandby(true);
  motorBrake(); // Phanh cả hai

  lastSpeedMs = millis();
  Serial.println(F("Ready (Line Following Robot). Type base RPM."));
}

//=========================================================
//                        LOOP
//=========================================================
void loop() {
  // ---- nhận lệnh RPM CƠ SỞ (base_rpm) qua Serial ----
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c=='\n' || c=='\r') {
      if (inLine.length()) haveLine = true;
    } else {
      inLine += c;
    }
  }
  if (haveLine) {
    inLine.trim();
    base_rpm = (double) inLine.toInt();  // Nhập RPM cơ sở
    inLine = ""; haveLine = false;
    
    // Khi đặt lệnh mới, reset bộ tích phân
    integ_1 = 0.0; 
    integ_2 = 0.0;
    last_err_line = 0.0;
    Serial.print("Base RPM set to: "); Serial.println(base_rpm);
  }

  // ---- VÒNG ĐIỀU KHIỂN CHÍNH (theo chu kỳ) ----
  unsigned long now = millis();
  if (now - lastSpeedMs >= SPEED_DT_MS) {
    float dt = (now - lastSpeedMs) / 1000.0f;   // giây
    lastSpeedMs = now;

    // ==========================================================
    // === VÒNG NGOÀI: ĐỌC SENSOR VÀ TÍNH TOÁN LÁI (PD) ===
    // ==========================================================
    
    // 1. Đọc và chuẩn hóa 5 cảm biến
    int s_raw[5];
    s_raw[0] = analogRead(S1);
    s_raw[1] = analogRead(S2);
    s_raw[2] = analogRead(S3);
    s_raw[3] = analogRead(S4);
    s_raw[4] = analogRead(S5);

    for (int i = 0; i < 5; i++) {
        norm_sensor[i] = clampMap(s_raw[i], rawMin[i], rawMax[i], 0, 1000);
    }

    // 2. Tính sai số line
    err_line = lineErr_Cal(norm_sensor);

    // 3. Tính toán điều khiển PD cho lái
    double deriv_line = (err_line - last_err_line) / dt;
    double steering_adjustment = Kp_line * err_line + Kd_line * deriv_line;
    last_err_line = err_line;

    // 4. Tính toán điểm đặt RPM mới cho 2 động cơ
    // Giả sử Motor 1 (Trái), Motor 2 (Phải)
    // Nếu err_line > 0 (lệch phải), cần rẽ phải -> Motor 1 nhanh, Motor 2 chậm
    double rpm_set_1 = base_rpm + steering_adjustment;
    double rpm_set_2 = base_rpm - steering_adjustment;


    // ==========================================================
    // === VÒNG TRONG: ĐIỀU KHIỂN TỐC ĐỘ (PI) ===
    // ==========================================================

    // --- Tính toán RPM thực tế Motor 1 ---
    long cnt1 = encCount1;
    long dp1  = cnt1 - lastEnc1;
    lastEnc1  = cnt1;
    rpm_meas_1 = (dp1 / dt) * 60.0 / CPR;

    // --- Tính toán RPM thực tế Motor 2 ---
    long cnt2 = encCount2;
    long dp2  = cnt2 - lastEnc2;
    lastEnc2  = cnt2;
    rpm_meas_2 = (dp2 / dt) * 60.0 / CPR;

    // --- Logic PI Motor 1 (Bám theo rpm_set_1) ---
    double e1 = rpm_set_1 - rpm_meas_1;
    integ_1 += e1 * Ki_1 * dt;
    integ_1 = constrain(integ_1, -PWM_MAX, PWM_MAX); // Anti-windup
    double u1 = Kp_1 * e1 + integ_1;
    motor1Drive((int)u1);

    // --- Logic PI Motor 2 (Bám theo rpm_set_2) ---
    double e2 = rpm_set_2 - rpm_meas_2;
    integ_2 += e2 * Ki_2 * dt;
    integ_2 = constrain(integ_2, -PWM_MAX, PWM_MAX); // Anti-windup
    double u2 = Kp_2 * e2 + integ_2;
    motor2Drive((int)u2);

    // ==========================================================
    // === FEEDBACK SERIAL (Theo yêu cầu) ===
    // ==========================================================
    
    Serial.print("RPM: ");
    Serial.print(rpm_meas_1, 1);
    Serial.print(", ");
    Serial.print(rpm_meas_2, 1);

    Serial.print(" | Err: ");
    Serial.print(err_line, 2);

    Serial.print(" | Norm: ");
    for (int i = 0; i < 5; i++) {
        Serial.print(norm_sensor[i]);
        Serial.print(" ");
    }
    Serial.println();
  }
}