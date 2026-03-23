/*
 * ESP32 - BỘ ĐIỀU KHIỂN ROBOT BÁM LINE (CẬP NHẬT PID VÒNG NGOÀI)
 * ==============================================================
 * HỆ THỐNG ĐIỀU KHIỂN XẾP TẦNG (CASCADE CONTROL)
 *
 * 1. Vòng ngoài (PID - Bám line):
 * - Input: err_line (từ cảm biến)
 * - Output: steering_adjustment (điều chỉnh tốc độ)
 *
 * 2. Vòng trong (PI - Tốc độ):
 * - Input: rpm_set (tính từ vòng ngoài)
 * - Output: PWM động cơ
 */

#include <Arduino.h>

// ===== PIN MAP (ĐỘNG CƠ) =====
const int PIN_AIN1 = 14;
const int PIN_AIN2 = 27;
const int PIN_PWMA = 25;
const int PIN_ENC_A_1 = 16;
const int PIN_ENC_B_1 = 17;

const int PIN_BIN1 = 13;
const int PIN_BIN2 = 5;
const int PIN_PWMB = 26;
const int PIN_ENC_A_2 = 19;
const int PIN_ENC_B_2 = 18;

const int PIN_STBY = 4;

// ===== PIN MAP (CẢM BIẾN LINE) =====
const int S1 = 36;
const int S2 = 39;
const int S3 = 34;
const int S4 = 35;
const int S5 = 32;

// ===== PWM (LEDC) =====
const int PWM_FREQ = 20000;
const int PWM_RES  = 10;
const double PWM_MAX = (1<<PWM_RES) - 1; // 1023

// ===== ENCODER / TỐC ĐỘ =====
const int   CPR = 390;
const float SPEED_DT_MS = 50;

// ===== CẢM BIẾN LINE =====
int rawMin[5] = {83, 59, 43, 43, 37};
int rawMax[5] = {3106, 2877, 2691, 2607, 2517};
int weight[5] = {-20, -10, 0, 10, 20};
int norm_sensor[5];
double err_line = 0.0;
double last_err_line = 0.0;

// ===== BỘ ĐIỀU KHIỂN TỐC ĐỘ (VÒNG TRONG - PI) =====
double rpm_meas_1 = 0.0;
double rpm_meas_2 = 0.0;

// Tune riêng cho động cơ (Vòng trong)
double Kp_1 = 0.19; double Ki_1 = 11; double integ_1 = 0.0;
double Kp_2 = 0.2;  double Ki_2 = 11; double integ_2 = 0.0;

// ===== BỘ ĐIỀU KHIỂN BÁM LINE (VÒNG NGOÀI - PID) =====
double base_rpm = 155.0;

// --- PID Vòng Ngoài (Tune cái này để bám vạch mượt) ---
double Kp_line = 4;   // P: Phản ứng nhanh với lỗi
double Ki_line = 0.75;  // I: Khử sai số tĩnh (giúp bám sát tâm khi cua), để nhỏ thôi!
double Kd_line = 1.5;   // D: Giảm rung lắc (quan trọng)

double integ_line = 0.0; // Tích phân lỗi line

// ===== Biến encoder =====
volatile long encCount1 = 0; long lastEnc1 = 0;
volatile long encCount2 = 0; long lastEnc2 = 0;
unsigned long lastSpeedMs = 0;

// ===== Nhập lệnh Serial =====
String inLine; bool haveLine = false;

// ---------- ISR ----------
void IRAM_ATTR encISR1() { if (digitalRead(PIN_ENC_B_1)) encCount1++; else encCount1--; }
void IRAM_ATTR encISR2() { if (digitalRead(PIN_ENC_B_2)) encCount2++; else encCount2--; }

// ---------- Điều khiển TB6612 ----------
static inline void motor1Brake() { digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); ledcWrite(PIN_PWMA, 0); }
static inline void motor1Drive(int signed_pwm) {
  int mag = constrain(abs(signed_pwm), 0, (int)PWM_MAX);
  if      (signed_pwm > 0) { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW); ledcWrite(PIN_PWMA, mag); }
else if (signed_pwm < 0) { digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); ledcWrite(PIN_PWMA, mag); }
  else { motor1Brake(); }
}

static inline void motor2Brake() { digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW); ledcWrite(PIN_PWMB, 0); }
static inline void motor2Drive(int signed_pwm) {
  int mag = constrain(abs(signed_pwm), 0, (int)PWM_MAX);
  if      (signed_pwm > 0) { digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW); ledcWrite(PIN_PWMB, mag); }
  else if (signed_pwm < 0) { digitalWrite(PIN_BIN1, LOW);  digitalWrite(PIN_BIN2, HIGH); ledcWrite(PIN_PWMB, mag); }
  else { motor2Brake(); }
}

static inline void motorStandby(bool en) { digitalWrite(PIN_STBY, en ? HIGH : LOW); }
static inline void motorBrake() { motor1Brake(); motor2Brake(); }

// ---------- HÀM CẢM BIẾN LINE ----------
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
    if (sumValue == 0) return last_err_line;
    return (double)sumWeight / sumValue;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  analogReadResolution(12);

  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_ENC_A_1, INPUT); pinMode(PIN_ENC_B_1, INPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT); pinMode(PIN_ENC_A_2, INPUT); pinMode(PIN_ENC_B_2, INPUT);

  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_1), encISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_2), encISR2, RISING);

  motorStandby(true);
  motorBrake();
  lastSpeedMs = millis();
  Serial.println(F("Ready (Cascade PID: Line PID -> Speed PI). Type base RPM."));
}

void loop() {
  // ---- NHẬN LỆNH TỪ SERIAL ----
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c=='\n' || c=='\r') { if (inLine.length()) haveLine = true; } else { inLine += c; }
  }
  if (haveLine) {
    inLine.trim();
    base_rpm = (double) inLine.toInt();
    inLine = ""; haveLine = false;
    
    // Reset toàn bộ tích phân khi đổi tốc độ để tránh giật
    integ_1 = 0.0; integ_2 = 0.0;
    integ_line = 0.0; // <--- Reset I vòng ngoài
    last_err_line = 0.0;
    Serial.print("Base RPM set to: "); Serial.println(base_rpm);
  }

  // ---- VÒNG ĐIỀU KHIỂN CHÍNH ----
  unsigned long now = millis();
  if (now - lastSpeedMs >= SPEED_DT_MS) {
    float dt = (now - lastSpeedMs) / 1000.0f;
    lastSpeedMs = now;

    // ==========================================================
// === VÒNG NGOÀI: ĐỌC SENSOR VÀ TÍNH PID HƯỚNG ===
    // ==========================================================
    int s_raw[5] = {analogRead(S1), analogRead(S2), analogRead(S3), analogRead(S4), analogRead(S5)};
    for (int i = 0; i < 5; i++) norm_sensor[i] = clampMap(s_raw[i], rawMin[i], rawMax[i], 0, 1000);

    err_line = lineErr_Cal(norm_sensor);

    // --- Tính toán PID cho Line ---
    // 1. Proportional
    double P_term = Kp_line * err_line;

    // 2. Integral (Mới thêm)
    integ_line += err_line * dt;
    // Anti-windup cho vòng ngoài (quan trọng): Giới hạn tích phân
    // Nếu không giới hạn, khi mất line integ sẽ tăng vô tận làm xe quay vòng tròn
    integ_line = constrain(integ_line, -50.0, 50.0); 
    double I_term = Ki_line * integ_line;

    // 3. Derivative
    double deriv_line = (err_line - last_err_line) / dt;
    double D_term = Kd_line * deriv_line;

    // Tổng hợp tín hiệu điều khiển lái
    double steering_adjustment = P_term + I_term + D_term;
    last_err_line = err_line;

    // Tính setpoint cho 2 bánh
    double rpm_set_1 = base_rpm + steering_adjustment;
    double rpm_set_2 = base_rpm - steering_adjustment;

    // ==========================================================
    // === VÒNG TRONG: ĐIỀU KHIỂN TỐC ĐỘ (PI) ===
    // ==========================================================
    long cnt1 = encCount1; long dp1 = cnt1 - lastEnc1; lastEnc1 = cnt1;
    rpm_meas_1 = (dp1 / dt) * 60.0 / CPR;

    long cnt2 = encCount2; long dp2 = cnt2 - lastEnc2; lastEnc2 = cnt2;
    rpm_meas_2 = (dp2 / dt) * 60.0 / CPR;

    // Motor 1 (Trái)
    double e1 = rpm_set_1 - rpm_meas_1;
    integ_1 += e1 * Ki_1 * dt;
    integ_1 = constrain(integ_1, -PWM_MAX, PWM_MAX);
    double u1 = Kp_1 * e1 + integ_1;
    motor1Drive((int)u1);

    // Motor 2 (Phải)
    double e2 = rpm_set_2 - rpm_meas_2;
    integ_2 += e2 * Ki_2 * dt;
    integ_2 = constrain(integ_2, -PWM_MAX, PWM_MAX);
    double u2 = Kp_2 * e2 + integ_2;
    motor2Drive((int)u2);

// ==========================================================
// === FEEDBACK SERIAL ===
// ==========================================================

Serial.print("S: ");
Serial.print(norm_sensor[0]); Serial.print(" ");
Serial.print(norm_sensor[1]); Serial.print(" ");
Serial.print(norm_sensor[2]); Serial.print(" ");
Serial.print(norm_sensor[3]); Serial.print(" ");
Serial.print(norm_sensor[4]);

Serial.print(" | Err: ");   Serial.print(err_line, 2);
Serial.print(" | Steer: "); Serial.print(steering_adjustment, 1);
Serial.print(" | RPM Set: ");
Serial.print(rpm_set_1, 0); Serial.print("/");
Serial.print(rpm_set_2, 0);
Serial.print(" | RPM Real: ");
Serial.print(rpm_meas_1, 0); Serial.print("/");
Serial.println(rpm_meas_2, 0);

  }
}
