/*
 * ESP32 + TB6612 + Quadrature Encoder
 * PI điều khiển tốc độ (RPM) cho 2 động cơ - ĐÃ SỬA LỖI LOGIC
 * ***** Tương thích ESP32 CORE 3.x (API MỚI) *****
 * Gõ một số RPM trên Serial Monitor (115200) rồi Enter, ví dụ: 150
 * Code này sẽ điều khiển CẢ HAI động cơ chạy cùng tốc độ.
 */

#include <Arduino.h>

// ===== PIN MAP (Đã cập nhật theo sơ đồ của bạn) =====
// --- Động cơ 1 (Motor A) ---
const int PIN_AIN1 = 14;
const int PIN_AIN2 = 27;
const int PIN_PWMA = 25;
const int PIN_ENC_A_1 = 16;  // Cần pull-up ngoài
const int PIN_ENC_B_1 = 17;  // Cần pull-up ngoài

// --- Động cơ 2 (Motor B) ---
const int PIN_BIN1 = 13;
const int PIN_BIN2 = 5;
const int PIN_PWMB = 26;
const int PIN_ENC_A_2 = 19;  // Cần pull-up ngoài
const int PIN_ENC_B_2 = 18;  // Cần pull-up ngoài

// --- Chung ---
const int PIN_STBY = 4;

// ===== PWM (LEDC) =====
const int PWM_FREQ = 20000;  // 20 kHz êm tiếng
const int PWM_RES  = 10;     // 10-bit (0..1023)

// ===== ENCODER / TỐC ĐỘ =====
const int   CPR = 390;             // Tỉ số xung (11 * 35.5)
const float SPEED_DT_MS = 100.0;   // chu kỳ tính tốc độ (ms)

// ===== PI controller (tự viết, không cần thư viện) =====
double rpm_meas_1 = 0.0; // RPM đo Motor 1
double rpm_meas_2 = 0.0; // RPM đo Motor 2
double rpm_set  = 0.0;   // RPM đặt (chung cho cả 2)

// Tham số PI (Tune riêng cho mỗi động cơ nếu cần)
double Kp_1 = 0.195; // Kp cho Motor 1
double Ki_1 = 10;   // Ki cho Motor 1
double integ_1 = 0.0; // Tích phân Motor 1

double Kp_2 = 0.204; // Kp cho Motor 2
double Ki_2 = 11;   // Ki cho Motor 2
double integ_2 = 0.0; // Tích phân Motor 2

const double PWM_MAX = (1<<PWM_RES) - 1; // 1023

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

void setup() {
  Serial.begin(115200);
  delay(200);

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
  Serial.println(F("Ready (2-Motor PI Controller). Type target RPM."));
}

void loop() {
  // ---- nhận lệnh RPM có dấu qua Serial ----
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
    rpm_set = (double) inLine.toInt();  // Nhập RPM, ví dụ: 150
    inLine = ""; haveLine = false;
    
    // Khi đặt lệnh mới, reset bộ tích phân
    integ_1 = 0.0; 
    integ_2 = 0.0;
  }

  // ---- cập nhật tốc độ & điều khiển theo chu kỳ ----
  unsigned long now = millis();
  if (now - lastSpeedMs >= SPEED_DT_MS) {
    float dt = (now - lastSpeedMs) / 1000.0f;   // giây
    lastSpeedMs = now;

    // --- Tính toán Motor 1 ---
    long cnt1 = encCount1;
    long dp1  = cnt1 - lastEnc1;
    lastEnc1  = cnt1;
    rpm_meas_1 = (dp1 / dt) * 60.0 / CPR;

    // --- Tính toán Motor 2 ---
    long cnt2 = encCount2;
    long dp2  = cnt2 - lastEnc2;
    lastEnc2  = cnt2;
    rpm_meas_2 = (dp2 / dt) *60.0 / CPR; // << CÓ THỂ BẠN CỐ TÌNH SAI Ở ĐÂY? SỬA LẠI:
    // rpm_meas_2 = (dp2 / dt) * 60.0 / CPR;


    // ==========================================================
    // === LOGIC PI ĐIỀU KHIỂN TỐC ĐỘ (Motor 1) ===
    // ==========================================================
    double e1 = rpm_set - rpm_meas_1;
    integ_1 += e1 * Ki_1 * dt;
    if (integ_1 > PWM_MAX) integ_1 = PWM_MAX;
    if (integ_1 < -PWM_MAX) integ_1 = -PWM_MAX;
    double u1 = Kp_1 * e1 + integ_1;
    motor1Drive((int)u1);

    // ==========================================================
    // === LOGIC PI ĐIỀU KHIỂN TỐC ĐỘ (Motor 2) ===
    // ==========================================================
    double e2 = rpm_set - rpm_meas_2;
    integ_2 += e2 * Ki_2 * dt;
    if (integ_2 > PWM_MAX) integ_2 = PWM_MAX;
    if (integ_2 < -PWM_MAX) integ_2 = -PWM_MAX;
    double u2 = Kp_2 * e2 + integ_2;
    motor2Drive((int)u2);

    // ==========================================================

    // In log để plot: set, meas1, meas2, pwm1, pwm2
    Serial.print((int)rpm_set); Serial.print(',');
    Serial.print(rpm_meas_1, 1);  Serial.print(',');
    Serial.print(rpm_meas_2, 1);  Serial.print(',');
    Serial.print((int)u1); Serial.print(',');
    Serial.println((int)u2); 
  }
}