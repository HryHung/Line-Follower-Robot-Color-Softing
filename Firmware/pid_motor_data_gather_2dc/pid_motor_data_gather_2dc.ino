/*
 * ESP32 + TB6612 - LẤY DỮ LIỆU ĐÁP ỨNG 2 ĐỘNG CƠ (OPEN-LOOP)
 * ***** CHỜ 1 GIÂY TRƯỚC KHI CHẠY *****
 *
 * 1. Mở Serial Monitor (115200).
 * 2. Nhập một RPM mục tiêu chung (ví dụ: 170) rồi Enter.
 * 3. Code bắt đầu ghi log (Time, 0, 0) ngay lập tức.
 * 4. Sau 1 giây (t=1.0s), động cơ được cấp PWM và bắt đầu chạy.
 * 5. Code dừng test ở t=3.0s (tổng thời gian log).
 */

#include <Arduino.h>

// ===== PIN MAP =====
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
const int PWM_FREQ = 20000; // 20 kHz
const int PWM_RES  = 10;     // 10-bit (0..1023)
const double PWM_MAX = (1 << PWM_RES) - 1; // 1023

// ===== ENCODER / TỐC ĐỘ =====
const int   CPR = 390;        // Tỉ số xung (11 * 35.5)
const float MAX_RPM = 170.0;    // RPM không tải (để map)
const float LOG_DT_MS = 20;     // dt = 0.02s
const long  TEST_DURATION_MS = 3000; // Tổng thời gian test (log)
const long  MOTOR_START_DELAY_MS = 1000; // Chờ 1s (1000ms) mới chạy

// ===== Biến encoder =====
// Động cơ 1
volatile long encCount1 = 0;
long lastEnc1 = 0;
// Động cơ 2
volatile long encCount2 = 0;
long lastEnc2 = 0;

unsigned long testStartTime = 0;
unsigned long lastLogTime = 0;

// ===== Trạng thái hệ thống =====
enum State { IDLE, RUNNING };
State currentState = IDLE;
bool motorsActivated = false; // Cờ báo động cơ đã chạy chưa
int global_pwm_to_apply = 0; // Lưu PWM để loop() sử dụng

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

// Phanh cả 2 động cơ
static inline void motorBrake() {
  motor1Brake();
  motor2Brake();
}
// Chạy cả 2 động cơ với CÙNG một giá trị PWM
static inline void motorDrive(int signed_pwm) {
  motor1Drive(signed_pwm);
  motor2Drive(signed_pwm);
}


// ---------- Xử lý Serial Input ----------
void handleSerialInput() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inLine.length()) haveLine = true;
    } else {
      inLine += c;
    }
  }

  if (haveLine) {
    inLine.trim();
    double rpm_target = (double)inLine.toInt(); // Nhập RPM (0-170)
    inLine = ""; haveLine = false;

    // Chỉ bắt đầu test nếu đang ở trạng thái IDLE
    if (currentState == IDLE) {
      // Ánh xạ RPM mục tiêu sang PWM cố định
      global_pwm_to_apply = map(rpm_target, 0, MAX_RPM, 0, PWM_MAX);
      global_pwm_to_apply = constrain(global_pwm_to_apply, 0, (int)PWM_MAX);

      Serial.print("Bắt đầu test. Mục tiêu RPM: ");
      Serial.print(rpm_target);
      Serial.print(" -> Áp dụng PWM: ");
      Serial.print(global_pwm_to_apply);
      Serial.print(" (Sau ");
      Serial.print(MOTOR_START_DELAY_MS / 1000.0);
      Serial.println("s)");
      
      // Header mới cho Excel
      Serial.println("Time(s),RPM_Motor1,RPM_Motor2"); 

      // Reset và Bắt đầu
      encCount1 = 0;
      lastEnc1 = 0;
      encCount2 = 0;
      lastEnc2 = 0;
      
      testStartTime = millis();
      lastLogTime = testStartTime;
      motorsActivated = false; // Reset cờ
      currentState = RUNNING;
      
      motorBrake(); // QUAN TRỌNG: Bắt đầu bằng cách phanh
    }
  }
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

  Serial.println(F("Ready (2-Motor Logger, 1s Delay). Type target RPM (0-170)."));
}

void loop() {
  // 1. Luôn kiểm tra Serial
  handleSerialInput();

  // 2. Nếu đang trong trạng thái test
  if (currentState == RUNNING) {
    unsigned long now = millis();

    // --- (MỚI) Kích hoạt động cơ sau khoảng thời gian chờ ---
    if (!motorsActivated && (now - testStartTime >= MOTOR_START_DELAY_MS)) {
      motorsActivated = true;
      motorDrive(global_pwm_to_apply); // Kích hoạt động cơ!
    }

    // --- Kiểm tra xem đã hết thời gian test (3s) chưa ---
    if (now - testStartTime >= TEST_DURATION_MS) {
      motorBrake(); // Phanh cả hai
      currentState = IDLE;
      motorsActivated = false; // Reset cờ cho lần chạy sau
      Serial.println("Test complete. Enter new RPM (0-170).");
      return; // Kết thúc
    }

    // --- Kiểm tra xem đã đến lúc ghi log (dt = 20ms) ---
    if (now - lastLogTime >= LOG_DT_MS) {
      float dt_sec = (now - lastLogTime) / 1000.0f;
      lastLogTime = now;

      // Lấy số đếm và tính toán cho Motor 1
      long cnt1 = encCount1;
      long dp1  = cnt1 - lastEnc1;
      lastEnc1  = cnt1;
      double rpm_meas1 = (dp1 / dt_sec) * 60.0 / CPR;

      // Lấy số đếm và tính toán cho Motor 2
      long cnt2 = encCount2;
      long dp2  = cnt2 - lastEnc2;
      lastEnc2  = cnt2;
      double rpm_meas2 = (dp2 / dt_sec) * 60.0 / CPR;

      // Tính thời gian (giây) kể từ lúc bắt đầu
      float time_elapsed = (now - testStartTime) / 1000.0f;

      // XUẤT DATA (định dạng "Time,RPM1,RPM2")
      Serial.print(time_elapsed, 3);
      Serial.print(",");
      Serial.print(rpm_meas1, 2);
      Serial.print(",");
      Serial.println(rpm_meas2, 2);
    }
  }
}