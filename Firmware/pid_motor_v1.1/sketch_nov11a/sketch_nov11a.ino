#include <Arduino.h>

// ===== PIN MAP =====
const int PIN_AIN1 = 26;
const int PIN_AIN2 = 27;
const int PIN_PWMA = 25;
const int PIN_STBY = 14;

const int PIN_ENC_A = 34;
const int PIN_ENC_B = 35;

// ===== PWM =====
const int PWM_FREQ = 20000;
const int PWM_RES  = 10;       // 10-bit (0..1023)
const int PWM_MAX  = (1<<PWM_RES) - 1; // 1023

// ===== ENCODER =====
const int CPR = 390;           // Xung trên mỗi vòng quay (11 PPR * 35.5)

// ===== CẤU HÌNH TEST (ĐÃ CẬP NHẬT) =====
const float LOG_DT_MS = 20;    // Ghi log mỗi 25ms (tần số 40Hz)
const long TEST_DURATION_MS = 1500; // Chạy test trong 5 giây
const long STEP_DELAY_MS = 500; // Bắt đầu step sau 500ms (0.5 giây)

// ===== Biến =====
volatile long encCount = 0;
long lastEnc = 0;
unsigned long testStartTime = 0;
unsigned long lastLogTime = 0;
bool stepApplied = false; // Biến mới: để theo dõi trạng thái step

enum State { IDLE, RUNNING };
State currentState = IDLE;

String inLine;
bool haveLine = false;

// ---------- ISR ----------
// Hàm ngắt (Interrupt Service Routine) để đọc Encoder
// Tăng/giảm biến đếm xung dựa trên chiều quay (đọc chân B)
void IRAM_ATTR encISR() {
  if (digitalRead(PIN_ENC_B)) encCount++;
  else                        encCount--;
}

// ---------- Điều khiển TB6612 ----------
static inline void motorStandby(bool en)      { digitalWrite(PIN_STBY, en ? HIGH : LOW); }
static inline void motorBrake()               { digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); ledcWrite(PIN_PWMA, 0); }

// Hàm motorDrive gốc (vẫn giữ để linh hoạt)
static inline void motorDrive(int signed_pwm) {
  int mag = constrain(abs(signed_pwm), 0, PWM_MAX);
  if      (signed_pwm > 0) { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);  ledcWrite(PIN_PWMA, mag); }
  else if (signed_pwm < 0) { digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); ledcWrite(PIN_PWMA, mag); }
  else                     { motorBrake(); }
}

// ---------- Serial Input ----------
void handleSerialInput() {
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
    inLine = ""; haveLine = false;

    if (currentState == IDLE) {
      Serial.println("Bắt đầu test PHẢN ỨNG BƯỚC (Trễ 0.5s, 0-100% PWM).");
      
      // Ghi log điểm T=0 (Time, RPM, %PWM)
      Serial.print(0.000, 3); Serial.print(",");
      Serial.print(0.00, 2);  Serial.print(",");
      Serial.println(0.0, 1);
      
      // Đặt lại các biến đếm
      encCount = 0;
      lastEnc = 0;
      testStartTime = millis();
      lastLogTime = testStartTime;
      currentState = RUNNING;
      stepApplied = false; // Đặt lại cờ
      
      // Kích hoạt động cơ và ĐẢM BẢO PHANH (để bắt đầu từ 0)
      motorStandby(true);
      motorBrake(); // Bắt đầu bằng 0
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  // LEDC setup
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);

  // Gắn hàm ngắt vào chân Encoder A
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, RISING);

  // Khởi động ở trạng thái phanh và standby
  motorStandby(true);
  motorBrake();

  Serial.println(F("Ready. Nhấn Enter để bắt đầu test Phản ứng Bước (Trễ 0.5s)."));
  Serial.println("Time(s),RPM_Actual,%PWM");
}

void loop() {
  handleSerialInput();

  if (currentState == RUNNING) {
    unsigned long now = millis();

    // --- Điều kiện Dừng Test (sau 5 giây) ---
    if (now - testStartTime >= TEST_DURATION_MS) {
      motorBrake();
      currentState = IDLE;
      Serial.println("Test complete. Nhấn Enter để bắt đầu lại.");
      return;
    }

    // --- [KHỐI LOGIC STEP MỚI] ---
    // Kích hoạt Step (bước) tại 0.5 giây (STEP_DELAY_MS)
    if (!stepApplied && (now - testStartTime >= STEP_DELAY_MS)) {
      motorDrive(PWM_MAX);
      stepApplied = true;
    }

    // --- Ghi log (Lấy mẫu mỗi 25ms) ---
    if (now - lastLogTime >= LOG_DT_MS) {
      // Tính toán dt (delta-time) thực tế để RPM chính xác
      float dt_sec = (now - lastLogTime) / 1000.0f;
      lastLogTime = now;

      // Đọc và đặt lại bộ đếm một cách an toàn
      long cnt = encCount;
      long dp  = cnt - lastEnc;
      lastEnc  = cnt;

      // Tính RPM: (xung/dt) * (60 giây/phút) / (xung/vòng)
      double rpm_meas = (dp / dt_sec) * 60.0 / CPR;
      float time_elapsed = (now - testStartTime) / 1000.0f;

      // Xuất dữ liệu CSV
      Serial.print(time_elapsed, 3); Serial.print(",");
      Serial.print(rpm_meas, 2);     Serial.print(",");
      
      // Ghi log %PWM chính xác (0% trước 0.5s, 100% sau 0.5s)
      float currentPwmPercent = stepApplied ? 100.0 : 0.0;
      Serial.println(currentPwmPercent, 1);
    }
  }
}