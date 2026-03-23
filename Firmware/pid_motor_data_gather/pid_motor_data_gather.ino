/*
 * ESP32 + TB6612 - LẤY DỮ LIỆU ĐÁP ỨNG ĐỘNG CƠ (OPEN-LOOP)
 * ***** Tương thích ESP32 CORE 3.x (API MỚI) *****
 *
 * 1. Mở Serial Monitor (115200).
 * 2. Nhập một RPM mục tiêu (ví dụ: 170) rồi Enter.
 * 3. Code sẽ map 170 RPM -> 1023 PWM (tối đa).
 * 4. Code sẽ cấp 1023 PWM và ghi log (Time, RPM) trong 3 giây.
 * 5. Mở Serial Plotter để xem biểu đồ, hoặc copy/paste dữ liệu sang Excel.
 */

#include <Arduino.h>

// ===== PIN MAP (theo sơ đồ của bạn) =====
const int PIN_AIN1 = 14;
const int PIN_AIN2 = 27;

const int PIN_PWMA = 25;
const int PIN_STBY = 4;

const int PIN_ENC_A = 16;  // input-only, không có pull-up nội
const int PIN_ENC_B = 17;  // input-only, không có pull-up nội

// ===== PWM (LEDC) =====
const int PWM_FREQ = 20000;  // 20 kHz êm tiếng
const int PWM_RES  = 10;     // 10-bit (0..1023)
const double PWM_MAX = (1<<PWM_RES) - 1; // 1023

// ===== ENCODER / TỐC ĐỘ =====
const int   CPR = 390;             // Tỉ số xung (11 * 35.5)
const float MAX_RPM = 170.0;       // RPM không tải (để map)
const float LOG_DT_MS = 20.0;      // dt = 0.02s
const long  TEST_DURATION_MS = 3000; // Chạy test trong 3 giây

// ===== Biến encoder =====
volatile long encCount = 0;
long lastEnc = 0;
unsigned long testStartTime = 0;
unsigned long lastLogTime = 0;

// ===== Trạng thái hệ thống =====
enum State { IDLE, RUNNING };
State currentState = IDLE;

// ===== Nhập lệnh Serial =====
String inLine; bool haveLine = false;

// ---------- ISR: cạnh LÊN kênh A, đọc B để xác định hướng ----------
void IRAM_ATTR encISR() {
  if (digitalRead(PIN_ENC_B)) encCount++;
  else                        encCount--;
}

// ---------- Điều khiển TB6612 ----------
static inline void motorStandby(bool en)       { digitalWrite(PIN_STBY, en ? HIGH : LOW); }
static inline void motorBrake()                { digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); ledcWrite(PIN_PWMA, 0); }
static inline void motorDrive(int signed_pwm) {
  int mag = constrain(abs(signed_pwm), 0, (int)PWM_MAX);
  if      (signed_pwm > 0) { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);  ledcWrite(PIN_PWMA, mag); }
  else if (signed_pwm < 0) { digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); ledcWrite(PIN_PWMA, mag); }
  else                     { motorBrake(); }
}

// ---------- Xử lý Serial Input ----------
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
    double rpm_target = (double)inLine.toInt(); // Nhập RPM (0-170)
    inLine = ""; haveLine = false;
    
    // Chỉ bắt đầu test nếu đang ở trạng thái IDLE
    if (currentState == IDLE) {
      // Ánh xạ RPM mục tiêu sang PWM cố định
// (ví dụ: 170 RPM -> 1023 PWM)
      int pwm_to_apply = map(rpm_target, 0, MAX_RPM, 0, PWM_MAX);
      pwm_to_apply = constrain(pwm_to_apply, 0, (int)PWM_MAX);

      Serial.print("Bắt đầu test. Mục tiêu RPM: ");
      Serial.print(rpm_target);
      Serial.print(" -> Áp dụng PWM cố định: ");
      Serial.println(pwm_to_apply);
      Serial.println("Time(s),RPM_Actual"); // Header cho Excel

      // Reset và Bắt đầu
      encCount = 0;
      lastEnc = 0;
      testStartTime = millis();
      lastLogTime = testStartTime;
      currentState = RUNNING;
      motorDrive(pwm_to_apply); // Cấp PWM cố định
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

  // API 3.x: Tự động setup và attach
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, RISING);

  motorStandby(true);
  motorBrake();

  Serial.println(F("Ready (Open-Loop Data Logger). Type target RPM (0-170)."));
}

void loop() {
  // 1. Luôn kiểm tra Serial
  handleSerialInput();

  // 2. Nếu đang trong trạng thái test
  if (currentState == RUNNING) {
    unsigned long now = millis();

    // --- Kiểm tra xem đã hết 3 giây chưa ---
    if (now - testStartTime >= TEST_DURATION_MS) {
      motorBrake();
      currentState = IDLE;
      Serial.println("Test complete. Enter new RPM (0-170).");
      return; // Kết thúc
    }

    // --- Kiểm tra xem đã đến lúc ghi log (dt = 20ms) ---
    if (now - lastLogTime >= LOG_DT_MS) {
      float dt_sec = (now - lastLogTime) / 1000.0f;
      lastLogTime = now;

      long cnt = encCount;
      long dp  = cnt - lastEnc;
      lastEnc  = cnt;

      // Tính RPM thực tế (đo được trong 20ms vừa qua)
      double rpm_meas = (dp / dt_sec) * 60.0 / CPR;

      // Tính thời gian (giây) kể từ lúc bắt đầu
      float time_elapsed = (now - testStartTime) / 1000.0f;

      // XUẤT DATA (định dạng "Time,RPM" để copy sang Excel)
      Serial.print(time_elapsed, 3);
      Serial.print(",");
      Serial.println(rpm_meas, 2);
    }
  }
}