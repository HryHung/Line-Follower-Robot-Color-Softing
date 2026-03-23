/*
 * ESP32 + TB6612 + Quadrature Encoder (A=34, B=35)
 * PI điều khiển tốc độ (RPM) cho 1 động cơ - ĐÃ SỬA LỖI LOGIC
 * ***** Tương thích ESP32 CORE 3.x (API MỚI) *****
 * Gõ một số RPM trên Serial Monitor (115200) rồi Enter, ví dụ: 150 hoặc -120
 */

#include <Arduino.h>

// ===== PIN MAP (theo sơ đồ của bạn) =====
const int PIN_AIN1 = 26;
const int PIN_AIN2 = 27;
const int PIN_PWMA = 25;
const int PIN_STBY = 14;

const int PIN_ENC_A = 34;  // input-only, không có pull-up nội
const int PIN_ENC_B = 35;  // input-only, không có pull-up nội

// ===== PWM (LEDC) =====
const int PWM_FREQ = 20000;  // 20 kHz êm tiếng
const int PWM_RES  = 10;     // 10-bit (0..1023)

// ===== ENCODER / TỐC ĐỘ =====
// Đang đếm x1: chỉ RISING của A + đọc B xác định hướng.
// Nếu encoder 11 PPR và tỉ số truyền 35.5 => CPR = 11 * 35.5 = 390.
const int   CPR = 390;             // chỉnh theo encoder của bạn
const float SPEED_DT_MS = 100.0;   // chu kỳ tính tốc độ (ms)

// ===== PI controller (tự viết, không cần thư viện) =====
double rpm_meas = 0.0;   // RPM đo (có dấu)
double rpm_set  = 0.0;   // RPM đặt (có dấu)

// Tham số PI (bạn tune dần)
// Kp, Ki này là các giá trị bạn phải tune (chỉnh định)
// Chúng KHÔNG phải là Kp, Ki bạn tính từ hàm truyền G(s)
double Kp = 0.2233;
double Ki = 14.4; // Thử giảm Ki so với code cũ (30)
double integ = 0.0; // Tích phân
const  double PWM_MAX = (1<<PWM_RES) - 1; // 1023

// ===== Biến encoder =====
volatile long encCount = 0;
long lastEnc = 0;
unsigned long lastSpeedMs = 0;

// ===== Nhập lệnh Serial =====
String inLine; bool haveLine = false;

// (Không cần khai báo prototype cho các hàm inline)

// ---------- ISR: cạnh LÊN kênh A, đọc B để xác định hướng ----------
void IRAM_ATTR encISR() {
  if (digitalRead(PIN_ENC_B)) encCount++;
  else                        encCount--;
}

// ---------- Điều khiển TB6612 ----------
static inline void motorStandby(bool en)       { digitalWrite(PIN_STBY, en ? HIGH : LOW); }
static inline void motorBrake()                { digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); ledcWrite(PIN_PWMA, 0); }
static inline void motorDrive(int signed_pwm) {
  // constrain() sẽ tự động giới hạn giá trị trong khoảng -1023 đến 1023
  int mag = constrain(abs(signed_pwm), 0, (int)PWM_MAX);
  
  if      (signed_pwm > 0) { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);  ledcWrite(PIN_PWMA, mag); }
  else if (signed_pwm < 0) { digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); ledcWrite(PIN_PWMA, mag); }
  else                     { motorBrake(); }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  // SỬA LỖI CORE 3.x: Dùng hàm ledcAttach() mới
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encISR, RISING);

  motorStandby(true);
  motorBrake();

  lastSpeedMs = millis();
  Serial.println(F("Ready (Core 3.x, PI Logic fixed). Type target RPM."));
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
    integ = 0.0; 
  }

  // ---- cập nhật tốc độ & điều khiển theo chu kỳ ----
  unsigned long now = millis();
  if (now - lastSpeedMs >= SPEED_DT_MS) {
    float dt = (now - lastSpeedMs) / 1000.0f;   // giây
    lastSpeedMs = now;

    long cnt = encCount;
    long dp  = cnt - lastEnc;
    lastEnc  = cnt;

    // RPM = (dp / dt) * 60 / CPR
    rpm_meas = (dp / dt) * 60.0 / CPR;

    // ==========================================================
    // === LOGIC PI ĐIỀU KHIỂN TỐC ĐỘ (ĐÃ SỬA) ===
    // ==========================================================
    
    // 1. Tính sai số (error) CÓ DẤU
    //    Ví dụ: Lệnh=150, Chạy=160 -> e = -10 (Cần phanh)
    double e = rpm_set - rpm_meas;

    // 2. Tính Tích phân (Integral)
    integ += e * Ki * dt;

    // 3. Chống tràn tích phân (Anti-windup)
    //    Cho phép 'integ' được âm (để phanh)
    if (integ > PWM_MAX) integ = PWM_MAX;
    if (integ < -PWM_MAX) integ = -PWM_MAX;

    // 4. Tính Tín hiệu Điều khiển (u)
    //    'u' bây giờ có thể âm (nếu e âm hoặc integ âm)
    double u = Kp * e + integ;

    // 5. Gửi lệnh PWM có dấu (âm/dương)
    //    Hàm motorDrive() sẽ tự động kẹp giá trị nếu 'u' > 1023
    motorDrive((int)u);

    // ==========================================================

    // In log để plot: set, meas, pwmSigned
    Serial.print((int)rpm_set); Serial.print(',');
    Serial.print(rpm_meas, 1);  Serial.print(',');
    Serial.println((int)u); // In ra tín hiệu 'u'
  }
}