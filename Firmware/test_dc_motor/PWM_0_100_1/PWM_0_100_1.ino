#include <Arduino.h>

// === CẤU HÌNH CHÂN ===
#define AIN1  26
#define AIN2  27
#define PWMA  25
#define STBY  14
#define ENCA  34
#define ENCB  35

// === PWM ===
#define PWM_CHANNEL 0
#define PWM_FREQ    20000
#define PWM_RES     8

// === THÔNG SỐ ĐỘNG CƠ ===
#define PPR             390     // 11 × 35.5 = 390 xung/vòng (trục ra)
#define SAMPLE_TIME_MS  250     // thời gian lấy mẫu RPM
#define STEP_INTERVAL_MS 250    // thời gian tăng PWM mỗi bước
#define TOTAL_STEPS     101     // PWM 0 → 100%

// Biến toàn cục
volatile long encoderCount = 0;
long lastEncoderCount = 0;

unsigned long lastSampleTime = 0;
unsigned long lastStepTime = 0;
int currentPWM = 0;
int stepCount = 0;

//PID
float kp = 81;
float ki = 275;
float err = 0;
float err_delta = 0;
float pwm_target = 0;
float P = 0;
float I = 0;
float u = 0;

// --- Biến đo tốc độ ---
long currentCount = 0;
long deltaPulses = 0;
float dt = 0;
float pulsesPerSecond = 0;
float rpm = 0;
float timeSec = 0;

void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCA) == digitalRead(ENCB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR, RISING);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_CHANNEL);

  Serial.begin(115200);
  delay(2000);

  Serial.println("=== ĐO TỐC ĐỘ ĐỘNG CƠ - PWM tăng mỗi 0.25s ===");
  Serial.println("time(s)\tPWM(%)\tPulses\tRPM (trục ra)");
  Serial.println("-------------------------------------------");

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  encoderCount = 0;
  lastEncoderCount = 0;
  lastSampleTime = millis();
  lastStepTime = millis();
  currentPWM = 0;
  stepCount = 0;
}

void loop() {
  unsigned long now = millis();

  // === LẤY MẪU RPM ===
  if (now - lastSampleTime >= SAMPLE_TIME_MS) {
    currentCount = encoderCount;
    deltaPulses = abs(currentCount - lastEncoderCount);
    lastEncoderCount = currentCount;

    dt = SAMPLE_TIME_MS / 1000.0;
    pulsesPerSecond = deltaPulses / dt;
    rpm = (pulsesPerSecond * 60.0) / PPR;
    timeSec = now / 1000.0;

    Serial.print(timeSec, 2);
    Serial.print("\t");
    Serial.print(currentPWM);
    Serial.print("\t");
    Serial.print(deltaPulses);
    Serial.print("\t");
    Serial.println(rpm, 2);

    lastSampleTime = now;
  }

  // === TĂNG PWM MỖI 0.25s ===
  if (now - lastStepTime >= STEP_INTERVAL_MS && stepCount < TOTAL_STEPS) {
    currentPWM = stepCount;  // 0 → 100
    int pwmValue = map(currentPWM, 0, 100, 0, 255);

    rpm_target = 170*0.01*currentPWM;
    err = rpm_target - rpm;
    int last_err = 0;
    err_delta = err - last_err;
    P = err * kp;
    I += err_delta * ki * dt;
    int control = P + I;

    if (control > 255) control = 255;
    if (control < 0) control = 0;

    ledcWrite(PWM_CHANNEL, control);

    stepCount++;
    lastStepTime = now;
    last_err = err;
  }

  // === SAU KHI PWM = 100%, GIỮ 5 GIÂY RỒI DỪNG ===
  if (stepCount >= TOTAL_STEPS && currentPWM >= 100) {
    static bool finished = false;
    static unsigned long finishStart = 0;

    if (!finished) {
      finished = true;
      finishStart = millis();
      Serial.println("=== PWM đạt 100% - giữ thêm 2 giây ===");
    }

    if (millis() - finishStart >= 2000) {
      ledcWrite(PWM_CHANNEL, 0);
Serial.println("=== HOÀN TẤT ===");
      while (1) delay(1000);
    }
  }
}
