/*
 * PID_data_verify – Speed PI Control Test
 * Motor targets:
 *   Motor 1 = 120 rpm
 *   Motor 2 = 100 rpm
 * Sample time = 0.025s
 * Log format: Time(s),RPM1,RPM2,PWM1,PWM2
 */

#include <Arduino.h>

// ===== PIN MAP =====
const int PIN_AIN1 = 14;
const int PIN_AIN2 = 27;
const int PIN_PWMA = 25;

const int PIN_BIN1 = 13;
const int PIN_BIN2 = 5;
const int PIN_PWMB = 26;

const int PIN_ENC_A_1 = 16;
const int PIN_ENC_B_1 = 17;
const int PIN_ENC_A_2 = 19;
const int PIN_ENC_B_2 = 18;

const int PIN_STBY = 4;

// ===== PWM CONFIG =====
const int PWM_FREQ = 20000;
const int PWM_RES  = 10;
const int PWM_MAX  = (1 << PWM_RES) - 1;

// ===== ENCODER =====
const int CPR = 390;
volatile long encCount1 = 0;
volatile long encCount2 = 0;
long lastEnc1 = 0, lastEnc2 = 0;

// ===== PID (PI) GAINS =====
double Kp_1 = 0.19, Ki_1 = 11.0;
double Kp_2 = 0.20, Ki_2 = 11.0;

double integ_1 = 0;
double integ_2 = 0;

// ===== CONTROL TARGET =====
const double TARGET_1 = 120.0;
const double TARGET_2 = 100.0;

// ===== SAMPLING =====
const float dt = 0.025;        // 0.025s = 40Hz
unsigned long lastTick = 0;
float T = 0.0;

// ===== TEST DURATION =====
const float TEST_TIME = 8.0;   // 8 giây kiểm thử
bool testFinished = false;

// ===== ISR =====
void IRAM_ATTR encISR1() {
  encCount1 += (digitalRead(PIN_ENC_B_1) ? 1 : -1);
}
void IRAM_ATTR encISR2() {
  encCount2 += (digitalRead(PIN_ENC_B_2) ? 1 : -1);
}

// ===== MOTOR CONTROL =====
void motor1Drive(int pwm) {
  int mag = constrain(abs(pwm), 0, PWM_MAX);

  if (pwm > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else if (pwm < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    mag = 0;
  }
  ledcWrite(PIN_PWMA, mag);
}

void motor2Drive(int pwm) {
  int mag = constrain(abs(pwm), 0, PWM_MAX);

  if (pwm > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
  } else if (pwm < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    mag = 0;
  }
  ledcWrite(PIN_PWMB, mag);
}

void motorBrake() {
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);
  ledcWrite(PIN_PWMA, 0);
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW);
  ledcWrite(PIN_PWMB, 0);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Standby
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  // Motor pins
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  pinMode(PIN_ENC_A_1, INPUT);
  pinMode(PIN_ENC_B_1, INPUT);
  pinMode(PIN_ENC_A_2, INPUT);
  pinMode(PIN_ENC_B_2, INPUT);

  // PWM
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);

  // Encoder interrupts
  attachInterrupt(PIN_ENC_A_1, encISR1, RISING);
  attachInterrupt(PIN_ENC_A_2, encISR2, RISING);

  Serial.println("Time(s),RPM1,RPM2,PWM1,PWM2");

  lastTick = millis();
}

void loop() {
  unsigned long now = millis();
  if (testFinished) return;

  // ===== Sampling exactly every dt (25ms) =====
  if (now - lastTick >= 25) {
    lastTick += 25;
    T += dt;

    // ===== Compute RPMs =====
    long c1 = encCount1;
    long dp1 = c1 - lastEnc1;
    lastEnc1 = c1;
    double rpm1 = (dp1 / dt) * 60.0 / CPR;

    long c2 = encCount2;
    long dp2 = c2 - lastEnc2;
    lastEnc2 = c2;
    double rpm2 = (dp2 / dt) * 60.0 / CPR;

    // ===== PI Controller Motor 1 =====
    double e1 = TARGET_1 - rpm1;
    integ_1 += e1 * dt;
    double u1 = Kp_1 * e1 + Ki_1 * integ_1;
    u1 = constrain(u1, -PWM_MAX, PWM_MAX);

    // ===== PI Controller Motor 2 =====
    double e2 = TARGET_2 - rpm2;
    integ_2 += e2 * dt;
    double u2 = Kp_2 * e2 + Ki_2 * integ_2;
    u2 = constrain(u2, -PWM_MAX, PWM_MAX);

    // ===== Apply PWM =====
    motor1Drive((int)u1);
    motor2Drive((int)u2);

    // ===== Log data =====
    Serial.printf("%.3f,%.2f,%.2f,%d,%d\n",
                  T, rpm1, rpm2, (int)u1, (int)u2);

    // ===== End test =====
    if (T >= TEST_TIME) {
      motorBrake();
      Serial.println("=== PID TEST COMPLETE ===");
      testFinished = true;
    }
  }
}
