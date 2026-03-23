/*
 * ESP32 ROBOT – FINAL FULL VERSION (UPDATED LOGIC)
 * =================================================
 * - Sequence: SPECIAL CONDITIONS → LINE FOLLOW → SPEED PID → FEEDBACK
 * - Pickup Zone + cargo_type Logic fully rewritten and optimized.
 * Hung - 20/11
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// ==========================================================
// 1. HARDWARE
// ==========================================================

// Motor pins
const int PIN_AIN1 = 14, PIN_AIN2 = 27, PIN_PWMA = 25;
const int PIN_BIN1 = 13, PIN_BIN2 = 5, PIN_PWMB = 26;
const int PIN_STBY = 4;

const int PIN_ENC_A_1 = 16, PIN_ENC_B_1 = 17;
const int PIN_ENC_A_2 = 19, PIN_ENC_B_2 = 18;

// Line sensors
const int S1 = 36, S2 = 39, S3 = 34, S4 = 35, S5 = 32;

// Color sensor
Adafruit_TCS34725 tcs(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

// PWM + Encoder
const int PWM_FREQ = 20000;
const int PWM_RES = 10;
const float PWM_MAX = 1023.0;
const int CPR = 390;
const float DT_MS = 50;

// --- Measured RPM ---
double rpm_meas_1 = 0;
double rpm_meas_2 = 0;

// ==========================================================
// 2. PID PARAMETERS (RESTORED VALUES)
// ==========================================================

// Inner loop (Speed PI)
double Kp_1 = 0.19, Ki_1 = 11.0;
double Kp_2 = 0.20, Ki_2 = 11.0;

double integ_1 = 0, integ_2 = 0;

// Outer loop (Line PID)
double base_rpm = 155.0;

double Kp_line = 5.0;
double Ki_line = 0.75;
double Kd_line = 1.3;

double err_line = 0, last_err_line = 0, integ_line = 0;

// Sensors calibration
int rawMin[5] = {53, 43, 30, 33, 27};
int rawMax[5] = {3200, 2965, 2748, 2684, 2691};

//int rawMin[5] = {28, 35, 34, 43, 44};
//int rawMax[5] = {2600, 2328, 2138, 2753, 2821};

int weight[5]  = {-20, -10, 0, 10, 20};
int norm_sensor[5];

// Color calib
float redCalib[3]  = {47.8, 20.9, 14.9};
float blueCalib[3] = {38.0, 45.6, 38.6};
float unkCalib[3]  = {26.0, 24.0, 16.8};
const float COLOR_THRESHOLD = 150.0;

// Encoder
volatile long encCount1 = 0, encCount2 = 0;
long lastEnc1 = 0, lastEnc2 = 0;

unsigned long lastLoopTime = 0;

// ==========================================================
// 3. STATE MACHINE
// ==========================================================
enum RobotState {
  STATE_STOP,
  STATE_MOVING,
  STATE_LOADING,
  STATE_EXIT_PICKUP,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT
};

RobotState currentState = STATE_STOP;
unsigned long stateStart = 0;

// ==========================================================
// 4. cargo_type LOGIC VARIABLES
// ==========================================================
bool isHasCargo = false;  // Already scanned?
int cargo_type = 0;           // 0: none, 1: red, 2: blue

String inLine; bool haveLine = false;

// ==========================================================
// HELPERS
// ==========================================================
void IRAM_ATTR encISR1() {
  if (digitalRead(PIN_ENC_B_1)) encCount1++; else encCount1--;
}
void IRAM_ATTR encISR2() {
  if (digitalRead(PIN_ENC_B_2)) encCount2++; else encCount2--;
}

void motorDrive(int pin1, int pin2, int pwmPin, int val) {
  val = constrain(val, -1023, 1023);

  if (val > 0) {
    digitalWrite(pin1, HIGH); digitalWrite(pin2, LOW);
  } else if (val < 0) {
    digitalWrite(pin1, LOW); digitalWrite(pin2, HIGH);
  } else {
    digitalWrite(pin1, LOW); digitalWrite(pin2, LOW);
  }

  ledcWrite(pwmPin, abs(val));
}

void setMotors(int u1, int u2) {
  motorDrive(PIN_AIN1, PIN_AIN2, PIN_PWMA, u1);
  motorDrive(PIN_BIN1, PIN_BIN2, PIN_PWMB, u2);
}

int clampMap(int x, int in_min, int in_max, int out_min, int out_max) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  return (long)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readLineSensors() {
  int pins[5] = {S1, S2, S3, S4, S5};
  for (int i = 0; i < 5; i++)
    norm_sensor[i] = clampMap(analogRead(pins[i]), rawMin[i], rawMax[i], 0, 1000);
}

double getLineError() {
  long sumW = 0, sumV = 0;
  for (int i = 0; i < 5; i++) {
    sumW += norm_sensor[i] * weight[i];
    sumV += norm_sensor[i];
  }
  if (sumV < 300) return last_err_line;
  return (double)sumW / sumV;
}

float getDist(float r, float g, float b, float calib[3]) {
  return sqrt(pow(r - calib[0], 2) + pow(g - calib[1], 2) + pow(b - calib[2], 2));
}

int detectColorRaw() {
  
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Convert to float (giống print_color)
  float fR = r;
  float fG = g;
  float fB = b;

  // Tính khoảng cách
  float dRed  = getDist(fR, fG, fB, redCalib);
  float dBlue = getDist(fR, fG, fB, blueCalib);
  float dUnk  = getDist(fR, fG, fB, unkCalib);

  // Chọn min
  float minDist = dRed;
  int color = 1;   // RED

  if (dBlue < minDist) {
    minDist = dBlue;
    color = 2;     // BLUE
  }
  if (dUnk < minDist) {
    minDist = dUnk;
    color = 0;     // UNKNOWN
  }

  return color;
}


void resetPID() {
  integ_1 = integ_2 = 0;
  integ_line = 0;
  last_err_line = 0;
}

// ==========================================================
// SETUP
// ==========================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(12);
  Wire.begin();

  if (!tcs.begin()) {
    Serial.println("Color Sensor FAIL");
    while (1);
  }
  tcs.setInterrupt(false); // Enable sensor

  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);

  pinMode(PIN_ENC_A_1, INPUT);
  pinMode(PIN_ENC_B_1, INPUT);
  pinMode(PIN_ENC_A_2, INPUT);
  pinMode(PIN_ENC_B_2, INPUT);

  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_1), encISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_2), encISR2, RISING);

  Serial.println("Press: g to go  || s to stop");
}

// ==========================================================
// MAIN LOOP
// ==========================================================
void loop() {

  // ============LOOP: 1. START/STATE_STOP=========
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inLine.length()) haveLine = true;
    } else inLine += c;
  }

  if (haveLine) {
    inLine.trim();
    if (inLine == "g") {currentState = STATE_MOVING; cargo_type = 0; isHasCargo = false;}
    else if (inLine == "s") { currentState = STATE_STOP; setMotors(0,0); }
    haveLine = false;
    inLine = "";
  }

  // ========== LOOP: 2. DATA =========
  unsigned long now = millis();
  if (now - lastLoopTime < DT_MS) return;

  float dt = (now - lastLoopTime) / 1000.0f;
  lastLoopTime = now;

  // Encoder
  long c1 = encCount1, c2 = encCount2;
  long dp1 = c1 - lastEnc1; lastEnc1 = c1;
  long dp2 = c2 - lastEnc2; lastEnc2 = c2;

  double rpm1 = (dp1 / dt) * 60.0 / CPR;
  double rpm2 = (dp2 / dt) * 60.0 / CPR;

  rpm_meas_1 = rpm1;
  rpm_meas_2 = rpm2;

  // Read line sensors
  readLineSensors();

  // Shortcut vars
  int s1 = norm_sensor[0];
  int s2 = norm_sensor[1];
  int s3 = norm_sensor[2];
  int s4 = norm_sensor[3];
  int s5 = norm_sensor[4];

  // =============LOOP: 3. SPECIAL CASE CHECK===========

  bool isPickupZone = ((s2 >= 400 && s3 >= 400 && s4 >= 400));
  bool iscargo_type = (s3 <= 300 && s1 >= 700 && s5 >= 700);
  bool isFinish = (s1<200 && s2<200 && s3<200 && s4<200 && s5<200); 

  //A. Check finish
  if (currentState == STATE_MOVING && isFinish) {
       setMotors(-20,-20);
       currentState = STATE_STOP; 
       Serial.println("STATE_STOP SHIPPING");
       return;
  }

  //B. Check Pickup Zone
  if (currentState == STATE_MOVING && !isHasCargo && isPickupZone) {
    setMotors(-20,-20);
    resetPID();
    Serial.println("Arrive Pickup_Zone !!!");

    delay(2000);

    Serial.println("START LOADING...");

    currentState = STATE_LOADING;
    stateStart = now;
    return;
  }

  //C. Check Intersection
  if (currentState == STATE_MOVING && iscargo_type) {
    
    setMotors(0,0);
    resetPID();

    if (!isHasCargo) {
      Serial.println(" HADN'T LOADING → STATE_STOP");
      currentState = STATE_STOP;
      return;
    }

    if (cargo_type == 1) {
      Serial.println("==> RED → LEFT");
      currentState = STATE_TURN_LEFT;
    } else if (cargo_type == 2) {
      Serial.println("==> BLUE → RIGHT");
      currentState = STATE_TURN_RIGHT;
    }

    stateStart = now;
    return;
  }

  // =============LOOP: 4.ROBOT STATE=========
  switch (currentState) {

    //a. STATE_STOP
    case STATE_STOP:
      setMotors(-20,-20);
      break;

    //
    case STATE_LOADING: {
      setMotors(-20, -20);

      static int lastColor = 0;        // màu ở chu kỳ trước
      static unsigned long colorStart = 0;  // thời điểm bắt đầu giữ 1 màu
      static bool waitingMsgShown = false;

      int cargo_color = detectColorRaw();   // 0 = UNKNOWN, 1 = RED, 2 = BLUE

      if (!waitingMsgShown) {
        Serial.println("WAITING FOR LOADING...");
        waitingMsgShown = true;
      }

      // Nếu UNKNOWN → không tính, reset và tiếp tục chờ
      if (cargo_color == 0) {
        lastColor = 0;
        colorStart = 0;
        Serial.println("WAITING FOR LOADING...");
        break;
      }

      // Nếu màu thay đổi → reset timer
      if (cargo_color != lastColor) {
        lastColor = cargo_color;
        colorStart = now;  // bắt đầu đếm lại
        // Debug:
        if (cargo_color == 1) Serial.println("RED → FINISH LOADING IN 3S...");
        if (cargo_color == 2) Serial.println("BLUE → FINISH LOADING IN 3S...");
      }

      // Kiểm tra giữ màu đủ 3s
      if (colorStart > 0 && (now - colorStart >= 3000)) {

        cargo_type = cargo_color;
        isHasCargo = true;

        if (cargo_color == 1) Serial.println("RECEIVED - RED CARGO");
        else if (cargo_color == 2) Serial.println("RECEIVED - BLUE CARGO");

        // Reset biến
        lastColor = 0;
        colorStart = 0;
        waitingMsgShown = false;

        delay(500);
        currentState = STATE_EXIT_PICKUP;
        stateStart = now;
      }

    }
    break;


    // --------------------------------------------------------
    case STATE_EXIT_PICKUP:
      setMotors(250, 250);
      if (now - stateStart >= 400) {
        currentState = STATE_MOVING;
        resetPID();
      }
      break;

    // --------------------------------------------------------
    case STATE_TURN_LEFT:
      setMotors(-250, 250);
      if (now - stateStart >= 100) {
        setMotors(50,50);
        currentState = STATE_MOVING;
        resetPID();
      }
      break;

    // --------------------------------------------------------
    case STATE_TURN_RIGHT:
      setMotors(150, -150);
      if (now - stateStart >= 100) {
        setMotors(50,50);
        currentState = STATE_MOVING;
        resetPID();
      }
      break;

    // --------------------------------------------------------
    case STATE_MOVING: {
      // LINE PID
      err_line = getLineError();

      double P = Kp_line * err_line;
      integ_line += err_line * dt;
      integ_line = constrain(integ_line, -50, 50);
      double I = Ki_line * integ_line;
      double D = Kd_line * (err_line - last_err_line) / dt;
      last_err_line = err_line;

      double steer = P + I + D;

      double set1 = base_rpm + steer;
      double set2 = base_rpm - steer;

      // SPEED PID
      integ_1 += (set1 - rpm_meas_1) * Ki_1 * dt;
      integ_1 = constrain(integ_1, -1023, 1023);
      double u1 = Kp_1*(set1 - rpm_meas_1) + integ_1;

      integ_2 += (set2 - rpm_meas_2) * Ki_2 * dt;
      integ_2 = constrain(integ_2, -1023, 1023);
      double u2 = Kp_2*(set2 - rpm_meas_2) + integ_2;

      setMotors((int)u1, (int)u2);
    }
    break;
  }


  // ================================================================
  // 4. FEEDBACK / DEBUG
  // ================================================================
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 1000) {
    lastPrint = now;

    Serial.print(" | Err: ");   Serial.print(err_line, 2);
    Serial.print(" | STATUS "); Serial.print(currentState); 
    Serial.printf("| S:%d %d %d %d %d | inter:%d | rpm:%.0f %.0f\n",
      s1, s2, s3, s4, s5,
      cargo_type,
      rpm_meas_1, rpm_meas_2

    );
  }
}
