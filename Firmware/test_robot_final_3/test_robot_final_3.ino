/*
 * ESP32 ROBOT – CLEANED AND STATUS-TRACKED VERSION (FIXED)
 * ==================================================
 * - Logic: Line Following + Pickup Zone + Intersection + End Point
 * - Updated Status Names and Tracking Variables
 * - RPM, Line Sensors, and Cargo Color printed
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
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// PWM + Encoder
const int CPR = 390;
const float DT_MS = 50;

// --- Measured RPM ---
double rpm_meas_1 = 0;
double rpm_meas_2 = 0;
double rpm_left = 0;  // Added declaration
double rpm_right = 0; // Added declaration

unsigned long startTime = 0; // Renamed from timer for clarity

// ==========================================================
// 2. PID PARAMETERS
// ==========================================================

// Inner loop (Speed PI)
double Kp_1 = 0.19, Ki_1 = 11.0;
double Kp_2 = 0.20, Ki_2 = 11.0;
double integ_1 = 0, integ_2 = 0;

// Outer loop (Line PID)
double base_rpm = 140.0;
double Kp_line = 5.0;
double Ki_line = 0.75;
double Kd_line = 1.3;

double err_line = 0, last_err_line = 0, integ_line = 0, biggest_err = 0;

// Sensors calibration
int rawMin[5] = {53, 43, 30, 33, 27};
int rawMax[5] = {3200, 2965, 2748, 2684, 2691};
int weight[5] = {-20, -10, 0, 10, 20};
int norm_sensor[5];

// Color calib
float redCalib[3]  = {47.8, 20.9, 14.9};
float blueCalib[3] = {38.0, 45.6, 38.6};
float unkCalib[3]  = {26.0, 24.0, 16.8};

// Encoder
volatile long encCount1 = 0, encCount2 = 0;
long lastEnc1 = 0, lastEnc2 = 0;
unsigned long lastLoopTime = 0;

// ===================== STATE MACHINE =====================
enum RobotState {
  MOVING,
  STOP,
  WAIT_LOADING,
  INTERSECTION_DECISION,
  FINISH_SHIPPING
};

// Tracking variable for Debugging
String trackingStatus = "Wait_to_start"; 

RobotState currentState = STOP;
unsigned long stateStart = 0; 
int cargoColor = 0; // 0=UNKNOWN, 1=RED, 2=BLUE
bool isPickupZone = false;
bool isIntersection = false;
bool isFinish = false; 
String inLine; 
bool haveLine = false;

// ===================== HELPERS =====================
void IRAM_ATTR encISR1() { if(digitalRead(PIN_ENC_B_1)) encCount1++; else encCount1--; }
void IRAM_ATTR encISR2() { if(digitalRead(PIN_ENC_B_2)) encCount2++; else encCount2--; }

void motorDrive(int pin1, int pin2, int pwmPin, int val){
  val = constrain(val, -1023, 1023);
  if(val > 0){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
  else if(val < 0){
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  else{
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
  ledcWrite(pwmPin, abs(val));
}

void setMotors(int left, int right){
  motorDrive(PIN_AIN1, PIN_AIN2, PIN_PWMA, left);
  motorDrive(PIN_BIN1, PIN_BIN2, PIN_PWMB, right);
}

int clampMap(int x, int in_min, int in_max, int out_min, int out_max){
  if(x <= in_min) return out_min;
  if(x >= in_max) return out_max;
  return (long)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readLineSensors(){
  int pins[5] = {S1, S2, S3, S4, S5};
  for(int i = 0; i < 5; i++) 
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

int detectColorRaw(){
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  float fR = r, fG = g, fB = b;
  float dR = getDist(fR, fG, fB, redCalib);
  float dB = getDist(fR, fG, fB, blueCalib);
  float dU = getDist(fR, fG, fB, unkCalib);
  
  float minD = dR; 
  int color = 1; // Default assume RED if closest
  if(dB < minD){ minD = dB; color = 2; }
  if(dU < minD) color = 0; // Unknown
  return color;
}

void resetPID() {
  integ_1 = integ_2 = 0;
  integ_line = 0;
  last_err_line = 0;
}

// ===================== SETUP =====================
void setup(){
  Serial.begin(115200); delay(200);
  analogReadResolution(12);
  Wire.begin();
  if(!tcs.begin()){ Serial.println("Color Sensor FAIL"); while(1); }
  tcs.setInterrupt(false);

  pinMode(PIN_STBY, OUTPUT); 
  digitalWrite(PIN_STBY, HIGH);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_ENC_A_1, INPUT); pinMode(PIN_ENC_B_1, INPUT);
  pinMode(PIN_ENC_A_2, INPUT); pinMode(PIN_ENC_B_2, INPUT);

  ledcAttach(PIN_PWMA, 5000, 10); // Note: Corrected frequency/syntax
  ledcAttach(PIN_PWMB, 5000, 10);
  
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_1), encISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_2), encISR2, RISING);

  // Fixed quotes
  Serial.println("Press: 'g' to go || 's' to stop");
  startTime = millis();
}

// ===================== LOOP =====================
void loop(){
  //====================== START/STOP =================
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inLine.length()) haveLine = true;
    } else inLine += c;
  }

  if (haveLine) {
    inLine.trim();
    if (inLine == "g") {
      currentState = MOVING; 
      trackingStatus = "Moving_to_loading"; 
      cargoColor = 0; 
      isIntersection = false;
      startTime = millis(); // Reset timer when start
      resetPID();
    }
    else if (inLine == "s") {
      currentState = STOP; 
      trackingStatus = "Stopped_by_user";
      haveLine = false;
      inLine = "";
    }
    haveLine = false; // Reset flag
    inLine = "";
  }

  //================= DATA ANALYSIS =========
  unsigned long now = millis();
  if(now - lastLoopTime < DT_MS) return;
  float dt = (now - lastLoopTime) / 1000.0f; 
  lastLoopTime = now;

  long c1 = encCount1, c2 = encCount2;
  long dp1 = c1 - lastEnc1; lastEnc1 = c1;
  long dp2 = c2 - lastEnc2; lastEnc2 = c2;
  
  // Update global RPM variables
  rpm_left = (dp1 / dt) * 60.0 / CPR;
  rpm_right = (dp2 / dt) * 60.0 / CPR;
  
  // Update PID inputs
  rpm_meas_1 = rpm_left;
  rpm_meas_2 = rpm_right;

  readLineSensors();
  int s1 = norm_sensor[0], s2 = norm_sensor[1], s3 = norm_sensor[2], s4 = norm_sensor[3], s5 = norm_sensor[4];

  // ============= LOOP: 3. SPECIAL CASE CHECK ===========
  isPickupZone = (s2 >= 400 && s3 >= 400 && s4 >= 400);
  isIntersection = (s3 <= 300 && s1 >= 700 && s5 >= 700);
  isFinish = (s1 < 200 && s2 < 200 && s3 < 200 && s4 < 200 && s5 < 200); 

  // A. Check finish
  if (currentState == MOVING && isFinish) {
    setMotors(-20, -20);
    resetPID();
    currentState = FINISH_SHIPPING;
    trackingStatus = "Finish_shipping";
    return;
  }

  // B. Check Pickup Zone
  // FIXED: cargoColor == 0 (compare), not = (assign)
  if (currentState == MOVING && cargoColor == 0 && isPickupZone) {
    setMotors(-20, -20);
    resetPID();
    Serial.println("Arrive Pickup_Zone !!!");
    currentState = WAIT_LOADING;
    trackingStatus = "Wait_to_loading";
    stateStart = now;
    return;
  }

  // C. Check Intersection
  if (currentState == MOVING && isIntersection && cargoColor != 0) {
    currentState = INTERSECTION_DECISION;
    trackingStatus = "Moving_to_end_point";
    stateStart = now;
    return;
  }

  // ========== STATUS LOGIC ==========
  switch(currentState){

    case MOVING: {
      // LINE PID
      err_line = getLineError();
      if(abs(err_line) >= abs(biggest_err)) biggest_err = err_line;

      double P = Kp_line * err_line;
      integ_line += err_line * dt;
      integ_line = constrain(integ_line, -50, 50);
      double I = Ki_line * integ_line;
      double D = Kd_line * (err_line - last_err_line) / dt;
      last_err_line = err_line;

      double steer = P + I + D;
      double set1 = base_rpm + steer;
      double set2 = base_rpm - steer;

      // SPEED PID MOTOR 1
      integ_1 += (set1 - rpm_meas_1) * Ki_1 * dt;
      integ_1 = constrain(integ_1, -1023, 1023);
      double u1 = Kp_1 * (set1 - rpm_meas_1) + integ_1;

      // SPEED PID MOTOR 2
      integ_2 += (set2 - rpm_meas_2) * Ki_2 * dt;
      integ_2 = constrain(integ_2, -1023, 1023);
      double u2 = Kp_2 * (set2 - rpm_meas_2) + integ_2;

      setMotors((int)u1, (int)u2);
      break; // Correctly breaks out of switch
    }

    case STOP: 
      setMotors(0, 0);
      break;

    case WAIT_LOADING: {
      setMotors(0, 0);
      static int lastColor = 0;       
      static unsigned long colorStart = 0; 

      int cargo_color = detectColorRaw();   // 0=UNKNOWN, 1=RED, 2=BLUE

      if (cargo_color == 0) {
        lastColor = 0;
        colorStart = 0;
        // Serial.println("WAITING..."); // Reduce spam
        break;
      }

      if (cargo_color != lastColor) {
        lastColor = cargo_color;
        colorStart = now; 
        if (cargo_color == 1) Serial.println("Detecting RED...");
        if (cargo_color == 2) Serial.println("Detecting BLUE...");
      }

      // Check for 3 seconds stability
      if (colorStart > 0 && (now - colorStart >= 3000)) {
        cargoColor = cargo_color;
        if (cargo_color == 1) Serial.println("RECEIVED - RED CARGO");
        else if (cargo_color == 2) Serial.println("RECEIVED - BLUE CARGO");
        
        lastColor = 0;
        colorStart = 0;
        delay(1000); // Small delay before moving
        currentState = MOVING;
        trackingStatus = "Moving_to_intersection";
        stateStart = now;
      }
      break;
    }

    case INTERSECTION_DECISION: 
      if (cargoColor == 0) {
        Serial.println(" ERROR: NO CARGO -> STOP");
        currentState = STOP;
        break;
      }
      if (cargoColor == 1) { // RED -> LEFT
        setMotors(-250, 250); // Spin left
        Serial.println("==> RED -> TURN LEFT"); // Fixed missing semicolon
        if (now - stateStart >= 350) { // Increased duration for better turn
          resetPID();
          currentState = MOVING;
        }
        break;
      } 
      if (cargoColor == 2) { // BLUE -> RIGHT
        setMotors(250, -250); // Spin right
        Serial.println("==> BLUE -> TURN RIGHT"); // Fixed missing semicolon
        if (now - stateStart >= 350) { // Increased duration
          resetPID();
          currentState = MOVING;
        }
        break;
      } 
      break;
      
    case FINISH_SHIPPING: 
      setMotors(0, 0); // Fixed function name setMotors
      Serial.println("FINISH SHIPPING"); 
      Serial.println("=========RESULT========"); 
      Serial.print("Max Err: "); Serial.println(biggest_err);
      Serial.print("Time (s): "); Serial.println((millis() - startTime) / 1000.0);
      currentState = STOP; // Stay stopped
      break;
      
  } // END SWITCH

  // ====== SERIAL DEBUG ======
  Serial.print("St: "); Serial.print(trackingStatus); 
  Serial.print(" | Sensors: ");
  Serial.print(s1); Serial.print(" "); Serial.print(s2); Serial.print(" ");
  Serial.print(s3); Serial.print(" "); Serial.print(s4); Serial.print(" ");
  Serial.print(s5);
  Serial.print(" | Color: "); Serial.print(cargoColor);
  Serial.print(" | RPM: "); Serial.print((int)rpm_left); Serial.print("/"); Serial.println((int)rpm_right);
}