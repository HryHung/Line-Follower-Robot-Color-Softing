/*
 * ESP32 ROBOT - SLAVE (UPDATED KINEMATIC OBSERVER)
 * ==================================================
 * - Logic cũ: GIỮ NGUYÊN
 * - Update: Hàm getErrorCenter dùng động học ngược + Lọc nhiễu
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <esp_now.h>
#include <WiFi.h>

// ===================== ESP-NOW =====================

uint8_t masterAddress[] = {0xFC, 0xE8, 0xC0, 0xDF, 0x73, 0x7C}; // <--- GIỮ NGUYÊN MAC CỦA BẠN

typedef struct struct_telemetry {
  int status;       
  int cargoColor;   
  float rpmLeft;
  float rpmRight;
  float errorLine;
  float errorCenter;   // Giá trị tâm xe mới
  int s1; int s2; int s3; int s4; int s5;
  float biggestError;
  unsigned long travelTime;
} struct_telemetry;

typedef struct struct_command {
  char cmd;
} struct_command;

struct_telemetry myData;
struct_command incomingCmd;

volatile bool newCommandReceived = false;
volatile char commandChar = ' ';

// ===================== HARDWARE =====================
const int PIN_AIN1 = 14, PIN_AIN2 = 27, PIN_PWMA = 25;
const int PIN_BIN1 = 13, PIN_BIN2 = 5, PIN_PWMB = 26;
const int PIN_STBY = 4;
const int PIN_ENC_A_1 = 16, PIN_ENC_B_1 = 17;
const int PIN_ENC_A_2 = 19, PIN_ENC_B_2 = 18;
const int S1 = 36, S2 = 39, S3 = 34, S4 = 35, S5 = 32;

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const int CPR = 390;
const float DT_MS = 20;

double rpm_meas_1 = 0, rpm_meas_2 = 0;
double rpm_left = 0, rpm_right = 0;

// ===================== PID =====================
double Kp_1 = 0.19, Ki_1 = 11.0;
double Kp_2 = 0.20, Ki_2 = 11.0;
double integ_1 = 0, integ_2 = 0;

double base_rpm_set = 167.0;
double base_rpm = 160;
double Kp_line = 6.0, Ki_line = 0.80, Kd_line = 1.3;
double err_line = 0, last_err_line = 0, integ_line = 0;

// ===================== THỐNG KÊ =====================
double current_biggest_err = 0;
unsigned long missionStartTime = 0;
unsigned long finalTravelTime = 0;

// ===================== CALIB SENSOR (GIỮ NGUYÊN) =====================
/*
int rawMin[5] = {53, 43, 30, 33, 27};
int rawMax[5] = {3200, 2965, 2748, 2684, 2691};
*/
int rawMin[5] = {67, 57, 40, 40, 28};
int rawMax[5] = {3063, 2964, 2871, 2721, 2650};
// Lưu ý: Weight này bạn nói là mm thực tế
int weight[5] = {-24, -12, 0, 12, 24}; 
int norm_sensor[5];
/*
float redCalib[3]  = {47.8, 20.9, 14.9};
float blueCalib[3] = {38.0, 45.6, 38.6};
float unkCalib[3]  = {26.0, 24.0, 16.8};
*/
/*
float redCalib[3]  = {74.2, 30.5, 23.0};
float blueCalib[3] = {56.7, 68.1, 59.2};
float unkCalib[3]  = {66.2, 57.9, 50.9};
*/
/*
float redCalib[3]  = {84.3, 37.0, 29.5};
float blueCalib[3] = {74.0, 90.0, 81.7};
float unkCalib[3]  = {445.1, 457.5, 455.3};
*/
/*
float redCalib[3]  = {169.2, 44.2, 35.1};
float blueCalib[3] = {67.9, 106.5, 128.7};
float unkCalib[3]  = {77.4, 71.3, 61.1};
*/
/*
float redCalib[3]  = {187.3, 47.7, 38.4};
float blueCalib[3] = {83.3, 136.8, 172.4};
float unkCalib[3]  = {156.9, 149.3, 130.7};
*/
float redCalib[3]  = {176.7, 46.2, 36.6};
float blueCalib[3] = {77.7, 127.3, 158.7};
float unkCalib[3]  = {34.7, 28.8, 22.7};

int cargo_color = 0;

volatile long encCount1 = 0, encCount2 = 0;
long lastEnc1 = 0, lastEnc2 = 0;
unsigned long lastLoopTime = 0;
unsigned long lastTeleTime = 0;

enum RobotState { MOVING, STOP, WAIT_LOADING, INTERSECTION_DECISION, FINISH_SHIPPING };
enum ShippingStatus { WAIT_TO_START, MOVING_TO_LOADING, WAIT_AT_LOADING, MOVING_TO_INTERSECTION, MOVING_TO_ENDPOINT, FINISHED_DELIVERY, STOPPED_BY_USER };

RobotState currentState = STOP;
ShippingStatus currentShipStatus = WAIT_TO_START;

unsigned long stateStart = 0; 
int cargoColor = 0; 
bool isPickupZone = false, isIntersection = false, isFinish = false;
// ===================== ESP-NOW CALLBACK =====================
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
  memcpy(&incomingCmd, incomingData, sizeof(incomingCmd));
  commandChar = incomingCmd.cmd;
  newCommandReceived = true;
}

// ===================== MOTOR HELPER =====================
void IRAM_ATTR encISR1() { if(digitalRead(PIN_ENC_B_1)) encCount1++; else encCount1--; }
void IRAM_ATTR encISR2() { if(digitalRead(PIN_ENC_B_2)) encCount2++; else encCount2--; }

void motorDrive(int pin1, int pin2, int pwmPin, int val){
  val = constrain(val, -1023, 1023);
  if(val > 0){ digitalWrite(pin1, HIGH); digitalWrite(pin2, LOW); }
  else if(val < 0){ digitalWrite(pin1, LOW); digitalWrite(pin2, HIGH); }
  else{ digitalWrite(pin1, LOW); digitalWrite(pin2, LOW); }
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

// ===================== LINE ERROR =====================
double getLineError() {
  long sumW = 0, sumV = 0;
  for (int i = 0; i < 5; i++) {
    sumW += norm_sensor[i] * weight[i];
    sumV += norm_sensor[i];
  }
  if (sumV < 300) return last_err_line;
  return (double)sumW / sumV;
}


// ===================== [NEW] ERR CENTER CALCULATION =====================
// Các thông số mới thêm vào
const float WHEEL_DIAMETER = 85.0; // mm
const float DIST_SENSOR_CENTER = 173.0; // mm
float last_err_center_filter = 0; // Biến lưu giá trị lọc

double getErrorCenter(double currentErr, double lastErr, float dt_sec, float avgRpm){
  double distMoved = (avgRpm / 60.0) * (WHEEL_DIAMETER * 3.14159) * dt_sec;
  if (distMoved < 1.0) return currentErr;
  double deltaErr = currentErr - lastErr;
  double tanTheta = deltaErr / distMoved;
  tanTheta = constrain(tanTheta, -1.0, 1.0);
  double raw_center_err = currentErr - (DIST_SENSOR_CENTER * tanTheta);
  float alpha = 0.2; //filter = 20%
  last_err_center_filter = (alpha * raw_center_err) + ((1.0 - alpha) * last_err_center_filter);
  return last_err_center_filter;
}

// ===================== COLOR =====================
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
  float minD = dR; int color = 1; 
  if(dB < minD){ minD = dB; color = 2; }
  if(dU < minD) color = 0; 
  return color;
}

void resetPID() {
  integ_1 = integ_2 = 0; integ_line = 0; last_err_line = 0;
  last_err_line = 0;
}

// ===================== TELEMETRY =====================
// Cần tham số dt để tính toán chính xác
void sendTelemetry(float dt_sec) {
  myData.status = (int)currentShipStatus;
  myData.cargoColor = cargoColor;
  myData.rpmLeft = (float)rpm_left;
  myData.rpmRight = (float)rpm_right;
  myData.errorLine = (float)err_line;
  
  // Gọi hàm tính toán mới
  float avgRpm = abs(rpm_left + rpm_right) / 2.0;
  myData.errorCenter = (float)getErrorCenter(err_line, last_err_line, dt_sec, avgRpm);
  
  myData.s1 = norm_sensor[0]; myData.s2 = norm_sensor[1]; 
  myData.s3 = norm_sensor[2]; myData.s4 = norm_sensor[3]; myData.s5 = norm_sensor[4];
  //myData.biggestError = (float)current_biggest_err;
  myData.travelTime = finalTravelTime; 

  esp_now_send(masterAddress, (uint8_t *) &myData, sizeof(myData));
}

// ===================== SETUP =====================
void setup(){
  Serial.begin(115200); delay(200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW Fail"); return; }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  analogReadResolution(12);
  Wire.begin();
  if(!tcs.begin()){ Serial.println("Color Sensor FAIL"); while(1); }
  tcs.setInterrupt(false);

  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, HIGH);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_ENC_A_1, INPUT); pinMode(PIN_ENC_B_1, INPUT);
  pinMode(PIN_ENC_A_2, INPUT); pinMode(PIN_ENC_B_2, INPUT);

  ledcAttach(PIN_PWMA, 5000, 10);
  ledcAttach(PIN_PWMB, 5000, 10);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_1), encISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_2), encISR2, RISING);

  Serial.println("=== ROBOT SLAVE READY (KINEMATIC OBSERVER) ===");
}

// ===================== LOOP =====================
void loop(){
  // --- HANDLE COMMAND ---
  if (newCommandReceived) {
    newCommandReceived = false; 
    if (commandChar == 'g') {
currentState = MOVING; 
      currentShipStatus = MOVING_TO_LOADING; 
      cargoColor = 0; isIntersection = false;
      resetPID();
      //current_biggest_err = 0;
      missionStartTime = millis(); 
      finalTravelTime = 0;
      Serial.println(">>> START MISSION");
}
    else if (commandChar == 's') {
      currentState = STOP; 
      currentShipStatus = STOPPED_BY_USER;
      Serial.println(">>> STOP CMD");
    }
  }

  // Time calculation
  unsigned long now = millis();
  if(now - lastLoopTime < DT_MS) return;
  float dt = (now - lastLoopTime) / 1000.0f; 
  lastLoopTime = now;

  // --- SEND TELEMETRY ---
  if (now - lastTeleTime > 100) { 
    sendTelemetry(dt); // Truyền dt vào để tính toán
    lastTeleTime = now; 
  }

  // --- RPM CALC ---
  long c1 = encCount1, c2 = encCount2;
  long dp1 = c1 - lastEnc1; lastEnc1 = c1;
  long dp2 = c2 - lastEnc2; lastEnc2 = c2;
  rpm_left = (dp1 / dt) * 60.0 / CPR;
  rpm_right = (dp2 / dt) * 60.0 / CPR;
  rpm_meas_1 = rpm_left; rpm_meas_2 = rpm_right;
  readLineSensors();

  // --- LOGIC ---
  isPickupZone = (norm_sensor[1]>=500 && norm_sensor[2]>=500 && norm_sensor[3]>=500);
  isIntersection = ((norm_sensor[1]>=500 && norm_sensor[2]>=500 && norm_sensor[3]>=500) && isPickupZone != 0 && currentShipStatus==MOVING_TO_INTERSECTION );
  isFinish = (norm_sensor[0]<300 && norm_sensor[1]<300 && norm_sensor[2]<300 && norm_sensor[3]<300 && norm_sensor[4]<300);

  if(currentState == MOVING){
    if(isFinish){
      setMotors(-1023,-1023);
      stateStart = now;
      if(now-stateStart<=600) {setMotors(-1023,-1023);}
      currentState = FINISH_SHIPPING;
      resetPID();
      //setMotors(-1023,-1023);
      finalTravelTime = millis() - missionStartTime;
      currentShipStatus = FINISHED_DELIVERY;
      return;
    }
    if(cargoColor==0 && isPickupZone){
      resetPID();
      cargo_color = 0;
      currentState = WAIT_LOADING; currentShipStatus = WAIT_AT_LOADING;
      stateStart = now; 
      return;
    }
    if(isIntersection && cargoColor!=0){
      currentState = INTERSECTION_DECISION; stateStart=now;
      return;
    }
    double currentErrAbs = abs(getLineError());
    if(currentErrAbs > abs(current_biggest_err)) current_biggest_err = currentErrAbs;
  }

  // --- STATE MACHINE ---
  switch(currentState){
    case MOVING:{
      err_line = getLineError();
      double P = Kp_line*err_line;
      integ_line += err_line*dt; integ_line=constrain(integ_line,-50,50);
      double I = Ki_line*integ_line;
      double D = Kd_line*(err_line-last_err_line)/dt;

      last_err_line = err_line; 
      //if(currentShipStatus==MOVING_TO_ENDPOINT) {base_rpm_set = 120;}
      double steer = P+I+D;
      double set1 = base_rpm_set + steer;
      double set2 = base_rpm_set - steer;

      integ_1 += (set1-rpm_meas_1)*Ki_1*dt; integ_1=constrain(integ_1,-1023,1023);
      double u1 = Kp_1*(set1-rpm_meas_1)+integ_1;
      integ_2 += (set2-rpm_meas_2)*Ki_2*dt; integ_2=constrain(integ_2,-1023,1023);
      double u2 = Kp_2*(set2-rpm_meas_2)+integ_2;
      setMotors((int)u1,(int)u2);
      break;
    }
    case STOP: setMotors(0,0); break;

    case WAIT_LOADING:{
      setMotors(-20,-20);
      static int lastColor=0;
      static unsigned long colorStart=0;
cargo_color = detectColorRaw();    
      if(cargo_color!=0 && cargo_color!=lastColor){ lastColor=cargo_color; colorStart=now; }
      if(cargo_color==0){ lastColor=0; colorStart=0; }
      if(colorStart>0 && now-colorStart>=500){
        setMotors(500,550);
        if(colorStart>0 && now-colorStart>=700){
          cargoColor=cargo_color;
          currentState=MOVING; currentShipStatus=MOVING_TO_INTERSECTION;
          stateStart=now;
        }
      }
      resetPID();
      break;
    }
    case INTERSECTION_DECISION:{
      if(cargoColor==1) err_line=-12; // RED
      else if(cargoColor==2) err_line=10; // BLUE
      double P = Kp_line*err_line;
      integ_line += err_line*dt; integ_line=constrain(integ_line,-50,50);
      double I = Ki_line*integ_line;
      double D = Kd_line*(err_line-last_err_line)/dt;
      last_err_line=err_line;
      double steer=P+I+D;
      double set1=base_rpm_set+steer;
      double set2=base_rpm_set-steer;
      integ_1+=(set1-rpm_meas_1)*Ki_1*dt; integ_1=constrain(integ_1,-1023,1023);
      double u1=Kp_1*(set1-rpm_meas_1)+integ_1;
      integ_2+=(set2-rpm_meas_2)*Ki_2*dt; integ_2=constrain(integ_2,-1023,1023);
      double u2=Kp_2*(set2-rpm_meas_2)+integ_2;
      setMotors((int)u1,(int)u2);
      if(now-stateStart>=200) { currentState=MOVING; currentShipStatus=MOVING_TO_ENDPOINT; }
      break;
    }
    case FINISH_SHIPPING: 
    setMotors(-20,-20); break;
    }
  }