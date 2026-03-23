/*
 * ESP32 ROBOT - SLAVE (FINAL VERSION)
 * ==================================================
 * - Feature: Tính toán thời gian chạy & Lỗi lớn nhất
 * - Send: Gửi dữ liệu tổng kết về Master khi về đích
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <esp_now.h>
#include <WiFi.h>

// ==========================================================
// 1. CẤU HÌNH ESP-NOW & DATA STRUCT
// ==========================================================
uint8_t masterAddress[] = {0x6C, 0xC8, 0x40, 0x34, 0x7A, 0x68}; // <--- THAY MAC MASTER CỦA BẠN VÀO ĐÂY

// Struct GỬI ĐI (Phải giống hệt bên Master)
typedef struct struct_telemetry {
  int status;       
  int cargoColor;   
  float rpmLeft;
  float rpmRight;
  float errorLine;
  int s1; int s2; int s3; int s4; int s5;
  float biggestError;      // Lỗi lệch line lớn nhất
  unsigned long travelTime; // Tổng thời gian chạy (ms)
} struct_telemetry;

// Struct NHẬN VỀ
typedef struct struct_command {
  char cmd; 
} struct_command;

struct_telemetry myData;
struct_command incomingCmd;

volatile bool newCommandReceived = false;
volatile char commandChar = ' ';

// ==========================================================
// 2. HARDWARE
// ==========================================================
const int PIN_AIN1 = 14, PIN_AIN2 = 27, PIN_PWMA = 25;
const int PIN_BIN1 = 13, PIN_BIN2 = 5, PIN_PWMB = 26;
const int PIN_STBY = 4;
const int PIN_ENC_A_1 = 16, PIN_ENC_B_1 = 17;
const int PIN_ENC_A_2 = 19, PIN_ENC_B_2 = 18;
const int S1 = 36, S2 = 39, S3 = 34, S4 = 35, S5 = 32;

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const int CPR = 390;
const float DT_MS = 25;

double rpm_meas_1 = 0, rpm_meas_2 = 0;
double rpm_left = 0, rpm_right = 0;

// ==========================================================
// 3. PID & BIẾN LOGIC
// ==========================================================
double Kp_1 = 0.19, Ki_1 = 11.0;
double Kp_2 = 0.20, Ki_2 = 11.0;
double integ_1 = 0, integ_2 = 0;

double base_rpm = 160;
double base_rpm_set = 0.0;
double Kp_line = 5.0, Ki_line = 0.75, Kd_line = 1.3;
double err_line = 0, last_err_line = 0, integ_line = 0;

// --- BIẾN THỐNG KÊ ---
double current_biggest_err = 0;
unsigned long missionStartTime = 0;
unsigned long finalTravelTime = 0;

int rawMin[5] = {44, 24, 34, 19, 37};
int rawMax[5] = {2852, 2331, 2533, 2066, 2509};

int weight[5] = {-24, -12, 0, 12, 24};
int norm_sensor[5];
/*
float redCalib[3]  = {47.8, 20.9, 14.9};
float blueCalib[3] = {38.0, 45.6, 38.6};
float unkCalib[3]  = {26.0, 24.0, 16.8};
*/
float redCalib[3]  = {161.0, 27.0, 25.0};
float blueCalib[3] = {74.0, 89.0, 70.0};
float unkCalib[3]  = {58.3, 67.4, 51.2};

volatile long encCount1 = 0, encCount2 = 0;
long lastEnc1 = 0, lastEnc2 = 0;
unsigned long lastLoopTime = 0;
unsigned long lastTeleTime = 0;


enum RobotState { MOVING, STOP, WAIT_LOADING, INTERSECTION_DECISION, FINISH_SHIPPING };
enum ShippingStatus { 
  WAIT_TO_START, MOVING_TO_LOADING, WAIT_AT_LOADING, 
  MOVING_TO_INTERSECTION, MOVING_TO_ENDPOINT, FINISHED_DELIVERY, STOPPED_BY_USER 
};

RobotState currentState = STOP;
ShippingStatus currentShipStatus = WAIT_TO_START;

unsigned long stateStart = 0; 
int cargoColor = 0; 
bool isPickupZone = false, isIntersection = false, isFinish = false; 

// ===================== ESP-NOW =====================
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
  memcpy(&incomingCmd, incomingData, sizeof(incomingCmd));
  commandChar = incomingCmd.cmd;
  newCommandReceived = true;
}

void sendTelemetry() {
  myData.status = (int)currentShipStatus;
  myData.cargoColor = cargoColor;
  myData.rpmLeft = (float)rpm_left;
  myData.rpmRight = (float)rpm_right;
  myData.errorLine = (float)err_line;
  myData.s1 = norm_sensor[0]; myData.s2 = norm_sensor[1]; 
  myData.s3 = norm_sensor[2]; myData.s4 = norm_sensor[3]; myData.s5 = norm_sensor[4];
  
  // Gửi giá trị thống kê
  myData.biggestError = (float)current_biggest_err;
  myData.travelTime = finalTravelTime; 

  esp_now_send(masterAddress, (uint8_t *) &myData, sizeof(myData));
}

// ===================== HELPERS =====================
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
  float minD = dR; int color = 1; 
  if(dB < minD){ minD = dB; color = 2; }
  if(dU < minD) color = 0; 
  return color;
}

void resetPID() {
  integ_1 = integ_2 = 0; integ_line = 0; last_err_line = 0;
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

  Serial.println("=== ROBOT READY ===");
}

// ===================== LOOP =====================
void loop(){
  // 1. XỬ LÝ LỆNH
  if (newCommandReceived) {
    newCommandReceived = false; 
    if (commandChar == 'g') {
      //setMotors(250,250);
      currentState = MOVING; 
      currentShipStatus = MOVING_TO_LOADING; 
      cargoColor = 0; isIntersection = false;
      
      // --- RESET THÔNG SỐ CHẠY ---
      resetPID();
      current_biggest_err = 0;
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

  // 2. TELEMETRY
  unsigned long now = millis();
  if (now - lastTeleTime > 100) { sendTelemetry(); lastTeleTime = now; }

  if(now - lastLoopTime < DT_MS) return;
  float dt = (now - lastLoopTime) / 1000.0f; 
  lastLoopTime = now;

  // Tính RPM
  long c1 = encCount1, c2 = encCount2;
  long dp1 = c1 - lastEnc1; lastEnc1 = c1;
  long dp2 = c2 - lastEnc2; lastEnc2 = c2;
  rpm_left = (dp1 / dt) * 60.0 / CPR;
  rpm_right = (dp2 / dt) * 60.0 / CPR;
  rpm_meas_1 = rpm_left; rpm_meas_2 = rpm_right;

  readLineSensors();
  int s1 = norm_sensor[0], s2 = norm_sensor[1], s3 = norm_sensor[2], s4 = norm_sensor[3], s5 = norm_sensor[4];

  // 3. LOGIC CẢM BIẾN
  isPickupZone = (s2 >= 400 && s3 >= 400 && s4 >= 400);
  isIntersection = (s3 <= 300 && s1 >= 700 && s5 >= 700);
  isFinish = (s1 < 200 && s2 < 200 && s3 < 200 && s4 < 200 && s5 < 200); 

  // Xử lý sự kiện đặc biệt khi đang chạy
  if (currentState == MOVING) {
    // Cập nhật lỗi lớn nhất
    double currentErrAbs = abs(getLineError());
    if (currentErrAbs > abs(current_biggest_err)) {
        current_biggest_err = currentErrAbs;
    }

    static unsigned long finishTimer = 0; 
    if (isFinish) {
      if (finishTimer == 0) {
        // Lần đầu tiên phát hiện isFinish, bắt đầu hẹn giờ
        finishTimer = now; 
      } else if (now - finishTimer >= 100) {
        finalTravelTime = now - missionStartTime; // Tính thời gian bằng current time - start time
        currentState = FINISH_SHIPPING; 
        currentShipStatus = FINISHED_DELIVERY;
        finishTimer = 0; // Reset timer cho lần chạy sau
        return;
      }
      } else {
      finishTimer = 0;
      resetPID();
    }


    if (cargoColor == 0 && isPickupZone) {
      setMotors(-20, -20); resetPID();
      currentState = WAIT_LOADING; currentShipStatus = WAIT_AT_LOADING;
      stateStart = now; 
      return;
    }
    if (isIntersection && cargoColor != 0) {
      currentState = INTERSECTION_DECISION; stateStart = now;
      return;
    }
  }

  // 4. ĐIỀU KHIỂN ĐỘNG CƠ (STATE MACHINE)
  switch(currentState){
    case MOVING: {
      if(cargoColor != 0 && isIntersection == false){base_rpm_set = 120.0;}
      else {base_rpm_set = base_rpm;}
      err_line = getLineError();
      double P = Kp_line * err_line;
      integ_line += err_line * dt; integ_line = constrain(integ_line, -50, 50);
      double I = Ki_line * integ_line;
      double D = Kd_line * (err_line - last_err_line) / dt;
      last_err_line = err_line;

      double steer = P + I + D;
      double set1 = base_rpm_set + steer;
      double set2 = base_rpm_set - steer;

      integ_1 += (set1 - rpm_meas_1) * Ki_1 * dt; integ_1 = constrain(integ_1, -1023, 1023);
      double u1 = Kp_1 * (set1 - rpm_meas_1) + integ_1;

      integ_2 += (set2 - rpm_meas_2) * Ki_2 * dt; integ_2 = constrain(integ_2, -1023, 1023);
      double u2 = Kp_2 * (set2 - rpm_meas_2) + integ_2;

      setMotors((int)u1, (int)u2);
      break; 
    }

    case STOP: 
      setMotors(-20, -20); 
      break;

    case WAIT_LOADING: {
      setMotors(-20, -20);
      static int lastColor = 0; static unsigned long colorStart = 0; 
      int cargo_color = detectColorRaw();    
      if (cargo_color != 0 && cargo_color != lastColor) { lastColor = cargo_color; colorStart = now; }
      if (cargo_color == 0) { lastColor = 0; colorStart = 0; }

      if (colorStart > 0 && (now - colorStart >= 3000)) {
        cargoColor = cargo_color;
        currentState = MOVING; currentShipStatus = MOVING_TO_INTERSECTION;
        stateStart = now;
      }
      resetPID();
      break;
    }

    case INTERSECTION_DECISION: 
      setMotors(50,50);
      resetPID();
      if (cargoColor == 1) { // RED -> LEFT
        //resetPID(); 
        setMotors(150, 255); 
        //if (now - stateStart >= 250) { 
          resetPID(); currentState = MOVING; currentShipStatus = MOVING_TO_ENDPOINT; 
         // }
      } 
      else if (cargoColor == 2) { // BLUE -> RIGHT
        resetPID(); 
        setMotors(255, 150); 
        //if (now - stateStart >= 250) { 
          resetPID(); currentState = MOVING; currentShipStatus = MOVING_TO_ENDPOINT; 
        //}
      } 
      else currentState = STOP;
      break;
        
    case FINISH_SHIPPING: 
      setMotors(-20, -20); 
      // Ở trạng thái này, robot vẫn gửi sendTelemetry chứa finalTravelTime
      break;
  } 
}