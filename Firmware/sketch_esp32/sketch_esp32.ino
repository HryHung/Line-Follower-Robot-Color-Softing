#include <Arduino.h>

// =============================
// Task Handles
// =============================
TaskHandle_t TaskCore0;
TaskHandle_t TaskCore1;

// =============================
// Pin cấu hình
// =============================
// TCRT5000
const int tcrtPins[7] = {34, 35, 32, 33, 25, 26, 27}; 

// TB6612FNG
#define AIN1 14
#define AIN2 12
#define PWMA 13
#define BIN1 18
#define BIN2 19
#define PWMB 23
#define STBY 5

// Encoder
#define ENCODER_LEFT_A 36
#define ENCODER_RIGHT_A 39

// TCS3200
#define S0 4
#define S1 16
#define S2 17
#define S3 21
#define OUT 22

// =============================
// Biến toàn cục / dữ liệu chia sẻ
// =============================
volatile int tcrtValues[7];     
volatile int encoderLeft = 0;
volatile int encoderRight = 0;
volatile float lineError = 0;
volatile float deltaError = 0;
volatile int targetSpeed = 120;   // PWM base
volatile int motorLeftPWM = 0;
volatile int motorRightPWM = 0;
volatile bool hasCargo = false;
volatile int cargoColor = 0;      // 1=Red, 2=Blue

// =============================
// PID parameters
// =============================
float Kp_line = 30, Ki_line = 0, Kd_line = 15;
float Kp_speed = 0.5, Ki_speed = 0, Kd_speed = 0.1;

float lastError = 0;
float integral_line = 0;

// =============================
// Hàm khởi tạo
// =============================
void Init_Sensors() {
  for (int i = 0; i < 7; i++) pinMode(tcrtPins[i], INPUT);
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  digitalWrite(S0, HIGH); 
  digitalWrite(S1, LOW);  // scale = 20%
}

void Init_PID_Fuzzy() {
  // mặc định dùng PID line, fuzzy cho speed (simple mapping)
}

void Init_Motor_Driver() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
}

void Init_Communication() {
  Serial.begin(115200);
}

// =============================
// Các hàm xử lý
// =============================
void Read_TCRT5000_Array() {
  for (int i = 0; i < 7; i++) {
    tcrtValues[i] = analogRead(tcrtPins[i]); // 0–4095
  }
}

void Read_Encoder() {
  // Giả lập (thực tế: ISR tăng biến encoder)
}

void Filter_Threshold() {
  for (int i = 0; i < 7; i++) {
    if (tcrtValues[i] > 2000) tcrtValues[i] = 1; else tcrtValues[i] = 0;
  }
}

// [-3; -2; -1; 0; 1; 2; 3]
float Centroid_Calc() {
  int sum = 0, count = 0;
  for (int i = 0; i < 7; i++) {
    if (tcrtValues[i]) {
      sum += (i - 3); // center = 0
      count++;
    }
  }
  if (count == 0) return 999; // lost line
  return (float)sum / count;
}

void Fuzzy_Adaptive_Speed() {
  // đơn giản: khi error lớn thì giảm speed
  if (abs(lineError) > 2) targetSpeed = 80;
  else targetSpeed = 120;
}

void PID_Line_Control() {
  float P = lineError;
  integral_line += lineError;
  float D = lineError - lastError;
  float output = Kp_line * P + Ki_line * integral_line + Kd_line * D;
  lastError = lineError;

  motorLeftPWM  = targetSpeed - output;
  motorRightPWM = targetSpeed + output;

  motorLeftPWM  = constrain(motorLeftPWM, 0, 255);
  motorRightPWM = constrain(motorRightPWM, 0, 255);
}

void PID_Speed_Control() {
  // placeholder, có thể bù theo encoder
}

void Motor_Output(int left, int right) {
  // Motor A (Left)
  if (left >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); left = -left; }
  ledcWrite(0, abs(left));

  // Motor B (Right)
  if (right >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); right = -right; }
  ledcWrite(1, abs(right));
}

bool LineLost() {
  for (int i = 0; i < 7; i++) if (tcrtValues[i] == 1) return false;
  return true;
}

void Recovery_Algorithm() {
  // quay tại chỗ tìm line
  Motor_Output(80, -80);
  delay(100);
}

int Read_TCS3200_Color() {
  // Giản lược: đo tần số cho kênh RED/BLUE
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  int red = pulseIn(OUT, LOW, 10000);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  int blue = pulseIn(OUT, LOW, 10000);

  if (red < blue) return 1; // Red
  else return 2;            // Blue
}

void Decision_Checkpoint() {
  cargoColor = Read_TCS3200_Color();
  hasCargo = true;
  if (cargoColor == 1) {
    Serial.println("Cargo RED -> Đi trái");
  } else if (cargoColor == 2) {
    Serial.println("Cargo BLUE -> Đi phải");
  }
}

void Send_Data_UART() {
  Serial.print("Err: "); Serial.print(lineError);
  Serial.print(" L: "); Serial.print(motorLeftPWM);
  Serial.print(" R: "); Serial.println(motorRightPWM);
}

// =============================
// TASK Core 1 (APP CPU) – Real-time
// =============================
void TaskCore1Code(void *pvParameters) {
  for (;;) {
    Read_TCRT5000_Array();
    Read_Encoder();
    Filter_Threshold();

    float linePos = Centroid_Calc();
    if (linePos == 999) {
      Recovery_Algorithm();
      continue;
    }

    lineError = linePos;
    deltaError = lineError - lastError;

    // PID / Fuzzy
    Fuzzy_Adaptive_Speed();
    PID_Line_Control();
    PID_Speed_Control();

    // Xuất PWM ra motor driver
    Motor_Output(motorLeftPWM, motorRightPWM);

    vTaskDelay(5 / portTICK_PERIOD_MS);  // loop ~200Hz
  }
}

// =============================
// TASK Core 0 (PRO CPU) – Logic & Communication
// =============================
void TaskCore0Code(void *pvParameters) {
  for (;;) {
    // Giả định: checkpoint khi tất cả sensor đều = 1
    int sum = 0; for (int i = 0; i < 7; i++) sum += tcrtValues[i];
    if (sum >= 5 && !hasCargo) {
      Decision_Checkpoint();
    }

    Send_Data_UART();
    vTaskDelay(50 / portTICK_PERIOD_MS); // loop ~20Hz
  }
}

// =============================
// Setup
// =============================
void setup() {
  Init_Sensors();
  Init_PID_Fuzzy();
  Init_Motor_Driver();
  Init_Communication();

  // PWM channel
  ledcSetup(0, 1000, 8); ledcAttachPin(PWMA, 0);
  ledcSetup(1, 1000, 8); ledcAttachPin(PWMB, 1);

  // Tạo task cho từng core
  xTaskCreatePinnedToCore(TaskCore1Code,"TaskCore1",8192,NULL,2,&TaskCore1,1);
  xTaskCreatePinnedToCore(TaskCore0Code,"TaskCore0",8192,NULL,1,&TaskCore0,0);
}

void loop() {
  // loop() trống, vì đã dùng FreeRTOS task
}
