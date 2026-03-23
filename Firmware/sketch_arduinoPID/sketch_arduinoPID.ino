  //Project: Đồ án chuyên nghành: Xe dò line
  //Author: Lê Huy Hùng
  //Last updated: 30/09/25
  
  #include <SparkFun_TB6612.h>

  #define AIN1 4
  #define BIN1 6
  #define AIN2 3
  #define BIN2 7
  #define PWMA 9
  #define PWMB 10
  #define STBY 5

  const int offsetA = 1;
  const int offsetB = 1;

  Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
  Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

  int minValues[6], maxValues[6], threshold[6]; //Thực nghiệm sau
  int P, I, D, previousError;
  int lsp, rsp;
  int speed = 200;

  float Kp = 0.6, Ki = 0, Kd = 6.0;   // tuỳ chỉnh

  // ==================== SENSOR ====================

  // hiệu chỉnh min/max
  void calibrate() {
    for (int i = 1; i =< 5; i++) {
      minValues[i] = analogRead(i);
      maxValues[i] = analogRead(i);
    }

    for (int j = 0; j < 3000; j++) {
      motor1.drive(50);
      motor2.drive(-50);

      for (int i = 1; i =< 5; i++) {
        int val = analogRead(i);
        if (val < minValues[i]) minValues[i] = val;
        if (val > maxValues[i]) maxValues[i] = val;
      }
    }

    motor1.drive(0);
    motor2.drive(0);
  }

  // tính ngưỡng threshold
  void computeThresholds() {
    for (int i = 1; i =< 5; i++) {
      threshold[i] = (minValues[i] + maxValues[i]) / 2;
    }
  }

  // đọc sensor và chuẩn hóa về 0–1000
  int normalizeSensor(int pin) {
    int val = analogRead(pin);
    if (val < minValues[pin]) val = minValues[pin];
    if (val > maxValues[pin]) val = maxValues[pin];
    return map(val, minValues[pin], maxValues[pin], 0, 1000);
  }

  // tính sai số weighted method [-2,-1,0,1,2] 
  int lineError() {
    long weightVal = 0, totalVal = 0; //Value [0,4095] 12bits
    int weight[5] = {-2, -1, 0, 1, 2};

    for (int i = 1; i =< 5; i++) {
      int normVal = normalizeSensor(i);
      weightVal  += (long)weight[i-1] * normVal;
      totalVal += normVal;
    }

    if (totalVal == 0) return 0;   // tránh chia 0
    int line_error = weightVal / totalVal;
    return line_error;
  }

  // ==================== PID ====================
  int computePID(int error) {
    P = error * Kp;
    I = (I + error) * Ki;
    D = (P - previousError) * Kd;
    previousError = error;
    return P+I+D;
  }


  // ==================== MOTOR ====================

  void setMotorSpeed(int left, int right) {
    if (left > 255) left = 255;
    if (left < 0)   left = 0;
    if (right > 255) right = 255;
    if (right < 0)   right = 0;

    motor1.drive(left);
    motor2.drive(right);
  }

  // ==================== MAIN ====================

  void setup() {
    I = 0; 
    previousError = 0;
    Serial.begin(9600);
    pinMode(11, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);

    while (digitalRead(11)) {}  // chờ nút
    delay(1000);
    calibrate();
    computeThresholds();

    while (digitalRead(12)) {}
    delay(1000);
  }

  void loop() {
    int error = lineError();
    int PIDvalue = computePID(error);
    int speed_safety = 0;

    if(error >= 2000 && error <= 4000) speed = -0.5;
    if(error >= 4000 && error <= 6000) speed = -1;
    else errror = 0;

    lsp = speed - PIDvalue + speed_safety;
    rsp = speed + PIDvalue + speed_safety;

    setMotorSpeed(lsp, rsp);
  }
