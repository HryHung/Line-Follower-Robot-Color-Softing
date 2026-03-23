// Motor A (left)
#define AIN1 14
#define AIN2 27
#define PWMA 25

// Motor B (right)
#define BIN1 13
#define BIN2 5
#define PWMB 26

#define STBY 4

void setup() {
  Serial.begin(115200);
  delay(300);

  // Direction pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Standby
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);   // enable TB6612

  // Set direction: forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  // PWM attach theo API mới
  ledcAttach(PWMA, 20000, 10);   // 20 kHz, 10-bit (0–1023)
  ledcAttach(PWMB, 20000, 10);

  // Set PWM = 100%
  ledcWrite(PWMA, 1023);
  ledcWrite(PWMB, 1023);

  Serial.println("Motors running at 100% PWM");
}

void loop() {
  delay(1000);
}
