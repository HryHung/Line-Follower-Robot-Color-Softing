/*
 * ESP32 ROBOT – CLEANED AND STATUS-TRACKED VERSION
 * ==================================================
 * - Logic: Line Following + Pickup Zone + Intersection + End Point
 * - Updated Status Names and Tracking Variables
 * - RPM, Line Sensors, and Cargo Color printed
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// ===================== HARDWARE PINS =====================
const int PIN_AIN1 = 14, PIN_AIN2 = 27, PIN_PWMA = 25;
const int PIN_BIN1 = 13, PIN_BIN2 = 5, PIN_PWMB = 26;
const int PIN_STBY = 4;

const int PIN_ENC_A_1 = 16, PIN_ENC_B_1 = 17;
const int PIN_ENC_A_2 = 19, PIN_ENC_B_2 = 18;

const int S1 = 36, S2 = 39, S3 = 34, S4 = 35, S5 = 32;

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ===================== PID PARAMETERS =====================
double Kp_speed = 0.19, Ki_speed = 11.0;
double Kp_line = 5.0, Ki_line = 0.75, Kd_line = 1.3;
double base_rpm = 140.0;
double integ_speed1 = 0, integ_speed2 = 0, integ_line = 0, last_err_line = 0;

// ===================== SENSOR CALIBRATION =====================
int rawMin[5] = {53, 43, 30, 33, 27};
int rawMax[5] = {3200, 2965, 2748, 2684, 2691};
int weight[5] = {-20, -10, 0, 10, 20};
int norm_sensor[5];

float redCalib[3]  = {47.8, 20.9, 14.9};
float blueCalib[3] = {38.0, 45.6, 38.6};
float unkCalib[3]  = {26.0, 24.0, 16.8};

// ===================== ENCODER =====================
volatile long encCount1 = 0, encCount2 = 0;
long lastEnc1 = 0, lastEnc2 = 0;
double rpm_left = 0, rpm_right = 0;
const int CPR = 390;
const unsigned long DT_MS = 50;
unsigned long lastLoopTime = 0;

// ===================== STATE MACHINE =====================
enum RobotState {
  MOVING,
  STOP,
  MOVING_TO_LOADING,
  ARRIVE_LOADING,
  WAIT_LOADING,
  MOVING_TO_INTERSECTION,
  INTERSECTION_DECISION,
  MOVING_TO_END_POINT,
  FINISH_SHIPPING
};

RobotState currentState = STOP;
unsigned long stateStart = 0; 
bool hasIntersection = false;
int cargoColor = 0; // 0=UNKNOWN,1=RED,2=BLUE

// ===================== HELPERS =====================
void IRAM_ATTR encISR1() { if(digitalRead(PIN_ENC_B_1)) encCount1++; else encCount1--; }
void IRAM_ATTR encISR2() { if(digitalRead(PIN_ENC_B_2)) encCount2++; else encCount2--; }

void motorDrive(int pin1,int pin2,int pwmPin,int val){
  val = constrain(val,-1023,1023);
  if(val>0){digitalWrite(pin1,HIGH);digitalWrite(pin2,LOW);}
  else if(val<0){digitalWrite(pin1,LOW);digitalWrite(pin2,HIGH);}
  else{digitalWrite(pin1,LOW);digitalWrite(pin2,LOW);}
  ledcWrite(pwmPin,abs(val));
}

void setMotors(int left,int right){
  motorDrive(PIN_AIN1,PIN_AIN2,PIN_PWMA,left);
  motorDrive(PIN_BIN1,PIN_BIN2,PIN_PWMB,right);
}

int clampMap(int x,int in_min,int in_max,int out_min,int out_max){
  if(x<=in_min) return out_min;
  if(x>=in_max) return out_max;
  return (long)(x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

void readLineSensors(){
  int pins[5]={S1,S2,S3,S4,S5};
  for(int i=0;i<5;i++) norm_sensor[i]=clampMap(analogRead(pins[i]),rawMin[i],rawMax[i],0,1000);
}

double getLineError(){
  long sumW=0,sumV=0;
  for(int i=0;i<5;i++){sumW+=norm_sensor[i]*weight[i];sumV+=norm_sensor[i];}
  if(sumV<300) return last_err_line;
  return (double)sumW/sumV;
}

float getDist(float r,float g,float b,float calib[3]){ return sqrt(pow(r-calib[0],2)+pow(g-calib[1],2)+pow(b-calib[2],2)); }

int detectColorRaw(){
  uint16_t r,g,b,c;
  tcs.getRawData(&r,&g,&b,&c);
  float fR=r,fG=g,fB=b;
  float dR=getDist(fR,fG,fB,redCalib);
  float dB=getDist(fR,fG,fB,blueCalib);
  float dU=getDist(fR,fG,fB,unkCalib);
  float minD=dR; int color=1;
  if(dB<minD){minD=dB;color=2;}
  if(dU<minD) color=0;
  return color;
}

void resetPID(){ integ_speed1=integ_speed2=integ_line=last_err_line=0; }

double getSteeringAdjustment(double err, double dt){
    integ_line += err * dt;
    double deriv = (err - last_err_line) / dt;
    last_err_line = err;
    return Kp_line*err + Ki_line*integ_line + Kd_line*deriv;
}

int calcMotorPWM(double rpm_meas, double rpm_set, double &integ){
    double error = rpm_set - rpm_meas;
    integ += error * (DT_MS/1000.0);
    double pwm = Kp_speed*error + Ki_speed*integ;
    return constrain((int)pwm,-1023,1023);
}

// ===================== SETUP =====================
void setup(){
  Serial.begin(115200); delay(200);
  analogReadResolution(12);
  Wire.begin();
  if(!tcs.begin()){Serial.println("Color Sensor FAIL");while(1);}
  tcs.setInterrupt(false);

  pinMode(PIN_STBY,OUTPUT); 
  digitalWrite(PIN_STBY,HIGH);
  pinMode(PIN_AIN1,OUTPUT); 
  pinMode(PIN_AIN2,OUTPUT);
  pinMode(PIN_BIN1,OUTPUT); 
  pinMode(PIN_BIN2,OUTPUT);
  pinMode(PIN_ENC_A_1,INPUT); pinMode(PIN_ENC_B_1,INPUT);
  pinMode(PIN_ENC_A_2,INPUT); pinMode(PIN_ENC_B_2,INPUT);

  ledcAttach(PIN_PWMA,0,10); ledcAttach(PIN_PWMB,1,10);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_1),encISR1,RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_2),encISR2,RISING);

  Serial.println("Press: "g" to go  || "s" to stop");
}

// ===================== LOOP =====================
void loop(){

//======================START/STOP=================
    while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inLine.length()) haveLine = true;
    } else inLine += c;
  }

  if (haveLine) {
    inLine.trim();
    if (inLine == "g") {currentState = MOVING_TO_LOADING; intersection = 0; has_intersection = false;}
    else if (inLine == "s") { currentState = STOP; setMotors(-20,-20); }
    haveLine = false;
    inLine = "";
  }

//=================DATA ANALYSYS=========
  unsigned long now=millis();
  if(now-lastLoopTime<DT_MS) return;
  float dt=(now-lastLoopTime)/1000.0f; lastLoopTime=now;

  long c1=encCount1,c2=encCount2;
  long dp1=c1-lastEnc1,lastEnc1=c1;
  long dp2=c2-lastEnc2,lastEnc2=c2;
  rpm_left=(dp1/dt)*60.0/CPR;
  rpm_right=(dp2/dt)*60.0/CPR;

  readLineSensors();
  int s1=norm_sensor[0],s2=norm_sensor[1],s3=norm_sensor[2],s4=norm_sensor[3],s5=norm_sensor[4];

  // =============LOOP: 3. SPECIAL CASE CHECK===========

  bool isPickupZone = ((s2 >= 400 && s3 >= 400 && s4 >= 400));
  bool iscargo_type = (s3 <= 300 && s1 >= 700 && s5 >= 700);
  bool isFinish = (s1<200 && s2<200 && s3<200 && s4<200 && s5<200); 

  //A. Check finish
  if (currentState == MOVING_TO_END_POINT && isFinish) {
       currentState = FINISH_SHIPPING; 
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
      Serial.println(" HADN'T LOADING → STOP");
      currentState = STOP;
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

  // ========== STATUS LOGIC ==========
  switch(currentState){

    case MOVING: {

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

    case STOP: setMotors(-20,-20); break;

    case MOVING_TO_LOADING:
      ;
      if(s2>=400 && s3>=400 && s4>=400){ currentState=ARRIVE_LOADING; stateStart=now; }
      break;

    case ARRIVE_LOADING:
      setMotors(-20,-20);
      if(now-stateStart>500){ currentState=WAIT_LOADING; stateStart=now; }
      break;

    case WAIT_LOADING:
      cargoColor=detectColorRaw();
      if(cargoColor>0){
        Serial.print("Cargo loaded: "); Serial.println(cargoColor==1?"RED":"BLUE");
        hasIntersection=true;
        currentState=MOVING_TO_INTERSECTION;
      }
      break;

    case MOVING_TO_INTERSECTION: 
      if(s3<=300 && s1>=700 && s5>=700){ currentState=INTERSECTION_DECISION; }
      break;

    case INTERSECTION_DECISION:
      if(cargoColor==1){
        setMotors(-250, 250);
        if (now - stateStart >= 100) {
          setMotors(50,50);
        Serial.println("RED CARGO: TURING LEFT"); 
        currentState=MOVING_TO_END_POINT;
          resetPID();
        }
      }
      else if(cargoColor==2){
        setMotors(150, -150);
        if (now - stateStart >= 100) {
          setMotors(50,50);
          erial.println("BLUE CARGO: TURNING RIGHT"); 
          currentState=MOVING_TO_END_POINT;
          resetPID();
        }
      }
      break;

    case MOVING_TO_END_POINT: setMotors(120,120);
      if(s1<200 && s2<200 && s3<200 && s4<200 && s5<200){ currentState=FINISH_SHIPPING; }
      break;
      
    case FINISH_SHIPPING: 
      setMotors(-20,-20); 
      Serial.println("FINISH SHIPPING"); 
      break;
  }

  // ====== SERIAL DEBUG ======
  Serial.print("Status: ");
  Serial.print(currentState); Serial.print(" | ");
  Serial.print("sensors: ");
  Serial.print(s1); Serial.print(",");
  Serial.print(s2); Serial.print(",");
  Serial.print(s3); Serial.print(",");
  Serial.print(s4); Serial.print(",");
  Serial.print(s5); Serial.print(" | ");
  Serial.print("Cargo: "); Serial.print(cargoColor); Serial.print(" | ");
  Serial.print("RPM: L="); Serial.print(rpm_left); Serial.print(" R="); Serial.println(rpm_right);
}
 