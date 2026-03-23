/* * CODE ROBOT TRÊN XE 
 * Đã điền MAC Remote: 6C:C8:40:34:7A:68
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <esp_now.h>
#include <WiFi.h>

// --- 1. ĐỊA CHỈ CỦA REMOTE (Đã điền sẵn) ---
uint8_t remoteAddress[] = {0x6C, 0xC8, 0x40, 0x34, 0x7A, 0x68};

typedef struct struct_message {
  int s[5]; float err; int st; int inter; float r1; float r2; char msg[32];
} struct_message;
struct_message myData;

typedef struct struct_command { char cmd; } struct_command;
struct_command incomingCmd;

esp_now_peer_info_t peerInfo;

// --- PHẦN CỨNG (Theo sơ đồ cũ của bạn) ---
const int PIN_AIN1=14, PIN_AIN2=27, PIN_PWMA=25;
const int PIN_BIN1=13, PIN_BIN2=5, PIN_PWMB=26, PIN_STBY=4;
const int S1=36, S2=39, S3=34, S4=35, S5=32; // Line sensor pins

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Encoder vars
volatile long c1=0, c2=0; long lastEnc1=0, lastEnc2=0;
void IRAM_ATTR isr1(){ if(digitalRead(17)) c1++; else c1--; } // Pin 16 A, Pin 17 B
void IRAM_ATTR isr2(){ if(digitalRead(18)) c2++; else c2--; } // Pin 19 A, Pin 18 B

// PID Vars
double Kp1=0.19, Ki1=11, Kp2=0.20, Ki2=11, it1=0, it2=0;
double base=140, KpL=5.0, KiL=0.75, KdL=1.3, err=0, lastErr=0, itL=0;
int minS[]={53,43,30,33,27}, maxS[]={3200,2965,2748,2684,2691}, w[]={-20,-10,0,10,20}, ns[5];
float rC[]={68,26.7,19.6}, bC[]={50.3,63.2,54.8}, uC[]={36.5,33.9,26.0};
double mR1=0, mR2=0;

enum State { IDLE, RUN, SCAN, EXIT, LEFT, RIGHT };
State st = IDLE;
unsigned long tStart=0, lastLoop=0;
bool hasInter=false, hasTurn=false; int inter=0;

// --- HÀM HỖ TRỢ ---
void motors(int u1, int u2){
  u1=constrain(u1,-1023,1023); u2=constrain(u2,-1023,1023);
  digitalWrite(PIN_AIN1,u1>0); digitalWrite(PIN_AIN2,u1<0); ledcWrite(PIN_PWMA,abs(u1));
  digitalWrite(PIN_BIN1,u2>0); digitalWrite(PIN_BIN2,u2<0); ledcWrite(PIN_PWMB,abs(u2));
}
int mapS(int x, int mn, int mx){ if(x<=mn)return 0; if(x>=mx)return 1000; return (long)(x-mn)*1000/(mx-mn); }
float dist(float r, float g, float b, float c[]){ return sqrt(pow(r-c[0],2)+pow(g-c[1],2)+pow(b-c[2],2)); }
int getCol(){ 
  uint16_t r,g,b,c; tcs.getRawData(&r,&g,&b,&c);
  float dR=dist(r,g,b,rC), dB=dist(r,g,b,bC), dU=dist(r,g,b,uC);
  if(dB<dR && dB<dU) return 2; if(dR<dB && dR<dU) return 1; return 0;
}

// Callback nhận lệnh START/STOP
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *data, int len) {
  memcpy(&incomingCmd, data, sizeof(incomingCmd));
  if(incomingCmd.cmd == 'g') { 
    st=RUN; inter=0; hasInter=false; hasTurn=false; 
    Serial.println("CMD: GO"); 
  }
  if(incomingCmd.cmd == 's') { 
    st=IDLE; motors(0,0); 
    Serial.println("CMD: STOP"); 
  }
}
// Callback gửi (trống để tránh lỗi)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

void setup() {
  Serial.begin(115200); WiFi.mode(WIFI_STA); 
  if(esp_now_init()!=ESP_OK) {Serial.println("ESP-NOW FAIL"); return;}

  // ĐĂNG KÝ HÀM CALLBACK (ĐÃ FIX LỖI THƯ VIỆN MỚI)
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent); 

  memcpy(peerInfo.peer_addr, remoteAddress, 6); peerInfo.channel=0; peerInfo.encrypt=false;
  esp_now_add_peer(&peerInfo);

  Wire.begin(); 
  if(!tcs.begin()){Serial.println("TCS FAIL"); while(1);} 
  tcs.setInterrupt(false);

  pinMode(PIN_STBY,OUTPUT); digitalWrite(PIN_STBY,1);
  pinMode(14,OUTPUT); pinMode(27,OUTPUT); pinMode(25,OUTPUT);
  pinMode(13,OUTPUT); pinMode(5,OUTPUT); pinMode(26,OUTPUT);
  pinMode(16,INPUT); pinMode(17,INPUT); pinMode(19,INPUT); pinMode(18,INPUT);
  
  ledcAttach(25,20000,10); ledcAttach(26,20000,10);
  attachInterrupt(16,isr1,RISING); attachInterrupt(19,isr2,RISING);
  Serial.println("ROBOT READY. WAITING CMD...");
}

void loop() {
  if(millis()-lastLoop<50) return; 
  float dt=(millis()-lastLoop)/1000.0; lastLoop=millis();
  
  // Tinh RPM
  mR1=((c1-lastEnc1)/dt)*60.0/390.0; lastEnc1=c1;
  mR2=((c2-lastEnc2)/dt)*60.0/390.0; lastEnc2=c2;

  // Doc Sensor
  int p[]={36,39,34,35,32}; long sw=0, sv=0;
  for(int i=0;i<5;i++){ ns[i]=mapS(analogRead(p[i]),minS[i],maxS[i]); sw+=ns[i]*w[i]; sv+=ns[i]; }
  if(sv>300) err=(double)sw/sv;

  // Logic State Machine
  bool pickup=(ns[1]>400&&ns[2]>400&&ns[3]>400);
  bool ngaba=(ns[2]<300&&ns[0]>700&&ns[4]>700);
  bool diemcuoi=(ns[0]<200&&ns[1]<200&&ns[2]<200&&ns[3]<200&&ns[4]<200);
  
  if(st==RUN && diemcuoi){ st=IDLE; motors(0,0); return;}
  if(st==RUN && !hasInter && pickup){ st=SCAN; tStart=millis(); motors(-10,-10); return; }
  if(st==RUN && ngaba){ 
    if(!hasInter){ st=IDLE; motors(0,0); return; }
    st=(inter==1)?LEFT:RIGHT; tStart=millis(); hasTurn=true; motors(0,0); return;
  }

  switch(st){
    case IDLE: motors(0,0); break;
    case SCAN: {
       motors(-10,-10); int c=getCol(); 
       if(c!=0){ delay(100); if(getCol()==c){ inter=c; hasInter=true; st=EXIT; tStart=millis(); delay(2000);}}
    } break;
    case EXIT: motors(250,250); if(millis()-tStart>400){ st=RUN; it1=it2=itL=lastErr=0; } break;
    case LEFT: motors(-250,250); if(millis()-tStart>150){ st=RUN; it1=it2=itL=lastErr=0; } break;
    case RIGHT: motors(250,-250); if(millis()-tStart>150){ st=RUN; it1=it2=itL=lastErr=0; } break;
    case RUN: {
      double P=KpL*err, I=KiL*(itL=constrain(itL+err*dt,-50,50)), D=KdL*(err-lastErr)/dt; lastErr=err;
      double steer=P+I+D;
      double s1=base+steer, s2=base-steer;
      it1=constrain(it1+(s1-mR1)*Ki1*dt,-1023,1023); 
      it2=constrain(it2+(s2-mR2)*Ki2*dt,-1023,1023);
      motors(Kp1*(s1-mR1)+it1, Kp2*(s2-mR2)+it2);
    } break;
  }

  // Gui Telemetry ve Remote
  static unsigned long lp=0;
  if(millis()-lp>200){
    lp=millis();
    String m = (st==IDLE)?"DUNG IM":(st==SCAN?"QUET MAU":"DANG CHAY");
    if(inter==1) m+=" (RED)"; if(inter==2) m+=" (BLUE)";
    m.toCharArray(myData.msg,32); myData.st=st; myData.r1=mR1; myData.r2=mR2; myData.err=err;
    esp_now_send(remoteAddress, (uint8_t*)&myData, sizeof(myData));
  }
}