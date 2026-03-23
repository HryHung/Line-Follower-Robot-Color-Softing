/*
 * ESP32 MASTER CONTROLLER - UI UPGRADE (DASHBOARD STYLE)
 * ======================================================
 * - Wifi: ROBOT_CENTER (Pass: 12345678)
 * - Web: http://192.168.4.1
 * - Tính năng: Monitor 7 trạng thái, Biểu đồ cảm biến, Log hệ thống
 */

#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>

// 1. ĐỊA CHỈ CỦA ROBOT (SLAVE)
uint8_t robotAddress[] = {0x4C, 0xC3, 0x82, 0xBE, 0xD9, 0xF8};

WebServer server(80);

// --- CẬP NHẬT STRUCT ĐỂ NHẬN THÊM CẢM BIẾN ---
// (Lưu ý: Phải cập nhật struct này bên code Slave cho giống y hệt)
typedef struct struct_telemetry {
  int status;       
  int cargoColor;   
  float rpmLeft;
  float rpmRight;
  float errorLine;
  // Thêm 5 giá trị cảm biến để hiển thị
  int s1;
  int s2;
  int s3;
  int s4;
  int s5;
} struct_telemetry;

typedef struct struct_command {
  char cmd; 
} struct_command;

struct_telemetry incomingData;
struct_command cmdToSend;

// Biến lưu trữ dữ liệu hiển thị
volatile int currentStatus = 0;
volatile int currentColor = 0;
volatile float rpmL = 0, rpmR = 0;
volatile int sens[5] = {0, 0, 0, 0, 0};

// Biến Log cho Web
String webLog = "System Started...";

// ================= ESP-NOW CALLBACKS =================
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingDataPtr, int len) {
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
    currentStatus = incomingData.status;
    currentColor = incomingData.cargoColor;
    rpmL = incomingData.rpmLeft;
    rpmR = incomingData.rpmRight;
    
    // Cập nhật sensor
    sens[0] = incomingData.s1;
    sens[1] = incomingData.s2;
    sens[2] = incomingData.s3;
    sens[3] = incomingData.s4;
    sens[4] = incomingData.s5;
  }
}

void addToLog(String msg) {
  // Giới hạn độ dài log để tránh tràn bộ nhớ
  if (webLog.length() > 1000) webLog = ""; 
  String timeStr = String(millis() / 1000) + "s: ";
  webLog = timeStr + msg + "\n" + webLog; 
}

void sendCommand(char c) {
  cmdToSend.cmd = c;
  esp_err_t result = esp_now_send(robotAddress, (uint8_t *) &cmdToSend, sizeof(cmdToSend));
  if (result == ESP_OK) {
    Serial.println(">> Gửi lệnh OK");
    addToLog("Sent CMD: " + String(c) + " (Success)");
  }
  else {
    Serial.println(">> Gửi lệnh LỖI");
    addToLog("Sent CMD: " + String(c) + " (FAILED)");
  }
}

// ================= WEB HANDLERS (GIAO DIỆN MỚI) =================

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  // CSS Reset & Base
  html += "body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 10px; background: #e9ecef; color: #333; }";
  html += "h2 { text-align: center; margin-bottom: 5px; color: #495057; }";
  
  // Card Design
  html += ".container { max-width: 600px; margin: 0 auto; }";
  html += ".card { background: white; padding: 15px; margin-bottom: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  html += ".card-title { font-weight: bold; margin-bottom: 10px; border-bottom: 1px solid #eee; padding-bottom: 5px; }";
  
  // Buttons
  html += ".btn-group { display: flex; gap: 10px; justify-content: center; }";
  html += ".btn { padding: 8px 16px; font-size: 14px; color: white; border: none; border-radius: 4px; cursor: pointer; text-decoration: none; text-align: center; width: 100px; }";
  html += ".btn-green { background-color: #28a745; } .btn-green:hover { background-color: #218838; }";
  html += ".btn-red { background-color: #dc3545; } .btn-red:hover { background-color: #c82333; }";

  // Status List
  html += ".status-list { list-style: none; padding: 0; margin: 0; display: grid; grid-template-columns: 1fr 1fr; gap: 5px; }";
  html += ".status-item { font-size: 12px; padding: 5px; background: #f8f9fa; color: #adb5bd; border-radius: 4px; text-align: center; border: 1px solid #dee2e6; }";
  html += ".status-active { background-color: #d4edda; color: #155724; border-color: #c3e6cb; font-weight: bold; }";

  // Dashboard Grid (RPM & Color)
  html += ".dash-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }";
  html += ".dash-val { font-size: 1.5em; font-weight: bold; text-align: center; }";
  html += ".color-box { width: 40px; height: 40px; border-radius: 50%; margin: 0 auto; border: 2px solid #ddd; }";

  // Sensor Bars
  html += ".sensor-box { display: flex; justify-content: space-between; align-items: flex-end; height: 100px; padding-top: 10px; border-top: 1px dashed #eee; }";
  html += ".bar-container { display: flex; flex-direction: column; align-items: center; width: 18%; }";
  html += ".bar { width: 100%; background-color: #007bff; transition: height 0.3s ease; border-radius: 3px 3px 0 0; }";
  html += ".bar-val { font-size: 10px; margin-top: 2px; }";

  // Console Log
  html += "#log-box { width: 100%; height: 100px; font-family: monospace; font-size: 12px; background: #333; color: #0f0; border: none; padding: 5px; box-sizing: border-box; resize: none; }";
  
  html += "</style></head><body>";
  
  html += "<div class='container'>";
  html += "<h2>ROBOT CONTROL</h2>";
  
  // 1. Buttons
  html += "<div class='card'>";
  html += "<div class='btn-group'>";
  html += "<a href='/start' class='btn btn-green'>START</a>";
  html += "<a href='/stop' class='btn btn-red'>STOP</a>";
  html += "</div></div>";

  // 2. Status List
  html += "<div class='card'>";
  html += "<div class='card-title'>Mission Status</div>";
  html += "<ul class='status-list'>";
  html += "<li id='st0' class='status-item'>0. Wait Start</li>";
  html += "<li id='st1' class='status-item'>1. To Loading</li>";
  html += "<li id='st2' class='status-item'>2. At Loading</li>";
  html += "<li id='st3' class='status-item'>3. To Intersec</li>";
  html += "<li id='st4' class='status-item'>4. To End</li>";
  html += "<li id='st5' class='status-item'>5. Finished</li>";
  html += "<li id='st6' class='status-item'>6. STOPPED</li>";
  html += "</ul></div>";

  // 3. RPM & Color
  html += "<div class='dash-grid'>";
  // RPM Card
  html += "<div class='card'>";
  html += "<div class='card-title'>Speed (RPM)</div>";
  html += "<div class='dash-val' id='rpm'>0 / 0</div>";
  html += "</div>";
  // Color Card
  html += "<div class='card'>";
  html += "<div class='card-title'>Cargo</div>";
  html += "<div id='color_disp' class='color-box' style='background:#eee;'></div>";
  html += "<div style='text-align:center; margin-top:5px;' id='color_txt'>None</div>";
  html += "</div>";
  html += "</div>"; // End grid

  // 4. Sensors
  html += "<div class='card'>";
  html += "<div class='card-title'>Line Sensors</div>";
  html += "<div class='sensor-box'>";
  for(int i=0; i<5; i++) {
    html += "<div class='bar-container'>";
    html += "<div id='bar" + String(i) + "' class='bar' style='height: 10%;'></div>";
    html += "<div id='val" + String(i) + "' class='bar-val'>0</div>";
    html += "</div>";
  }
  html += "</div></div>";

  // 5. System Log
  html += "<div class='card'>";
  html += "<div class='card-title'>System Log</div>";
  html += "<textarea id='log-box' readonly>Waiting for connection...</textarea>";
  html += "</div>";

  html += "</div>"; // End container

  // JAVASCRIPT
  html += "<script>";
  html += "setInterval(function() {";
  html += "  fetch('/data').then(res => res.json()).then(data => {";
  
  // Update Status Class
  html += "    for(let i=0; i<=6; i++) {";
  html += "      let el = document.getElementById('st' + i);";
  html += "      if(i == data.status_code) el.classList.add('status-active');";
  html += "      else el.classList.remove('status-active');";
  html += "    }";

  // Update RPM & Color
  html += "    document.getElementById('rpm').innerText = data.rpm;";
  html += "    document.getElementById('color_txt').innerText = data.color_name;";
  html += "    let cDiv = document.getElementById('color_disp');";
  html += "    if(data.color_code == 1) cDiv.style.background = '#dc3545';"; // Red
  html += "    else if(data.color_code == 2) cDiv.style.background = '#007bff';"; // Blue
  html += "    else cDiv.style.background = '#eee';";

  // Update Sensors
  html += "    for(let i=0; i<5; i++) {";
  html += "       let val = data.sensors[i];";
  html += "       let h = (val / 1000) * 100;"; // Map 0-1000 to 0-100%
  html += "       if(h>100) h=100;";
  html += "       document.getElementById('bar' + i).style.height = h + '%';";
  html += "       document.getElementById('val' + i).innerText = val;";
  html += "    }";

  // Update Log
  html += "    document.getElementById('log-box').value = data.log;";

  html += "  });";
  html += "}, 200);"; // Refresh rate 200ms
  html += "</script>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleStart() {
  sendCommand('g');
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStop() {
  sendCommand('s');
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleData() {
  String colorName = "None";
  if (currentColor == 1) colorName = "RED";
  if (currentColor == 2) colorName = "BLUE";

  // JSON Builder
  String json = "{";
  json += "\"status_code\": " + String(currentStatus) + ",";
  json += "\"color_name\": \"" + colorName + "\",";
  json += "\"color_code\": " + String(currentColor) + ",";
  json += "\"rpm\": \"" + String((int)rpmL) + " | " + String((int)rpmR) + "\",";
  
  // Sensor Array
  json += "\"sensors\": [";
  for(int i=0; i<5; i++) {
    json += String(sens[i]);
    if(i<4) json += ",";
  }
  json += "],";

  // Log (Cần xử lý ký tự xuống dòng cho JSON an toàn)
  String safeLog = webLog;
  safeLog.replace("\n", "\\n"); 
  safeLog.replace("\r", "");
  json += "\"log\": \"" + safeLog + "\"";
  
  json += "}";
  server.send(200, "application/json", json);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("\n--- MASTER CONTROL UI UPDATE ---");

  // Wifi Channel 1
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ROBOT_CENTER", "12345678", 1, 0, 4); 
  
  Serial.print("Web IP: "); Serial.println(WiFi.softAPIP());
  addToLog("Wifi Started: ROBOT_CENTER");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Fail");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 1; 
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_AP; 
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Add Peer Fail");
    addToLog("Error: Add Peer Fail");
  } else {
    Serial.println("Peer Added");
    addToLog("Robot Connected");
  }

  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Web Server Ready");
}

void loop() {
  server.handleClient();
}