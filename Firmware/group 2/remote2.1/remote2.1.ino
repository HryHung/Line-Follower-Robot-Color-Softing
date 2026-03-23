/*
 * ESP32 MASTER CONTROLLER - FINAL VERSION
 * ======================================================
 * - Wifi: ROBOT_CENTER (Pass: 12345678)
 * - Web: http://192.168.4.1
 * - Update: Giao diện List dọc (Bullet style) + Popup Report
 */

#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>

// ================= CẤU HÌNH =================
// <!!!> THAY MAC CỦA CON ROBOT (SLAVE) VÀO ĐÂY <!!!>
uint8_t robotAddress[] = {0x80, 0xF3, 0xDA, 0xAC, 0x60, 0x54};

WebServer server(80);

// --- STRUCT GỬI NHẬN (KHỚP VỚI SLAVE) ---
typedef struct struct_telemetry {
  int status;       
  int cargoColor;   
  float rpmLeft;
  float rpmRight;
  float errorLine;
  int s1; int s2; int s3; int s4; int s5;
  // Các biến kết quả
  float biggestError;
  unsigned long travelTime;
} struct_telemetry;

typedef struct struct_command {
  char cmd; 
} struct_command;

struct_telemetry incomingData;
struct_command cmdToSend;

// --- BIẾN TOÀN CỤC LƯU TRỮ DỮ LIỆU ---
volatile int currentStatus = 0;
volatile int currentColor = 0;
volatile float rpmL = 0, rpmR = 0;
volatile int sens[5] = {0, 0, 0, 0, 0};
volatile float maxErr = 0;
volatile unsigned long missionTime = 0;

String webLog = "System Ready...\n";

// ================= CÁC FUNCTION XỬ LÝ PHỤ TRỢ =================

// 1. Ghi log để hiển thị lên web
void addToLog(String msg) {
  if (webLog.length() > 800) webLog = ""; // Xóa bớt nếu quá dài
  webLog = String(millis()/1000) + "s: " + msg + "\n" + webLog; 
}

// 2. Callback khi nhận dữ liệu từ ESP-NOW
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingDataPtr, int len) {
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
    
    currentStatus = incomingData.status;
    currentColor = incomingData.cargoColor;
    rpmL = incomingData.rpmLeft;
    rpmR = incomingData.rpmRight;
    
    sens[0] = incomingData.s1; 
    sens[1] = incomingData.s2; 
    sens[2] = incomingData.s3;
    sens[3] = incomingData.s4; 
    sens[4] = incomingData.s5;

    maxErr = incomingData.biggestError;
    missionTime = incomingData.travelTime;
  }
}

// 3. Gửi lệnh đi
void sendCommand(char c) {
  cmdToSend.cmd = c;
  esp_err_t result = esp_now_send(robotAddress, (uint8_t *) &cmdToSend, sizeof(cmdToSend));
  if (result == ESP_OK) addToLog("Sent CMD: " + String(c));
  else addToLog("Send CMD Error");
}

// ================= WEB SERVER HANDLERS =================

// 4. API trả về JSON dữ liệu (Cho Javascript fetch)
void handleData() {
  String colorName = "None";
  if (currentColor == 1) colorName = "RED";
  if (currentColor == 2) colorName = "BLUE";

  String json = "{";
  json += "\"status_code\": " + String(currentStatus) + ",";
  json += "\"color_name\": \"" + colorName + "\",";
  json += "\"color_code\": " + String(currentColor) + ",";
  json += "\"rpm\": \"" + String((int)rpmL) + " | " + String((int)rpmR) + "\",";
  
  json += "\"sensors\": [";
  for(int i=0; i<5; i++) {
    json += String(sens[i]);
    if(i<4) json += ",";
  }
  json += "],";

  // Dữ liệu cho Popup
  json += "\"max_err\": " + String(maxErr) + ",";
  json += "\"time\": " + String(missionTime) + ",";

  // Xử lý chuỗi log để không lỗi JSON
  String safeLog = webLog;
  safeLog.replace("\n", "\\n"); 
  safeLog.replace("\r", "");
  json += "\"log\": \"" + safeLog + "\"";
  
  json += "}";
  server.send(200, "application/json", json);
}

// 5. API nhận lệnh từ nút bấm
void handleCmd() {
  if (server.hasArg("c")) {
    char c = server.arg("c").charAt(0);
    sendCommand(c);
  }
  server.send(200, "text/plain", "OK");
}

// 6. GIAO DIỆN CHÍNH (CODE CỦA BẠN ĐÃ ĐƯỢC CHÈN VÀO ĐÂY)
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  // --- CSS CHUNG ---
  html += "body { font-family: 'Segoe UI', sans-serif; margin: 0; padding: 10px; background: #e9ecef; color: #333; }";
  html += ".container { max-width: 600px; margin: 0 auto; }";
  html += ".card { background: white; padding: 15px; margin-bottom: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  html += ".card-title { font-weight: bold; margin-bottom: 10px; border-bottom: 1px solid #eee; padding-bottom: 5px; }";
  
  // --- BUTTONS ---
  html += ".btn-group { display: flex; gap: 10px; justify-content: center; }";
  html += ".btn { padding: 10px 20px; font-size: 14px; color: white; border: none; border-radius: 4px; cursor: pointer; text-decoration: none; width: 100px; text-align: center; }";
  html += ".btn-green { background-color: #28a745; }";
  html += ".btn-red { background-color: #dc3545; }";

  // --- SỬA ĐỔI: STATUS LIST STYLE (LIST DỌC) ---
  html += ".status-list { list-style: none; padding: 0; margin: 0; }";
  html += ".status-item { padding: 8px 10px; border-bottom: 1px dashed #eee; display: flex; align-items: center; transition: all 0.3s; font-size: 14px; }";
  
  // 1. Đã hoàn thành (Done) - Màu xanh
  html += ".st-done { color: #28a745; }";
  html += ".st-done::before { content: '✔'; margin-right: 10px; font-weight: bold; }"; // Thêm dấu tick
  
  // 2. Đang thực hiện (Active) - Màu xanh đậm, chữ to
  html += ".st-active { color: #1e7e34; font-weight: bold; font-size: 16px; background: #f0fff4; border-radius: 4px; border-left: 4px solid #28a745; }";
  html += ".st-active::before { content: '➤'; margin-right: 10px; }"; // Thêm mũi tên
  
  // 3. Chưa thực hiện (Pending) - Màu xám
  html += ".st-pending { color: #adb5bd; }";
  html += ".st-pending::before { content: '•'; margin-right: 10px; font-size: 20px; }"; // Dấu chấm tròn

  // --- DASHBOARD & SENSORS ---
  html += ".dash-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }";
  html += ".dash-val { font-size: 1.5em; font-weight: bold; text-align: center; }";
  html += ".color-box { width: 40px; height: 40px; border-radius: 50%; margin: 0 auto; border: 2px solid #ddd; }";
  html += ".sensor-box { display: flex; justify-content: space-between; align-items: flex-end; height: 100px; padding-top: 10px; }";
  html += ".bar-container { height: 100%; display: flex; flex-direction: column; justify-content: flex-end; align-items: center; width: 18%; }";
  html += ".bar { width: 100%; background-color: #77DD77; transition: height 0.3s ease; border-radius: 3px 3px 0 0; }";
  html += ".bar-val { font-size: 10px; margin-top: 2px; }";
  html += "#log-box { width: 100%; height: 80px; font-family: monospace; font-size: 11px; background: #333; color: #0f0; border: none; padding: 5px; resize: none; }";

  // --- MODAL POPUP ---
  html += ".modal { display: none; position: fixed; z-index: 1000; left: 0; top: 0; width: 100%; height: 100%; background-color: rgba(0,0,0,0.6); backdrop-filter: blur(5px); }";
  html += ".modal-content { background-color: #fff; margin: 30% auto; padding: 0; border-radius: 12px; width: 85%; max-width: 350px; box-shadow: 0 5px 15px rgba(0,0,0,0.3); overflow: hidden; }";
  html += ".modal-header { background: #28a745; color: white; padding: 15px; text-align: center; font-size: 18px; font-weight: bold; }";
  html += ".modal-body { padding: 20px; text-align: center; }";
  html += ".result-val { font-size: 24px; font-weight: bold; color: #333; margin: 5px 0 15px 0; }";
  html += ".btn-close { background: #6c757d; color: white; border: none; padding: 8px 20px; border-radius: 4px; width: 100%; font-size: 14px; }";

  html += "</style></head><body>";
  
  // --- HTML POPUP ---
  html += "<div id='resultModal' class='modal'><div class='modal-content'>";
  html += "<div class='modal-header'>MISSION COMPLETE!</div>";
  html += "<div class='modal-body'>";
  html += "<div>Travel Time</div><div id='res_time' class='result-val'>0.00s</div>";
  html += "<div>Max Deviation</div><div id='res_err' class='result-val'>0.0</div>";
  html += "<button class='btn-close' onclick='closeModal()'>CLOSE</button>";
  html += "</div></div></div>";

  // --- MAIN CONTENT ---
  html += "<div class='container'>";
  html += "<h2>ROBOT CONTROL</h2>";
  
  html += "<div class='card'><div class='btn-group'>";
  html += "<a href='#' onclick='sendCmd(\"g\")' class='btn btn-green'>START</a>";
  html += "<a href='#' onclick='sendCmd(\"s\")' class='btn btn-red'>STOP</a>";
  html += "</div></div>";

  // --- LIST STATUS (ĐÃ SỬA) ---
  html += "<div class='card'><div class='card-title'>Mission Progress</div>";
  html += "<ul class='status-list'>";
  html += "<li id='st0' class='status-item'>0. Waiting Start</li>";
  html += "<li id='st1' class='status-item'>1. Moving to Loading</li>";
  html += "<li id='st2' class='status-item'>2. At Loading Zone</li>";
  html += "<li id='st3' class='status-item'>3. Moving to Intersection</li>";
  html += "<li id='st4' class='status-item'>4. Moving to End</li>";
  html += "<li id='st5' class='status-item'>5. Finished</li>";
  html += "<li id='st6' class='status-item'>6. EMERGENCY STOP</li>";
  html += "</ul></div>";

  // --- RPM & COLOR ---
  html += "<div class='dash-grid'>";
  html += "<div class='card'><div class='card-title'>Speed (RPM)</div><div class='dash-val' id='rpm'>0 | 0</div></div>";
  html += "<div class='card'><div class='card-title'>Cargo</div><div id='color_disp' class='color-box' style='background:#eee;'></div><div style='text-align:center;margin-top:5px;' id='color_txt'>None</div></div>";
  html += "</div>";

  // --- SENSORS ---
  html += "<div class='card'><div class='card-title'>Sensors</div><div class='sensor-box'>";
  for(int i=0; i<5; i++) {
    html += "<div class='bar-container'><div id='bar" + String(i) + "' class='bar' style='height: 10%;'></div><div id='val" + String(i) + "' class='bar-val'>0</div></div>";
  }
  html += "</div></div>";

  html += "<div class='card'><div class='card-title'>Log</div><textarea id='log-box' readonly>Waiting...</textarea></div>";
  html += "</div>"; // End container

  // --- JAVASCRIPT ---
  html += "<script>";
  html += "let modalShown = false;";
  html += "function sendCmd(c) { fetch('/cmd?c='+c); modalShown = false; }";
  html += "function closeModal() { document.getElementById('resultModal').style.display = 'none'; }";

  html += "setInterval(function() {";
  html += "  fetch('/data').then(res => res.json()).then(data => {";
  
  // --- LOGIC MÀU STATUS (SỬA ĐỔI) ---
  html += "    for(let i=0; i<=6; i++) {";
  html += "      let el = document.getElementById('st' + i);";
  html += "      el.className = 'status-item';"; // Reset class về mặc định
  html += "      if (i < data.status_code) el.classList.add('st-done');";        // Đã xong -> Xanh lá
  html += "      else if (i == data.status_code) el.classList.add('st-active');"; // Đang chạy -> Active
  html += "      else el.classList.add('st-pending');";                           // Chưa tới -> Xám
  html += "    }";

  html += "    document.getElementById('rpm').innerText = data.rpm;";
  html += "    document.getElementById('color_txt').innerText = data.color_name;";
  html += "    let cDiv = document.getElementById('color_disp');";
  html += "    if(data.color_code == 1) cDiv.style.background = '#dc3545';"; 
  html += "    else if(data.color_code == 2) cDiv.style.background = '#007bff';";
  html += "    else cDiv.style.background = '#eee';";

  html += "    for(let i=0; i<5; i++) {";
  html += "       let val = data.sensors[i];";
  html += "       let h = (val / 1000) * 100; if(h>100) h=100;";
  html += "       document.getElementById('bar' + i).style.height = h + '%';";
  html += "       document.getElementById('val' + i).innerText = val;";
  html += "    }";

  html += "    document.getElementById('log-box').value = data.log;";

  html += "    if (data.status_code == 5 && !modalShown) {";
  html += "       document.getElementById('res_time').innerText = (data.time / 1000).toFixed(2) + 's';";
  html += "       document.getElementById('res_err').innerText = data.max_err.toFixed(1);";
  html += "       document.getElementById('resultModal').style.display = 'block';";
  html += "       modalShown = true;";
  html += "    }";

  html += "  });";
  html += "}, 200);"; 
  html += "</script></body></html>";
  
  server.send(200, "text/html", html);
}

// ================= SETUP & LOOP =================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. Setup WiFi AP (Phát Wifi)
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ROBOT_CENTER", "12345678", 1, 0, 4); 
  Serial.print("Web Server IP: "); Serial.println(WiFi.softAPIP());

  // 2. Setup ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Fail");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // 3. Add Peer (Robot)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 1; 
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_AP; 
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer Added Successfully");
  }

  // 4. Setup Web Server
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/cmd", handleCmd);
  server.begin();
  Serial.println("Web Server Started");
}

void loop() {
  server.handleClient();
}