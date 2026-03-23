/*
 * ESP32 MASTER CONTROLLER - FINAL (FIXED RPM GRAPH)
 * =================================================
 * - Update: Biểu đồ RPM có trục Vận tốc (Y) và Thời gian (X)
 * - Style: Giữ nguyên cấu trúc String html += của bạn
 */

#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>

// ================= CẤU HÌNH =================
uint8_t robotAddress[] = {0x80, 0xF3, 0xDA, 0xAC, 0x60, 0x54};

WebServer server(80);

// --- STRUCT GỬI NHẬN ---
typedef struct struct_telemetry {
  int status;       
  int cargoColor;   
  float rpmLeft;
  float rpmRight;
  float errorLine;      
  float err_center;     
  int s1; int s2; int s3; int s4; int s5;
  float biggestError;
  unsigned long travelTime;
} struct_telemetry;

typedef struct struct_command {
  char cmd; 
} struct_command;

struct_telemetry incomingData;
struct_command cmdToSend;

// --- BIẾN TOÀN CỤC ---
volatile int currentStatus = 0;
volatile int currentColor = 0;
volatile float rpmL = 0, rpmR = 0;
volatile float errLine = 0;      
volatile float errCenter = 0;    
volatile int sens[5] = {0, 0, 0, 0, 0};
volatile float maxErr = 0;
volatile unsigned long missionTime = 0;

String webLog = "System Ready...\n";

// ================= FUNCTION XỬ LÝ =================

void addToLog(String msg) {
  if (webLog.length() > 800) webLog = ""; 
  webLog = String(millis()/1000) + "s: " + msg + "\n" + webLog; 
}

void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingDataPtr, int len) {
  if (len == sizeof(incomingData)) {
    memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
    
    currentStatus = incomingData.status;
    currentColor = incomingData.cargoColor;
    rpmL = incomingData.rpmLeft;
    rpmR = incomingData.rpmRight;
    
    errLine = incomingData.errorLine;     
    errCenter = incomingData.err_center;  

    sens[0] = incomingData.s1; 
    sens[1] = incomingData.s2; 
    sens[2] = incomingData.s3;
    sens[3] = incomingData.s4; 
    sens[4] = incomingData.s5;

    maxErr = incomingData.biggestError;
    missionTime = incomingData.travelTime;
  }
}

void sendCommand(char c) {
  cmdToSend.cmd = c;
  esp_err_t result = esp_now_send(robotAddress, (uint8_t *) &cmdToSend, sizeof(cmdToSend));
  if (result == ESP_OK) addToLog("Sent CMD: " + String(c));
  else addToLog("Send CMD Error");
}

// ================= WEB SERVER (ĐÃ SỬA) =================

void handleData() {
  String colorName = "None";
  if (currentColor == 1) colorName = "RED";
  if (currentColor == 2) colorName = "BLUE";

  String json = "{";
  json += "\"status_code\": " + String(currentStatus) + ",";
  json += "\"color_name\": \"" + colorName + "\","; 
  json += "\"color_code\": " + String(currentColor) + ",";
  
  // --- [SỬA] Tách RPM để vẽ đồ thị ---
  json += "\"r1\": " + String((int)rpmL) + ","; // RPM Trái
  json += "\"r2\": " + String((int)rpmR) + ","; // RPM Phải
  // ----------------------------------
json += "\"rpm\": \"" + String((int)rpmL) + " | " + String((int)rpmR) + "\","; // Giữ cái cũ cho chắc
  json += "\"err_line\": " + String(errLine) + ",";
  json += "\"err_center\": " + String(errCenter) + ",";

  json += "\"sensors\": [";
  for(int i=0; i<5; i++) {
    json += String(sens[i]);
    if(i<4) json += ",";
  }
  json += "],";

  json += "\"max_err\": " + String(maxErr) + ",";
  json += "\"time\": " + String(missionTime) + ",";

  String safeLog = webLog;
  safeLog.replace("\n", "\\n"); 
  safeLog.replace("\r", "");
  json += "\"log\": \"" + safeLog + "\"";
  
  json += "}";
  server.send(200, "application/json", json);
}

void handleCmd() {
  if (server.hasArg("c")) {
    char c = server.arg("c").charAt(0);
    sendCommand(c);
  }
  server.send(200, "text/plain", "OK");
}

// ================= WEB SERVER (FINAL UPDATE - DEVIATION GRAPH) =================

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: 'Segoe UI', sans-serif; margin: 0; padding: 10px; background: #e9ecef; color: #333; }";
  html += ".container { max-width: 600px; margin: 0 auto; }";
  html += ".card { background: white; padding: 15px; margin-bottom: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
  html += ".card-title { font-weight: bold; margin-bottom: 10px; border-bottom: 1px solid #eee; padding-bottom: 5px; }";
  html += ".btn-group { display: flex; gap: 10px; justify-content: center; }";
  html += ".btn { padding: 10px 20px; font-size: 14px; color: white; border: none; border-radius: 4px; cursor: pointer; text-decoration: none; width: 100px; text-align: center; }";
  html += ".btn-green { background-color: #28a745; }";
  html += ".btn-red { background-color: #dc3545; }";
  html += ".status-list { list-style: none; padding: 0; margin: 0; }";
  html += ".status-item { padding: 8px 10px; border-bottom: 1px dashed #eee; display: flex; align-items: center; transition: all 0.3s; font-size: 14px; }";
  html += ".st-done { color: #28a745; } .st-done::before { content: '✔'; margin-right: 10px; font-weight: bold; }";
  html += ".st-active { color: #1e7e34; font-weight: bold; font-size: 16px; background: #f0fff4; border-radius: 4px; border-left: 4px solid #28a745; }";
  html += ".st-active::before { content: '➤'; margin-right: 10px; }";
  html += ".st-pending { color: #adb5bd; } .st-pending::before { content: '•'; margin-right: 10px; font-size: 20px; }";
  html += ".dash-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }";
  html += ".dash-val { font-size: 1.5em; font-weight: bold; text-align: center; }";
  html += ".color-box { width: 40px; height: 40px; border-radius: 50%; margin: 0 auto; border: 2px solid #ddd; }";
  html += ".sensor-box { display: flex; justify-content: space-between; align-items: flex-end; height: 100px; padding-top: 10px; }";
html += ".bar-container { height: 100%; display: flex; flex-direction: column; justify-content: flex-end; align-items: center; width: 18%; }";
  html += ".bar { width: 100%; background-color: #77DD77; transition: height 0.3s ease; border-radius: 3px 3px 0 0; }";
  html += ".bar-val { font-size: 10px; margin-top: 2px; }";
  html += "#log-box { width: 100%; height: 80px; font-family: monospace; font-size: 11px; background: #333; color: #0f0; border: none; padding: 5px; resize: none; }";
  html += ".modal { display: none; position: fixed; z-index: 1000; left: 0; top: 0; width: 100%; height: 100%; background-color: rgba(0,0,0,0.6); backdrop-filter: blur(5px); }";
  html += ".modal-content { background-color: #fff; margin: 30% auto; padding: 0; border-radius: 12px; width: 85%; max-width: 350px; box-shadow: 0 5px 15px rgba(0,0,0,0.3); overflow: hidden; }";
  html += ".modal-header { background: #28a745; color: white; padding: 15px; text-align: center; font-size: 18px; font-weight: bold; }";
  html += ".modal-body { padding: 20px; text-align: center; }";
  html += ".result-val { font-size: 24px; font-weight: bold; color: #333; margin: 5px 0 15px 0; }";
  html += ".btn-close { background: #6c757d; color: white; border: none; padding: 8px 20px; border-radius: 4px; width: 100%; font-size: 14px; }";
  html += ".dual-val { display: flex; justify-content: space-around; margin-bottom: 5px; }";
  html += ".lbl-small { font-size: 10px; color: #666; text-align: right; margin-bottom: 5px; padding-right: 5px;}";
  
  html += "canvas { background: #fafafa; border: 1px solid #eee; display: block; width: 100%; }";
  
  html += "</style></head><body>";
  
  html += "<div id='resultModal' class='modal'><div class='modal-content'>";
  html += "<div class='modal-header'>MISSION COMPLETE!</div>";
  html += "<div class='modal-body'>";
  html += "<div>Travel Time</div><div id='res_time' class='result-val'>0.00s</div>";
  html += "<div>Max Deviation</div><div id='res_err' class='result-val'>0.0</div>";
  html += "<button class='btn-close' onclick='closeModal()'>CLOSE</button>";
  html += "</div></div></div>";
  
  html += "<div class='container'>";
  html += "<h2>ROBOT CONTROL</h2>";
  html += "<div class='card'><div class='btn-group'>";
  html += "<a href='#' onclick='sendCmd(\"g\")' class='btn btn-green'>START</a>";
  html += "<a href='#' onclick='sendCmd(\"s\")' class='btn btn-red'>STOP</a>";
  html += "</div></div>";

  html += "<div class='card'><div class='card-title'>Mission Progress</div><ul class='status-list'>";
  html += "<li id='st0' class='status-item'>0. Waiting Start</li>";
  html += "<li id='st1' class='status-item'>1. Moving to Loading</li>";
  html += "<li id='st2' class='status-item'>2. At Loading Zone</li>";
  html += "<li id='st3' class='status-item'>3. Moving to Intersection</li>";
  html += "<li id='st4' class='status-item'>4. Moving to End</li>";
  html += "<li id='st5' class='status-item'>5. Finished</li>";
html += "<li id='st6' class='status-item'>6. EMERGENCY STOP</li>";
  html += "</ul></div>";

  // --- CHART RPM (No Change) ---
  html += "<div class='card'><div class='card-title'>Velocity Graph (RPM)</div>";
  html += "<div class='dual-val'>";
  html += "<div>Left: <b id='t_r1' style='color:#dc3545'>0</b></div>";  
  html += "<div>Right: <b id='t_r2' style='color:#007bff'>0</b></div>";
  html += "</div>";
  html += "<canvas id='chartRPM' width='300' height='120'></canvas>"; 
  html += "<div class='lbl-small'>Range: -50 to 250 RPM</div>";
  html += "</div>";

  // --- [UPDATED] CHART DEVIATION (Height 60 -> 120) ---
  html += "<div class='card'><div class='card-title'>Deviation Monitor (mm)</div>";
  html += "<div class='dual-val'>";
  html += "<div>Err Sensor: <b id='txt_err_line' style='color:#dc3545'>0.0</b></div>";
  html += "<div>Err Center: <b id='txt_err_center' style='color:#28a745'>0.0</b></div>";
  html += "</div>";
  // Tăng chiều cao lên 120 để vẽ rõ
  html += "<canvas id='chartLine' width='300' height='120'></canvas>";
  html += "<div class='lbl-small'>Grid: 10mm/div | Range: +/- 30mm</div>";
  html += "</div>";

  html += "<div class='dash-grid'>";
  html += "<div class='card'><div class='card-title'>Sensors</div><div class='sensor-box'>";
  for(int i=0; i<5; i++) {
    html += "<div class='bar-container'><div id='bar" + String(i) + "' class='bar' style='height: 10%;'></div><div id='val" + String(i) + "' class='bar-val'>0</div></div>";
  }
  html += "</div></div>";
  
  html += "<div class='card'><div class='card-title'>Cargo</div><div id='color_disp' class='color-box' style='background:#eee;'></div><div style='text-align:center;margin-top:5px;' id='color_txt'>None</div></div>";
  html += "</div>";

  html += "<div class='card'><div class='card-title'>Log</div><textarea id='log-box' readonly>Waiting...</textarea></div>";
  html += "</div>";

  // --- JAVASCRIPT ---
  html += "<script>\n"; 
  html += "let modalShown = false;\n";
  html += "let fullLineData = []; let fullCenterData = [];\n";
  html += "let fullRpmL = []; let fullRpmR = [];\n"; 

  html += "function sendCmd(c) { \n";
  html += "  if(c=='g') { fullLineData = []; fullCenterData = []; fullRpmL = []; fullRpmR = []; } \n";
  html += "  fetch('/cmd?c='+c); modalShown = false; \n";
  html += "}\n";
  
  html += "function closeModal() { document.getElementById('resultModal').style.display = 'none'; }\n";

  // --- HÀM VẼ RPM (GIỮ NGUYÊN) ---
  html += "function drawRPMGraph(id, dataL, dataR) {\n";
  html += "  let c = document.getElementById(id); let ctx = c.getContext('2d');\n";
  html += "  let w = c.width, h = c.height;\n";
  html += "  let padL = 30; let padB = 20; \n"; 
  html += "  ctx.clearRect(0,0,w,h);\n";
  html += "  let minVal = -50; let maxVal = 250; let range = maxVal - minVal; let drawH = h - padB;\n";
  html += "  ctx.strokeStyle = '#333'; ctx.lineWidth = 1; ctx.beginPath();\n";
html += "  ctx.moveTo(padL, 0); ctx.lineTo(padL, drawH); ctx.lineTo(w, drawH); ctx.stroke(); \n"; 
  html += "  ctx.font = '10px Arial'; ctx.textAlign = 'right';\n";
  html += "  for(let v = minVal; v <= maxVal; v += 50) {\n";
  html += "    let y = drawH - ((v - minVal) / range) * drawH;\n";
  html += "    ctx.fillStyle = '#666'; ctx.fillText(v, padL - 5, y + 3);\n";
  html += "    ctx.beginPath(); \n";
  html += "    if(v === 0) { ctx.strokeStyle = '#000'; ctx.lineWidth = 1.5; } \n";
  html += "    else { ctx.strokeStyle = '#eee'; ctx.lineWidth = 1; }\n";
  html += "    ctx.moveTo(padL, y); ctx.lineTo(w, y); ctx.stroke();\n";
  html += "  }\n";
  html += "  let len = dataL.length; let stepX = (w - padL) / (len > 1 ? len-1 : 1);\n";
  html += "  ctx.beginPath(); ctx.strokeStyle = '#dc3545'; ctx.lineWidth = 1.5;\n";
  html += "  for(let i=0; i<len; i++){\n";
  html += "    let v = dataL[i]; if(v > maxVal) v=maxVal; if(v < minVal) v=minVal;\n";
  html += "    let y = drawH - ((v - minVal) / range) * drawH;\n";
  html += "    let x = padL + i * stepX;\n";
  html += "    if(i==0) ctx.moveTo(x, y); else ctx.lineTo(x, y);\n";
  html += "  }\n";
  html += "  ctx.stroke();\n";
  html += "  ctx.beginPath(); ctx.strokeStyle = '#007bff'; ctx.lineWidth = 1.5;\n";
  html += "  for(let i=0; i<len; i++){\n";
  html += "    let v = dataR[i]; if(v > maxVal) v=maxVal; if(v < minVal) v=minVal;\n";
  html += "    let y = drawH - ((v - minVal) / range) * drawH;\n";
  html += "    let x = padL + i * stepX;\n";
  html += "    if(i==0) ctx.moveTo(x, y); else ctx.lineTo(x, y);\n";
  html += "  }\n";
  html += "  ctx.stroke();\n";
  html += "}\n";

  // --- [UPDATED] HÀM VẼ DEVIATION (CÓ LƯỚI & SỐ) ---
  html += "function drawDualChart(id, data1, data2, color1, color2) {\n";
  html += "  let c = document.getElementById(id); let ctx = c.getContext('2d');\n";
  html += "  let w = c.width, h = c.height;\n";
  html += "  let padL = 30; \n"; // Lề trái cho số
  html += "  ctx.clearRect(0,0,w,h);\n";

  // 1. Vẽ Lưới (Grid) & Số (-30, -20... +30)
  html += "  ctx.font = '10px Arial'; ctx.textAlign = 'right';\n";
  html += "  // Loop từ -30 đến 30, bước nhảy 10mm\n";
  html += "  for(let v = -30; v <= 30; v += 10) {\n";
  html += "     // Map -30..30 vào chiều cao Canvas\n";
  html += "     let y = (h/2) - (v / 30) * (h/2);\n";
  html += "     \n";
  html += "     // Vẽ số\n";
  html += "     ctx.fillStyle = '#888'; ctx.fillText(v, padL - 4, y + 3);\n";
  html += "     \n";
  html += "     // Vẽ đường ngang\n";
  html += "     ctx.beginPath();\n";
  html += "     if(v === 0) { ctx.strokeStyle = '#000'; ctx.lineWidth = 1.5; } // Đậm ở giữa\n";
  html += "     else { ctx.strokeStyle = '#eee'; ctx.lineWidth = 1; }\n";
  html += "     ctx.moveTo(padL, y); ctx.lineTo(w, y); ctx.stroke();\n";
  html += "  }\n";

  // 2. Vẽ Trục dọc\n";
  html += "  ctx.beginPath(); ctx.strokeStyle='#333'; ctx.lineWidth=1; \n";
html += "  ctx.moveTo(padL, 0); ctx.lineTo(padL, h); ctx.stroke();\n";

  // 3. Vẽ Line 1 (Sensor - Red)\n";
  html += "  let len = data1.length; let step = (w-padL) / (len > 1 ? len - 1 : 1);\n";
  html += "  ctx.beginPath(); ctx.strokeStyle = color1; ctx.lineWidth = 1;\n";
  html += "  for(let i=0; i<len; i++){\n";
  html += "    let v = data1[i]; if(v > 30) v=30; if(v < -30) v=-30; // Clamp\n";
  html += "    let y = (h/2) - (v / 30) * (h/2);\n";
  html += "    if(i==0) ctx.moveTo(padL, y); else ctx.lineTo(padL + i*step, y);\n";
  html += "  }\n";
  html += "  ctx.stroke();\n";

  // 4. Vẽ Line 2 (Center - Green)\n";
  html += "  ctx.beginPath(); ctx.strokeStyle = color2; ctx.lineWidth = 1.5; // Đậm hơn chút\n";
  html += "  for(let i=0; i<len; i++){\n";
  html += "    let v = data2[i]; if(v > 30) v=30; if(v < -30) v=-30;\n";
  html += "    let y = (h/2) - (v / 30) * (h/2);\n";
  html += "    if(i==0) ctx.moveTo(padL, y); else ctx.lineTo(padL + i*step, y);\n";
  html += "  }\n";
  html += "  ctx.stroke();\n";
  html += "}\n";

  html += "setInterval(function() {\n";
  html += "  fetch('/data').then(res => res.json()).then(data => {\n";
  html += "    for(let i=0; i<=6; i++) {\n";
  html += "      let el = document.getElementById('st' + i);\n";
  html += "      el.className = 'status-item';\n";
  html += "      if (i < data.status_code) el.classList.add('st-done');\n";
  html += "      else if (i == data.status_code) el.classList.add('st-active');\n";
  html += "      else el.classList.add('st-pending');\n";
  html += "    }\n";
  
  html += "    document.getElementById('t_r1').innerText = data.r1;\n";
  html += "    document.getElementById('t_r2').innerText = data.r2;\n";
  html += "    document.getElementById('color_txt').innerText = data.color_name;\n";
  html += "    let cDiv = document.getElementById('color_disp');\n";
  html += "    if(data.color_code == 1) cDiv.style.background = '#dc3545';\n"; 
  html += "    else if(data.color_code == 2) cDiv.style.background = '#007bff';\n";
  html += "    else cDiv.style.background = '#eee';\n";
  html += "    document.getElementById('txt_err_line').innerText = data.err_line.toFixed(1);\n";
  html += "    document.getElementById('txt_err_center').innerText = data.err_center.toFixed(1);\n";
  
  html += "    if(data.status_code >= 1 && data.status_code <= 5) {\n";
  html += "       if(data.status_code != 5 || !modalShown) {\n";
  html += "          fullLineData.push(data.err_line);\n";
  html += "          fullCenterData.push(data.err_center);\n";
  html += "          fullRpmL.push(data.r1); fullRpmR.push(data.r2);\n";
  html += "       }\n";
  html += "    }\n";

  html += "    if (data.status_code == 5) {\n";
  html += "       drawRPMGraph('chartRPM', fullRpmL, fullRpmR);\n"; 
  html += "       drawDualChart('chartLine', fullLineData, fullCenterData, '#dc3545', '#28a745');\n";
  html += "    } else {\n";
  html += "       let slL = fullRpmL.slice(-80); let slR = fullRpmR.slice(-80);\n";
html += "       drawRPMGraph('chartRPM', slL, slR);\n";
  html += "       let sliceLine = fullLineData.slice(-80);\n";
  html += "       let sliceCenter = fullCenterData.slice(-80);\n";
  html += "       drawDualChart('chartLine', sliceLine, sliceCenter, '#dc3545', '#28a745');\n";
  html += "    }\n";

  html += "    for(let i=0; i<5; i++) {\n";
  html += "      let val = data.sensors[i];\n";
  html += "      let h = (val / 1000) * 100; if(h>100) h=100;\n";
  html += "      document.getElementById('bar' + i).style.height = h + '%';\n";
  html += "      document.getElementById('val' + i).innerText = val;\n";
  html += "    }\n";
  html += "    document.getElementById('log-box').value = data.log;\n";
  html += "    if (data.status_code == 5 && !modalShown) {\n";
  html += "       document.getElementById('res_time').innerText = (data.time / 1000).toFixed(2) + 's';\n";
  html += "       document.getElementById('res_err').innerText = data.max_err.toFixed(1);\n";
  html += "       document.getElementById('resultModal').style.display = 'block';\n";
  html += "       modalShown = true;\n";
  html += "    }\n";
  html += "  });\n";
  html += "}, 200);\n"; 
  html += "</script></body></html>";
  
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("ROBOT_CENTER", "12345678", 1, 0, 4); 
  Serial.print("Web Server IP: "); Serial.println(WiFi.softAPIP());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Fail");
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
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer Added Successfully");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/cmd", handleCmd);
  server.begin();
  Serial.println("Web Server Started");
}

void loop() {
  server.handleClient();
}
