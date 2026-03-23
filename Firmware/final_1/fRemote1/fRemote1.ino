/*
 * ESP32 MASTER CONTROLLER - COMPATIBLE WITH KINEMATIC OBSERVER SLAVE
 */

#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>



// ================= CẤU HÌNH =================
// <!!!> THAY MAC CỦA CON ROBOT (SLAVE) VÀO ĐÂY <!!!>
uint8_t robotAddress[] = {0x4C, 0xC3, 0x82, 0xBE, 0xD9, 0xF8}; 

WebServer server(80);

// --- STRUCT GỬI NHẬN ---
typedef struct struct_telemetry {
  int status;       
  int cargoColor;   
  float rpmLeft;
  float rpmRight;
  float errorLine;      
  //float err_center;     
  int s1; int s2; int s3; int s4; int s5;
  //float biggestError;
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
volatile float maxErrCenter = 0;

volatile unsigned long missionTime = 0;


String webLog = "System Ready...\n";

// ====== ERR CENTER (5 POINT KINEMATIC RECONSTRUCTION) ======

const int N_ERR = 5;
float errHist[N_ERR] = {0,0,0,0,0};
float distHist[N_ERR] = {0,0,0,0,0};
bool firstData = true;

const float WHEEL_D = 85.0;         // mm
const float SENSOR_OFFSET = 173.0;  // mm
float last_center = 0;              // filtered center

// Tính quãng đường robot đi thêm mỗi lần nhận telemetry
float computeStep(float rpmL, float rpmR) {
  float v = fabs((rpmL + rpmR) / 2.0);               // RPM
  float v_mm_s = v * (WHEEL_D * 3.14159) / 60.0;     // mm/s
  return v_mm_s * 0.10f;                             // Telemetry 10Hz → dt ≈ 0.1s
}

float computeErrCenter_5P(float errLine, float rpmL, float rpmR) {

  float step = computeStep(rpmL, rpmR);

  // 1) Dịch mảng lịch sử (FILO queue)
  for (int i = N_ERR - 1; i > 0; i--) {
    errHist[i] = errHist[i-1];
    distHist[i] = distHist[i-1] + step;
    if (distHist[i] > 400) distHist[i] = 400;
  }
  errHist[0] = errLine;
  distHist[0] = 0;

  // Chỉ bắt đầu khi đủ dữ liệu
  if (firstData) {
    firstData = false;
    return errLine;
  }

  // 2) Ước lượng đạo hàm bằng Linear Regression: slope = d(err)/ds
  float S = 0, SE = 0, E = 0;
  for (int i = 0; i < N_ERR; i++) {
    float s = distHist[i]; 
    S  += s;
    E  += errHist[i];
    SE += s * errHist[i];
  }

  float mean_s = S / N_ERR;
  float mean_e = E / N_ERR;

  float numerator = 0;
  float denom = 0;

  for (int i = 0; i < N_ERR; i++) {
    float ds = distHist[i] - mean_s;
    float de = errHist[i] - mean_e;
    numerator += ds * de;
    denom += ds * ds;
  }

  float slope = 0;
  if (denom > 1e-6) slope = numerator / denom;

  // slope ≈ tan(theta)
  float raw_center = errLine - SENSOR_OFFSET * slope;

  // 3) Lọc mượt (EMA)
  float alpha = 0.25;
  last_center = alpha * raw_center + (1 - alpha) * last_center;

  return last_center;
}


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

    // Tính err_center theo pipeline đầy đủ
    errCenter = computeErrCenter_5P(errLine, rpmL, rpmR);

    // Record biggest err bằng đúng err_center
    if (fabs(errCenter) > maxErrCenter)
        maxErrCenter = fabs(errCenter);

    sens[0] = incomingData.s1;
    sens[1] = incomingData.s2;
    sens[2] = incomingData.s3;
    sens[3] = incomingData.s4;
    sens[4] = incomingData.s5;

    missionTime = incomingData.travelTime;
  }
}


void sendCommand(char c) {

  // RESET khi START hoặc STOP
  if (c == 'g' || c == 's') {
      maxErrCenter = 0;
      firstData = true;
      last_center = 0;
      for (int i = 0; i < N_ERR; i++) {
          errHist[i] = 0;
          distHist[i] = 0;
      }
  }

  // Gửi lệnh sang Slave
  cmdToSend.cmd = c;
  esp_err_t result = esp_now_send(robotAddress, (uint8_t *) &cmdToSend, sizeof(cmdToSend));

  if (result == ESP_OK) addToLog("Sent CMD: " + String(c));
  else addToLog("Send CMD Error");
}


// ================= WEB SERVER =================

void handleData() {
  String colorName = "None";
  if (currentColor == 1) colorName = "RED";
  if (currentColor == 2) colorName = "BLUE";

  String json = "{";
  json += "\"status_code\": " + String(currentStatus) + ",";
  json += "\"color_name\": \"" + colorName + "\","; 
  json += "\"color_code\": " + String(currentColor) + ",";
  json += "\"rpm\": \"" + String((int)rpmL) + " | " + String((int)rpmR) + "\",";

  json += "\"err_line\": " + String(errLine) + ",";
  json += "\"err_center\": " + String(errCenter) + ",";

  json += "\"sensors\": [";
  for(int i=0; i<5; i++) {
    json += String(sens[i]);
    if(i<4) json += ",";
  }
  json += "],";

  json += "\"max_err\": " + String(maxErrCenter) + ",";
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
  html += ".st-done { color: #28a745; } .st-done::before { content: 'X'; margin-right: 10px; font-weight: bold; }";
  html += ".st-active { color: #1e7e34; font-weight: bold; font-size: 16px; background: #f0fff4; border-radius: 4px; border-left: 4px solid #28a745; }";
  html += ".st-active::before { content: '>'; margin-right: 10px; }";
  html += ".st-pending { color: #adb5bd; } .st-pending::before { content: 'o'; margin-right: 10px; font-size: 20px; }";
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
  html += "canvas { background: #fafafa; border: 1px solid #eee; display: block; width: 100%; height: 60px; }";
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

  html += "<div class='dash-grid'>";
  html += "<div class='card'><div class='card-title'>Speed (RPM)</div><div class='dash-val' id='rpm'>0 | 0</div></div>";
  html += "<div class='card'><div class='card-title'>Cargo</div><div id='color_disp' class='color-box' style='background:#eee;'></div><div style='text-align:center;margin-top:5px;' id='color_txt'>None</div></div>";
  html += "</div>";

  // --- CHART AREA ---
  html += "<div class='card'><div class='card-title'>Deviation Monitor (mm)</div>";
  html += "<div class='dual-val'>";
  html += "<div>Err Sensor: <b id='txt_err_line' style='color:#dc3545'>0.0</b></div>";
  html += "<div>Err Center: <b id='txt_err_center' style='color:#28a745'>0.0</b></div>";
  html += "</div>";
  html += "<canvas id='chartLine' width='300' height='60'></canvas>";
  html += "<div class='lbl-small'>Sensor (Raw)</div>";
  html += "<canvas id='chartCenter' width='300' height='60'></canvas>";
  html += "<div class='lbl-small'>Center (Filtered)</div>";
  html += "</div>";

  html += "<div class='card'><div class='card-title'>Sensors</div><div class='sensor-box'>";
  for(int i=0; i<5; i++) {
    html += "<div class='bar-container'><div id='bar" + String(i) + "' class='bar' style='height: 10%;'></div><div id='val" + String(i) + "' class='bar-val'>0</div></div>";
  }
  html += "</div></div>";

  html += "<div class='card'><div class='card-title'>Log</div><textarea id='log-box' readonly>Waiting...</textarea></div>";
  html += "</div>";

  // --- JAVASCRIPT LOGIC ---
  html += "<script>\n"; 
  html += "let modalShown = false;\n";
  
  html += "let fullLineData = [];\n";
  html += "let fullCenterData = [];\n";

  html += "function sendCmd(c) { \n";
  html += "  if(c=='g') { fullLineData = []; fullCenterData = []; } \n";
  html += "  fetch('/cmd?c='+c); modalShown = false; \n";
  html += "}\n";
  
  html += "function closeModal() { document.getElementById('resultModal').style.display = 'none'; }\n";

  // UPDATE: Hàm vẽ với nhãn biên độ và nét mảnh
  html += "function drawChart(id, data, color) {\n";
  html += "  let c = document.getElementById(id);\n";
  html += "  let ctx = c.getContext('2d');\n";
  html += "  let w = c.width, h = c.height;\n";
  html += "  ctx.clearRect(0,0,w,h);\n";
  
  // Vẽ nhãn biên độ (Scale Markers)
  html += "  ctx.font = '10px sans-serif';\n";
  html += "  ctx.fillStyle = '#999';\n";
  html += "  ctx.fillText('+30mm', 2, 10);\n";
  html += "  ctx.fillText('-30mm', 2, h-2);\n";

  // Vẽ đường Zero
  html += "  ctx.beginPath(); ctx.strokeStyle='#eee'; ctx.moveTo(0, h/2); ctx.lineTo(w, h/2); ctx.stroke();\n";
  
  // Vẽ Data Line
  html += "  ctx.beginPath(); ctx.strokeStyle=color; \n";
  html += "  ctx.lineWidth = 1; \n"; // Nét vẽ mảnh hơn (1px)
  
  html += "  let len = data.length;\n";
  html += "  if(len < 1) return;\n";
  
  html += "  let step = w / (len - 1);\n"; 
  html += "  if (len == 1) step = w;\n";
  
  html += "  for(let i=0; i<len; i++){\n";
  html += "    let y = h/2 - (data[i] * (h/60));\n"; 
  html += "    if(y<0) y=0; if(y>h) y=h;\n";
  html += "    if(i==0) ctx.moveTo(0, y); else ctx.lineTo(i*step, y);\n";
  html += "  }\n";
  html += "  ctx.stroke();\n";
  html += "}\n";

  html += "setInterval(function() {\n";
  html += "  fetch('/data').then(res => res.json()).then(data => {\n";
    html += "  if (data.status_code <= 1) modalShown = false;{\n";
  html += "    for(let i=0; i<=6; i++) {\n";
  html += "      let el = document.getElementById('st' + i);\n";
  html += "      el.className = 'status-item';\n";
  html += "      if (i < data.status_code) el.classList.add('st-done');\n";
  html += "      else if (i == data.status_code) el.classList.add('st-active');\n";
  html += "      else el.classList.add('st-pending');\n";
  html += "    }\n";

  html += "    document.getElementById('rpm').innerText = data.rpm;\n";
  html += "    document.getElementById('color_txt').innerText = data.color_name;\n";
  html += "    let cDiv = document.getElementById('color_disp');\n";
  html += "    if(data.color_code == 1) cDiv.style.background = '#dc3545';\n"; 
  html += "    else if(data.color_code == 2) cDiv.style.background = '#007bff';\n";
  html += "    else cDiv.style.background = '#eee';\n";

  html += "    let el = data.err_line;\n";
  html += "    let el_txt = document.getElementById('txt_err_line');\n";
  html += "    el_txt.innerText = el.toFixed(1);\n";
  html += "    if (Math.abs(el) < 12) el_txt.style.color = '#28a745';\n";
  html += "    else if (Math.abs(el) <= 15) el_txt.style.color = '#ffc107';\n";
  html += "    else el_txt.style.color = '#dc3545';\n";

  html += "    let ec = data.err_center;\n";
  html += "    let ec_txt = document.getElementById('txt_err_center');\n";
  html += "    ec_txt.innerText = ec.toFixed(1);\n";
  html += "    if (Math.abs(ec) < 12) ec_txt.style.color = '#28a745';\n";
  html += "    else if (Math.abs(ec) <= 15) ec_txt.style.color = '#ffc107';\n";
  html += "    else ec_txt.style.color = '#dc3545';\n";

  
  html += "    if(data.status_code >= 1 && data.status_code <= 5) {\n";
  html += "       if(data.status_code != 5 || !modalShown) {\n";
  html += "          fullLineData.push(data.err_line);\n";
  html += "          fullCenterData.push(data.err_center);\n";
  html += "       }\n";
  html += "    }\n";

  html += "    if (data.status_code == 5) {\n";
  html += "       drawChart('chartLine', fullLineData, '#dc3545');\n";
  html += "       drawChart('chartCenter', fullCenterData, '#28a745');\n";
  html += "    } else {\n";
  html += "       let sliceLine = fullLineData.slice(-80);\n";
  html += "       let sliceCenter = fullCenterData.slice(-80);\n";
  html += "       drawChart('chartLine', sliceLine, '#dc3545');\n";
  html += "       drawChart('chartCenter', sliceCenter, '#28a745');\n";
  html += "    }\n";

  html += "    for(let i=0; i<5; i++) {\n";
  html += "       let val = data.sensors[i];\n";
  html += "       let h = (val / 1000) * 100; if(h>100) h=100;\n";
  html += "       document.getElementById('bar' + i).style.height = h + '%';\n";
  html += "       document.getElementById('val' + i).innerText = val;\n";
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