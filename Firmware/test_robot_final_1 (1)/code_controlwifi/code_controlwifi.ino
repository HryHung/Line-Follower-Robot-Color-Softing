/* * CODE TRẠM ĐIỀU KHIỂN (REMOTE)
 * Đã điền MAC Robot: 4C:C3:82:BE:D9:F8
 */
#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>

// --- 1. ĐỊA CHỈ CỦA ROBOT (Đã điền sẵn) ---
uint8_t robotAddress[] = {0x4C, 0xC3, 0x82, 0xBE, 0xD9, 0xF8}; 

// Cấu trúc dữ liệu nhận về
typedef struct struct_message {
  int s[5]; float err; int st; int inter; float r1; float r2; char msg[32];
} struct_message;
struct_message myData;

// Cấu trúc lệnh gửi đi
typedef struct struct_command { char cmd; } struct_command;
struct_command myCmd;

WebServer server(80);
esp_now_peer_info_t peerInfo;

// --- GIAO DIỆN WEB ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>CONTROL</title><style>
body{font-family:sans-serif;text-align:center;background:#f0f0f0;padding:10px;}
.btn{border:none;color:white;padding:25px;width:90%;font-size:25px;margin:10px 0;border-radius:15px;cursor:pointer;display:block;margin-left:auto;margin-right:auto;}
.g{background:#2ecc71;box-shadow:0 5px #27ae60;} .g:active{box-shadow:0 2px #27ae60;transform:translateY(3px);}
.s{background:#e74c3c;box-shadow:0 5px #c0392b;} .s:active{box-shadow:0 2px #c0392b;transform:translateY(3px);}
.card{background:white;padding:15px;margin-top:20px;border-radius:10px;box-shadow:0 4px 8px rgba(0,0,0,0.1);}
</style></head><body>
<h2>ĐIỀU KHIỂN ROBOT</h2>
<button class="btn g" onclick="c('g')">CHẠY (START)</button>
<button class="btn s" onclick="c('s')">DỪNG (STOP)</button>
<div class="card">
  <h3>Trạng thái: <span id="m" style="color:blue">...</span></h3>
  <p>Tốc độ: Trái <b id="r1">0</b> | Phải <b id="r2">0</b></p>
  <p>Line Err: <span id="e">0</span></p>
</div>
<script>
function c(v){var x=new XMLHttpRequest();x.open("GET","/c?v="+v,true);x.send();}
setInterval(function(){fetch("/d").then(r=>r.json()).then(o=>{
document.getElementById("m").innerHTML=o.m;document.getElementById("r1").innerHTML=Math.round(o.r1);
document.getElementById("r2").innerHTML=Math.round(o.r2);document.getElementById("e").innerHTML=o.e;
});},200);</script></body></html>
)rawliteral";

void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *data, int len) {
  memcpy(&myData, data, sizeof(myData));
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);
  
  // TÊN WIFI VÀ PASS (Bạn dùng cái này để kết nối điện thoại)
  WiFi.softAP("ROBOT_WIFI_CONTROL", "12345678"); 

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 0; peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  server.on("/", [](){ server.send(200, "text/html", index_html); });
  server.on("/c", [](){
    String v = server.arg("v");
    if(v.length()>0){
      myCmd.cmd = v.charAt(0);
      esp_now_send(robotAddress, (uint8_t *) &myCmd, sizeof(myCmd));
      Serial.println(v=="g"?"GO":"STOP");
    }
    server.send(200, "text/plain", "OK");
  });
  server.on("/d", [](){
    String j="{\"m\":\""+String(myData.msg)+"\",\"r1\":"+String(myData.r1)+",\"r2\":"+String(myData.r2)+",\"e\":"+String(myData.err)+"}";
    server.send(200, "application/json", j);
  });
  server.begin();
  Serial.println("WEB SERVER STARTED: 192.168.4.1");
}
void loop() { server.handleClient(); }