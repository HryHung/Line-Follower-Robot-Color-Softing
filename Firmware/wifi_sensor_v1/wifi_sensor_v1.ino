#include <WiFi.h>
#include <WebServer.h>

// Replace with your network credentials
const char* ssid = "KTX&DTDH-SV-FREE";
const char* password = "12345678";

// Set up the web server on port 80
WebServer server(80);

// Sensor pins
#define S1 32
#define S2 33
#define S3 34
#define S4 35
#define S5 36

// Calibration values
int rawMin[5] = {65, 69, 69, 80, 88};
int rawMax[5] = {2672, 2823, 2873, 3025, 3014};
int weight[5] = {-20, -10, 0, 10, 20};

// Global variables for sensor data
int norm[5] = {0, 0, 0, 0, 0};
int lineErr = 0;

// Clamp map function
int clampMap(int x, int in_min, int in_max, int out_min, int out_max) {
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  long v = (long)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return (int)v;
}

// Calculate line error
int lineErr_Cal(int sensor[5]) {
  long sumWeight = 0;
  long sumValue = 0;
  for (int i = 0; i < 5; i++) {
    sumWeight += (long)sensor[i] * weight[i];
    sumValue += sensor[i];
  }
  if (sumValue == 0) return 0; // Avoid division by zero
  return sumWeight / sumValue; // Error value ~ [-2…2]
}

// Minimal webpage HTML
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Sensor Data</title>
</head>
<body>
  <h1>Sensor Data</h1>
  <div id="sensors">
    <p>Sensor 1: <span id="s1">0</span></p>
    <p>Sensor 2: <span id="s2">0</span></p>
    <p>Sensor 3: <span id="s3">0</span></p>
    <p>Sensor 4: <span id="s4">0</span></p>
    <p>Sensor 5: <span id="s5">0</span></p>
    <p>Error: <span id="err">0</span></p>
  </div>
  <script>
    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('s1').innerText = data.norm[0];
          document.getElementById('s2').innerText = data.norm[1];
          document.getElementById('s3').innerText = data.norm[2];
          document.getElementById('s4').innerText = data.norm[3];
          document.getElementById('s5').innerText = data.norm[4];
          document.getElementById('err').innerText = data.err;
        })
        .catch(error => console.error('Error:', error));
    }
    setInterval(updateData, 100); // Update every 1 second
    updateData(); // Initial update
  </script>
</body>
</html>
)rawliteral";

// Handle root path
void handleRoot() {
  server.send(200, "text/html", webpage);
}

// Handle data endpoint
void handleData() {
  String json = "{\"norm\":[" + String(norm[0]) + "," + String(norm[1]) + "," + 
                String(norm[2]) + "," + String(norm[3]) + "," + String(norm[4]) + 
                "], \"err\":" + String(lineErr) + "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  Serial.println("=== TRCT5000 ===");

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to Wi-Fi. IP address: ");
  Serial.println(WiFi.localIP());

  // Routes
  server.on("/", handleRoot);
  server.on("/data", handleData);

  // Start the server
  server.begin();
  Serial.println("Web server started!");
}

void loop() {
  // Handle client requests
  server.handleClient();

  static unsigned long lastTime = 0;
  if (millis() - lastTime >= 100) {
    int s[5];
    s[0] = analogRead(S1);
    s[1] = analogRead(S2);
    s[2] = analogRead(S3);
    s[3] = analogRead(S4);
    s[4] = analogRead(S5);

    for (int i = 0; i < 5; i++) {
      norm[i] = clampMap(s[i], rawMin[i], rawMax[i], 0, 1000);
    }

    // Calculate error
    lineErr = lineErr_Cal(norm);

    // Print to serial (optional)
    Serial.print("Normalized: ");
    for (int i = 0; i < 5; i++) {
      Serial.print(norm[i]);
      Serial.print("\t");
    }
    Serial.print(" | Err = ");
    Serial.println(lineErr);

    lastTime = millis();
  }
}