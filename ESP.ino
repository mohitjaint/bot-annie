#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// motor driver pin definitions
#define ENA   14  // speed control motor Right  (GPIO14, D5)
#define ENB   12  // speed control motor Left   (GPIO12, D6)
#define IN_1  15  // L298N IN1 Right motor     (GPIO15, D8)
#define IN_2  13  // L298N IN2 Right motor     (GPIO13, D7)
#define IN_3  2   // L298N IN3 Left motor      (GPIO2,  D4)
#define IN_4  0   // L298N IN4 Left motor      (GPIO0,  D3)

const char* SSID = "Annie";
const char* PASS = "12345678";

ESP8266WebServer server(80);

// motion parameters
float lin = 0.0, ang = 0.0, secondsMoving = 0.0;
long  lin_i = 0, ang_i = 0;
int   leftSpeed = 0, rightSpeed = 0;
int   leftVal = 0, rightVal = 0;

void setup() {
  // Root page with control buttons
server.on("/", HTTP_GET, []() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>ESP8266 Robot Control</title>
      <style>
        body { font-family: Arial; text-align: center; margin-top: 40px; }
        button { width: 100px; height: 60px; margin: 10px; font-size: 18px; }
      </style>
      <script>
        function sendCommand(lin, ang, dur) {
          fetch("/twist", {
            method: "POST",
            headers: { "Content-Type": "text/plain" },
            body: "[" + lin + "," + ang + "," + dur + "]"
          });
        }
      </script>
    </head>
    <body>
      <h2>ESP8266 Motor Control</h2>
      <div>
        <button onclick="sendCommand(1.0,0.0,1.5)">Forward</button><br>
        <button onclick="sendCommand(0.0,-1.0,1)">Left</button>
        <button onclick="sendCommand(0.0,1.0,1)">Right</button><br>
        <button onclick="sendCommand(-1.0,0.0,1.5)">Backward</button><br>
        <button onclick="sendCommand(0.0,0.0,0.1)">Stop</button>
      </div>
    </body>
    </html>
  )rawliteral";
  server.send(200, "text/html", html);
});

  Serial.begin(9600);
  Serial.println();
  Serial.println("Annie, ESP8266 Differential Drive Bot starting...");

  // motor pins
  pinMode(ENA,   OUTPUT);
  pinMode(ENB,   OUTPUT);
  pinMode(IN_1,  OUTPUT);
  pinMode(IN_2,  OUTPUT);
  pinMode(IN_3,  OUTPUT);
  pinMode(IN_4,  OUTPUT);

  // PWM config
  analogWriteRange(1023);
  analogWriteFreq(1000);

  // start AP
  WiFi.softAP(SSID, PASS);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.println(WiFi.localIP());
  // HTTP POST /twist handler
  server.on("/twist", HTTP_POST, []() {
    String body = server.arg("plain");
    Serial.print("Received: ");
    Serial.println(body);
    if (processCommandString(body)) {
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "BadCommand");
    }
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

// parse “[lin,ang,dur]”
bool processCommandString(String s) {
  s.trim();
  if (!s.startsWith("[") || !s.endsWith("]")) return false;
  s = s.substring(1, s.length()-1);
  s.trim();

  int c1 = s.indexOf(',');
  int c2 = s.indexOf(',', c1+1);
  if (c1 < 0 || c2 < 0) return false;

  lin = s.substring(0, c1).toFloat();
  ang = s.substring(c1+1, c2).toFloat();
  secondsMoving = s.substring(c2+1).toFloat();

  Serial.printf("Parsed lin=%.2f, ang=%.2f, dur=%.2f\n", lin, ang, secondsMoving);
  doMove();
  return true;
}

void doMove() {
  // clamp inputs
  lin = constrain(lin, -2.0, 2.0);
  ang = constrain(ang, -2.0, 2.0);

  // scale
  lin_i = (long)(lin * 1000.0);
  ang_i = (long)(ang * 1000.0);

  // differential drive math
  leftSpeed  = lin_i - ang_i;
  rightSpeed = lin_i + ang_i;

  // map to PWM (0–1023)
  leftVal  = map(abs(leftSpeed),  0, 4000, 0, 1023);
  rightVal = map(abs(rightSpeed), 0, 4000, 0, 1023);
  leftVal  = max(leftVal,  800);  // motor dead-zone
  rightVal = max(rightVal, 800);

  // set motor directions
  // RIGHT motor uses IN_1/IN_2
  if (leftSpeed > rightSpeed) {
    digitalWrite(IN_1, HIGH);
    digitalWrite(IN_2, LOW);
  } else if (leftSpeed < rightSpeed) {
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, HIGH);
  } else if (leftSpeed < 0) {
    digitalWrite(IN_1, HIGH);
    digitalWrite(IN_2, LOW);
  } else {
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, HIGH);
  }

  // LEFT motor uses IN_3/IN_4
  if (rightSpeed > leftSpeed) {
    digitalWrite(IN_3, HIGH);
    digitalWrite(IN_4, LOW);
  } else if (rightSpeed < leftSpeed) {
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, HIGH);
  } else if (rightSpeed < 0) {
    digitalWrite(IN_3, HIGH);
    digitalWrite(IN_4, LOW);
  } else {
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, HIGH);
  }

  // apply PWM
  analogWrite(ENA, leftVal);
  analogWrite(ENB, rightVal);
  Serial.printf("Drive L:%d R:%d for %.0f ms\n", leftVal, rightVal, secondsMoving * 1000);

  // blocking delay
  unsigned long start = millis();
  while (millis() - start < (unsigned long)(secondsMoving * 1000.0)) {
    delay(10);
  }

  // stop
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  Serial.println("Move complete");
}