#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#include <MD_MAX72xx.h>
#include <SPI.h>
#include "MD_RobotEyes.h"

// -------------------- MOTOR DEFINES --------------------
#define ENA   14  // speed control motor Right  (GPIO14, D5)
#define ENB   12  // speed control motor Left   (GPIO12, D6)
#define IN_1  15  // L298N IN1 Right motor     (GPIO15, D8)
#define IN_2  13  // L298N IN2 Right motor     (GPIO13, D7)
#define IN_3  2   // L298N IN3 Left motor      (GPIO2,  D4)
#define IN_4  0   // L298N IN4 Left motor      (GPIO0,  D3)

// -------------------- WIFI CONFIG --------------------
const char* SSID = "Annie";
const char* PASS = "12345678";
ESP8266WebServer server(80);

// -------------------- MOTION VARIABLES --------------------
float lin = 0.0, ang = 0.0, secondsMoving = 0.0;
long  lin_i = 0, ang_i = 0;
int   leftSpeed = 0, rightSpeed = 0;
int   leftVal = 0, rightVal = 0;

// -------------------- EYES CONFIG --------------------
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW  
#define MAX_DEVICES   2   // two matrices side by side
#define DATA_PIN 4   // D2 -> DIN
#define CLK_PIN  5   // D1 -> CLK
#define CS_PIN   16  // D0 -> CS/LOAD

MD_MAX72XX M(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
MD_RobotEyes E;

// we only care about Neutral + Blink loop
const MD_RobotEyes::emotion_t BLINK_SEQ[] = { MD_RobotEyes::E_NEUTRAL, MD_RobotEyes::E_BLINK };
uint8_t blinkIndex = 0;
uint32_t lastBlinkTime = 0;
uint16_t blinkInterval = 2000;  // every 2 sec blink

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(9600);
  Serial.println("\nESP8266 Differential Drive + Eyes");

  // --- Motor pins ---
  pinMode(ENA,   OUTPUT);
  pinMode(ENB,   OUTPUT);
  pinMode(IN_1,  OUTPUT);
  pinMode(IN_2,  OUTPUT);
  pinMode(IN_3,  OUTPUT);
  pinMode(IN_4,  OUTPUT);
  analogWriteRange(1023);
  analogWriteFreq(1000);

  // --- WiFi AP ---
  WiFi.softAP(SSID, PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // --- HTTP Handlers ---
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

  server.on("/twist", HTTP_POST, []() {
    String body = server.arg("plain");
    Serial.print("Received: "); Serial.println(body);
    if (processCommandString(body)) {
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "BadCommand");
    }
  });

  server.begin();
  Serial.println("HTTP server started");

  // --- Eyes ---
  M.begin();
  E.begin(&M);
  E.setAnimation(MD_RobotEyes::E_NEUTRAL, true);  // start neutral
}

// -------------------- LOOP --------------------
void loop() {
  server.handleClient();

  // keep eyes animation alive
  E.runAnimation();

  // handle blinking every few seconds
  if (millis() - lastBlinkTime > blinkInterval) {
    blinkIndex = (blinkIndex + 1) % 2;  // toggle between Neutral & Blink
    E.setAnimation(BLINK_SEQ[blinkIndex], true);
    lastBlinkTime = millis();
  }
}

// -------------------- MOTOR FUNCTIONS --------------------
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
  leftVal  = max(leftVal,  800);  
  rightVal = max(rightVal, 800);

  // RIGHT motor dir
  if (rightSpeed > 0) { digitalWrite(IN_1, HIGH); digitalWrite(IN_2, LOW); }
  else if (rightSpeed < 0) { digitalWrite(IN_1, LOW); digitalWrite(IN_2, HIGH); }
  else { digitalWrite(IN_1, LOW); digitalWrite(IN_2, LOW); }

  // LEFT motor dir
  if (leftSpeed > 0) { digitalWrite(IN_3, HIGH); digitalWrite(IN_4, LOW); }
  else if (leftSpeed < 0) { digitalWrite(IN_3, LOW); digitalWrite(IN_4, HIGH); }
  else { digitalWrite(IN_3, LOW); digitalWrite(IN_4, LOW); }

  // apply PWM
  analogWrite(ENA, abs(leftVal));
  analogWrite(ENB, abs(rightVal));
  Serial.printf("Drive L:%d R:%d for %.0f ms\n", leftVal, rightVal, secondsMoving * 1000);

  // blocking delay
  unsigned long start = millis();
  while (millis() - start < (unsigned long)(secondsMoving * 1000.0)) {
    server.handleClient();   // keep server responsive
    E.runAnimation();        // keep eyes alive
  }

  // stop
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  Serial.println("Move complete");
}
