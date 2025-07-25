#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "espadmin";
const char* password = "12345678";
const char* server = "http://10.10.30.107:5000/score";

#define IR_FRONT 22
#define IR_BACK 23

bool alertSent = false;

void setup() {
  Serial.begin(115200);

  pinMode(IR_FRONT, INPUT);
  pinMode(IR_BACK, INPUT);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
}

void loop() {
  bool frontDetected = (digitalRead(IR_FRONT) == LOW);
  bool backDetected = (digitalRead(IR_BACK) == LOW);

  if (frontDetected && backDetected && !alertSent) {
    Serial.println("Defeat detected!");
    sendScoreUpdate();
    alertSent = true;
  }

  // Reset alertSent once sensors no longer detect black
  if (!frontDetected || !backDetected) {
    alertSent = false;
  }

  delay(100); // Small debounce delay
}

void sendScoreUpdate() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(server);
    http.addHeader("Content-Type", "application/json");

    String payload = "{\"robot\":\"R1\",\"event\":\"edge_detected\",\"score\":1}";
    int httpResponseCode = http.POST(payload);

    Serial.print("POST Response code: ");
    Serial.println(httpResponseCode);

    http.end();
  } else {
    Serial.println("WiFi not connected, cannot send score.");
  }
}
