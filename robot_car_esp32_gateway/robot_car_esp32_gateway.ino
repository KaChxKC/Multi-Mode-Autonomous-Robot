/*
 * 4-in-1 Autonomous Robot Car - ESP32 WiFi gateway (Blynk)
 * ---------------------------------------------------------
 * Connects to WiFi and the Blynk Cloud, relays dashboard commands to the Arduino Uno over a
 * serial link, and publishes the Uno's distance reading to the dashboard.
 *
 * Blynk datastreams
 *   V0  Power          (Integer 0..1)   Switch            -> robot ON / OFF
 *   V1  Mode Selector  (Integer 1..4)   Segmented switch  -> 1 obstacle, 2 line, 3 line+obstacle, 4 object
 *   V2  Distance       (Integer 0..400) Gauge             <- front distance in cm
 *
 * Serial link to the Uno (hardware Serial2, 9600 baud)
 *   ESP32 GPIO17 (TX2) -> Uno D2 (RX)
 *   ESP32 GPIO16 (RX2) <- Uno D4 (TX) through a 1k / 2k voltage divider (the ESP32 is 3.3 V only)
 *   GND <-> GND (common ground is required)
 *
 * Setup: copy arduino_secrets.example.h to arduino_secrets.h and fill in your Blynk template ID,
 * auth token and 2.4 GHz WiFi credentials (arduino_secrets.h is git-ignored).
 * Library: Blynk (Volodymyr Shymanskyy).  Board: "ESP32 Dev Module" (Espressif esp32 core).
 */

#include "arduino_secrets.h"

#define BLYNK_PRINT Serial   // debug over USB

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Hardware Serial2 to the Uno: RX2 = GPIO16, TX2 = GPIO17
#define UNO_RX_PIN 16
#define UNO_TX_PIN 17

BlynkTimer timer;
String rxLine = "";

// Send a command to the Uno several times so a corrupted/missed byte
// (SoftwareSerial can drop bytes while the Uno is busy) still gets through.
void sendToUno(const String &cmd)
{
  for (int i = 0; i < 4; i++)
  {
    Serial2.print(cmd);
    Serial2.print('\n');
    delay(20);
  }
}

// Blynk -> ON/OFF switch (V0)
BLYNK_WRITE(V0)
{
  int on = param.asInt();
  sendToUno(on ? "E1" : "E0");
}

// Blynk -> Mode selector (V1), values 1..4
BLYNK_WRITE(V1)
{
  int mode = param.asInt();
  if (mode >= 1 && mode <= 4)
  {
    sendToUno("M" + String(mode));
  }
}

// Read distance telemetry from the Uno and push to gauge (V2)
void readFromUno()
{
  while (Serial2.available())
  {
    char c = Serial2.read();
    if (c == '\n')
    {
      if (rxLine.length() > 1 && rxLine[0] == 'D')
      {
        int distance = rxLine.substring(1).toInt();
        Blynk.virtualWrite(V2, distance);
      }
      rxLine = "";
    }
    else if (c != '\r')
    {
      rxLine += c;
      if (rxLine.length() > 10) rxLine = "";
    }
  }
}

void setup()
{
  Serial.begin(115200);                                   // USB debug
  Serial2.begin(9600, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN); // link to Uno
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(300L, readFromUno);
}

void loop()
{
  Blynk.run();
  timer.run();
}
