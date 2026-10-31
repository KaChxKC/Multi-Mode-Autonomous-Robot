/*
 * Copy this file to "arduino_secrets.h" (same folder) and fill in your own values.
 * arduino_secrets.h is listed in .gitignore - never commit real credentials.
 */
#ifndef ARDUINO_SECRETS_H
#define ARDUINO_SECRETS_H

// ---- Blynk: copy these three lines from Blynk.Console -> your device -> Firmware configuration ----
#define BLYNK_TEMPLATE_ID   "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Robot Car"
#define BLYNK_AUTH_TOKEN    "YOUR_AUTH_TOKEN"

// ---- WiFi (the ESP32 supports 2.4 GHz only) ----
char ssid[] = "YOUR_WIFI_NAME";
char pass[] = "YOUR_WIFI_PASSWORD";

#endif
