// =============================================================
//  RX_ESP01_IMU.ino  –  ESP-01 Bidirectional ESP-NOW Relay
//
//  DOWN: TX ESP32 –[ESP-NOW]–> ESP-01 –[UART]–> ATmega2560
//  UP:   ATmega2560 –[UART]–> ESP-01 –[ESP-NOW]–> TX ESP32
//
//  ESP-01 UART (only one):
//    GPIO1 (TX) → BSS138 → ATmega RXD3 (PJ0)
//    GPIO3 (RX) → BSS138 → ATmega TXD3 (PJ1)
//
//  IMPORTANT: No Serial.print() anywhere in this file.
//    GPIO1 is the ATmega data line. Any debug print
//    goes straight to the ATmega as garbage bytes.
//
//  Board: Generic ESP8266 Module
//  Flash: 1M, CPU: 80MHz
// =============================================================

#include "espnow.h"
#include "user_interface.h"
#include <ESP8266WiFi.h>
#include <espnow.h>

// ── TX ESP32 AP MAC address ───────────────────────────────────
uint8_t TX_MAC[] = {0xD4, 0xE9, 0xF4, 0x71, 0xCA, 0x91};

// ── Telemetry buffer ──────────────────────────────────────────
#define TELEM_BUF 140
char    telem_buf[TELEM_BUF];
uint8_t telem_idx = 0;

// ── ESP-NOW send callback ─────────────────────────────────────
void onSent(uint8_t *mac, uint8_t status)
{
    // silent — cannot Serial.print here
}

// ── ESP-NOW receive callback ──────────────────────────────────
// Receives 3-byte PWM command from TX ESP32 → forward all bytes to ATmega
void onReceive(uint8_t *mac, uint8_t *data, uint8_t len)
{
    if (len == 3)
    {
        Serial.write(data, len);   // forward all 3 bytes to ATmega via UART
        esp_now_send(TX_MAC, (uint8_t*)"PKT_OK", 6);
    }
    else
    {
        // Wrong length — report back to TX for debugging, don't forward
        char bad[16];
        uint8_t n = (uint8_t)snprintf(bad, sizeof(bad), "PKT_BAD_LEN:%u", len);
        esp_now_send(TX_MAC, (uint8_t*)bad, n);
    }
}

// ── Setup ─────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);   // UART to ATmega2560

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    wifi_set_channel(1);
    delay(100);

    if (esp_now_init() != 0) { ESP.restart(); return; }

    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_add_peer(TX_MAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onReceive);

    // Diagnostic: confirm boot to TX
    esp_now_send(TX_MAC, (uint8_t*)"RX-TX connection established", 28);
}

void loop()
{
    // Diagnostic heartbeat every 2 seconds
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 2000)
    {
        esp_now_send(TX_MAC, (uint8_t*)"LOOP_ALIVE", 10);
        lastHeartbeat = millis();
    }

    // Telemetry: ATmega → ESP-01 → TX ESP32
    while (Serial.available())
    {
        char c = (char)Serial.read();

        // Diagnostic: confirms ATmega telemetry is flowing
        if (c == '$') esp_now_send(TX_MAC, (uint8_t*)"GOT_DOLLAR", 10);

        if (c == '\n')
        {
            if (telem_idx > 0 && telem_buf[0] == '$')
            {
                telem_buf[telem_idx] = '\0';
                esp_now_send(TX_MAC, (uint8_t*)telem_buf, telem_idx);
                telem_idx = 0;
                break;  // let yield() flush the send
            }
            telem_idx = 0;
        }
        else if (c == '$')
        {
            telem_idx = 0;
            telem_buf[telem_idx++] = c;
        }
        else if (telem_idx < TELEM_BUF - 1)
        {
            telem_buf[telem_idx++] = c;
        }
    }
    yield();
}