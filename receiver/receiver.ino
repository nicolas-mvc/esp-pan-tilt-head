/*
 * receiver.ino — ESP32 Pan-Tilt Head Receiver
 *
 * Receives PanTiltPacket via ESP-NOW from the transmitter and drives
 * two servos to the specified positions directly (potentiometer control).
 * Turns on blue built-in LED when connected.
 *
 * SETUP:
 *   1. Flash this sketch first.
 *   2. Open Serial Monitor at 115200 baud to verify MAC address.
 *   3. Ensure transmitter has this receiver's MAC address hardcoded.
 *
 * WIRING:
 *   Pan  servo signal → GPIO14
 *   Tilt servo signal → GPIO15
 *   Servo VCC         → external 5V rail (NOT the ESP32 3.3V pin)
 *   Servo GND         → common GND with ESP32
 *   Built-in LED      → GPIO2 (usually on-board)
 *
 * LIBRARY:
 *   Install "ESP32Servo" from the Arduino Library Manager.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// Pin constants
// ---------------------------------------------------------------------------
static constexpr int PIN_PAN = 14;
static constexpr int PIN_TILT = 15;
static constexpr int PIN_LED = 2; // Built-in LED

// ---------------------------------------------------------------------------
// Receiver MAC address (hardcoded)
// ---------------------------------------------------------------------------
static const uint8_t RECEIVER_MAC[6] = {0x08, 0xB6, 0x1F, 0xB8, 0xA3, 0xD0};

// ---------------------------------------------------------------------------
// Shared packet struct — must match transmitter.ino exactly
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    uint8_t pan_pos;  // 0 to 180 (servo angle in degrees)
    uint8_t tilt_pos; // 0 to 180 (servo angle in degrees)
} PanTiltPacket;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
Servo panServo;
Servo tiltServo;

volatile PanTiltPacket rxPacket;
volatile bool newData = false;
volatile bool connected = false;
volatile uint32_t lastPacketMs = 0;
static constexpr uint32_t CONNECTION_TIMEOUT_MS = 1000; // timeout after 1 sec of no packets

// ---------------------------------------------------------------------------
// ESP-NOW receive callback (runs in WiFi task context)
// ---------------------------------------------------------------------------
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len != sizeof(PanTiltPacket))
        return;
    memcpy((void *)&rxPacket, data, sizeof(PanTiltPacket));
    newData = true;
    lastPacketMs = millis();
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(500);

    // Configure LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW); // LED off initially

    // Print own MAC so it can be verified
    WiFi.mode(WIFI_STA);
    Serial.print("[Receiver] MAC address: ");
    Serial.println(WiFi.macAddress());
    Serial.print("[Receiver] Expected MAC: ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                  RECEIVER_MAC[0], RECEIVER_MAC[1], RECEIVER_MAC[2],
                  RECEIVER_MAC[3], RECEIVER_MAC[4], RECEIVER_MAC[5]);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[Receiver] ESP-NOW init failed — halting.");
        while (true)
            delay(1000);
    }
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("[Receiver] ESP-NOW ready, waiting for packets...");

    // Attach servos
    panServo.attach(PIN_PAN, 500, 2400);
    tiltServo.attach(PIN_TILT, 500, 2400);
    panServo.write(90); // Center position
    tiltServo.write(90);

    lastPacketMs = millis();
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop()
{
    // Check connection timeout
    uint32_t now = millis();
    bool wasConnected = connected;
    connected = (now - lastPacketMs) < CONNECTION_TIMEOUT_MS;

    if (connected != wasConnected)
    {
        if (connected)
        {
            digitalWrite(PIN_LED, HIGH); // Turn LED on
            Serial.println("[Receiver] Connected!");
        }
        else
        {
            digitalWrite(PIN_LED, LOW); // Turn LED off
            Serial.println("[Receiver] Connection lost.");
        }
    }

    // Process received packet
    if (newData)
    {
        newData = false;

        PanTiltPacket pkt;
        noInterrupts();
        memcpy(&pkt, (const void *)&rxPacket, sizeof(pkt));
        interrupts();

        // Constrain positions to valid servo range
        uint8_t panPos = constrain(pkt.pan_pos, 0, 180);
        uint8_t tiltPos = constrain(pkt.tilt_pos, 0, 180);

        panServo.write(panPos);
        tiltServo.write(tiltPos);

        Serial.printf("[Receiver] pos=(%3d°, %3d°)\n", panPos, tiltPos);
    }

    delay(10);
}
