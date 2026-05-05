/*
 * receiver.ino — ESP32 Pan-Tilt Head Receiver
 *
 * Receives PanTiltPacket via ESP-NOW from the transmitter and drives
 * two servos in rate-control mode (joystick deflection = movement speed).
 *
 * SETUP:
 *   1. Flash this sketch first.
 *   2. Open Serial Monitor at 115200 baud.
 *   3. Copy the printed MAC address into transmitter.ino.
 *
 * WIRING:
 *   Pan  servo signal → GPIO18
 *   Tilt servo signal → GPIO19
 *   Servo VCC         → external 5V rail (NOT the ESP32 3.3V pin)
 *   Servo GND         → common GND with ESP32
 *
 * LIBRARY:
 *   Install "ESP32Servo" from the Arduino Library Manager.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------
static constexpr int PIN_PAN = 18;
static constexpr int PIN_TILT = 19;

// Maximum degrees per second at full joystick deflection (speed = ±100).
// Increase to make movement faster, decrease to make it more precise.
static constexpr float DEG_PER_SEC = 90.0f; // °/s at full stick

static constexpr int LOOP_MS = 10;     // main loop period (ms)
static constexpr float CENTER = 90.0f; // reset / home position (°)

// ---------------------------------------------------------------------------
// Shared packet struct — must match transmitter.ino exactly
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    int8_t pan_speed;  // -100 to 100
    int8_t tilt_speed; // -100 to 100
    bool reset;
} PanTiltPacket;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
Servo panServo;
Servo tiltServo;

volatile PanTiltPacket rxPacket;
volatile bool newData = false;

float panPos = CENTER;
float tiltPos = CENTER;

// ---------------------------------------------------------------------------
// ESP-NOW receive callback (runs in WiFi task context)
// ---------------------------------------------------------------------------
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len != sizeof(PanTiltPacket))
        return;
    memcpy((void *)&rxPacket, data, sizeof(PanTiltPacket));
    newData = true;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    // Print own MAC so it can be hardcoded in the transmitter
    WiFi.mode(WIFI_STA);
    Serial.print("[Receiver] MAC address: ");
    Serial.println(WiFi.macAddress());

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
    panServo.write((int)panPos);
    tiltServo.write((int)tiltPos);
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop()
{
    static uint32_t lastMs = 0;
    uint32_t now = millis();
    uint32_t elapsed = now - lastMs;
    if (elapsed < (uint32_t)LOOP_MS)
        return;
    lastMs = now;

    // Snapshot volatile packet with interrupts disabled briefly
    PanTiltPacket pkt;
    noInterrupts();
    pkt = rxPacket;
    bool nd = newData;
    newData = false;
    interrupts();

    if (nd)
    {
        if (pkt.reset)
        {
            panPos = CENTER;
            tiltPos = CENTER;
            Serial.println("[Receiver] Reset to center.");
        }
        else
        {
            // Rate integration: deg = speed_fraction * DEG_PER_SEC * dt_sec
            float dt = elapsed / 1000.0f;
            panPos += (pkt.pan_speed / 100.0f) * DEG_PER_SEC * dt;
            tiltPos += (pkt.tilt_speed / 100.0f) * DEG_PER_SEC * dt;

            // Clamp to valid servo range
            panPos = constrain(panPos, 0.0f, 180.0f);
            tiltPos = constrain(tiltPos, 0.0f, 180.0f);

            Serial.printf("[Receiver] spd=(%4d,%4d)  pos=(%.1f°, %.1f°)\n",
                          pkt.pan_speed, pkt.tilt_speed, panPos, tiltPos);
        }

        panServo.write((int)panPos);
        tiltServo.write((int)tiltPos);
    }
}
