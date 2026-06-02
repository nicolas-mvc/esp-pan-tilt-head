/*
 * receiver.ino — ESP32 Pan-Tilt Head Receiver
 *
 * Receives PanTiltPacket via ESP-NOW from the transmitter and drives
 * two servos by directly writing received pulse widths.
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

static constexpr int PAN_MIN_US = 950;
static constexpr int PAN_MAX_US = 2130;
static constexpr int TILT_MIN_US = 1000;
static constexpr int TILT_MAX_US = 1500;
static constexpr int PAN_CENTER_US = 1550;
static constexpr int TILT_CENTER_US = (TILT_MIN_US + TILT_MAX_US) / 2;

// ---------------------------------------------------------------------------
// Receiver MAC reference printed at startup for manual verification
// ---------------------------------------------------------------------------
static const uint8_t RECEIVER_MAC[6] = {0x08, 0xB6, 0x1F, 0xB8, 0xA3, 0xD0};

// ---------------------------------------------------------------------------
// Shared packet struct — must match transmitter.ino exactly
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    uint16_t pan_us;  // constrained to PAN_MIN_US..PAN_MAX_US on receive
    uint16_t tilt_us; // constrained to TILT_MIN_US..TILT_MAX_US on receive
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
    panServo.attach(PIN_PAN, PAN_MIN_US, PAN_MAX_US);
    tiltServo.attach(PIN_TILT, TILT_MIN_US, TILT_MAX_US);
    panServo.writeMicroseconds(PAN_CENTER_US);
    tiltServo.writeMicroseconds(TILT_CENTER_US);

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

        // Constrain pulse widths to valid servo range
        uint16_t panUs = constrain(pkt.pan_us, PAN_MIN_US, PAN_MAX_US);
        uint16_t tiltUs = constrain(pkt.tilt_us, TILT_MIN_US, TILT_MAX_US);

        panServo.writeMicroseconds(panUs);
        tiltServo.writeMicroseconds(tiltUs);

        Serial.printf("[Receiver] us=(%4d, %4d)\n", panUs, tiltUs);
    }

    delay(10);
}
