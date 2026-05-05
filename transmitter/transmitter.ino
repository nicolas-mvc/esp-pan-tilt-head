/*
 * transmitter.ino — ESP32 Pan-Tilt Head Transmitter
 *
 * Reads a joystick (rate-control) and a reset button, then sends
 * PanTiltPacket structs to the receiver via ESP-NOW.
 *
 * SETUP:
 *   1. Flash receiver.ino first and note the MAC printed on Serial.
 *   2. Replace RECEIVER_MAC below with the actual receiver MAC.
 *   3. Flash this sketch.
 *
 * WIRING:
 *   Joystick X (pan)  → GPIO36 (ADC1_CH0)
 *   Joystick Y (tilt) → GPIO39 (ADC1_CH3)
 *   Joystick VCC      → 3.3V
 *   Joystick GND      → GND
 *   Reset button      → GPIO15 + GND  (internal pull-up; active LOW)
 */

#include <WiFi.h>
#include <esp_now.h>

// ---------------------------------------------------------------------------
// *** CHANGE THIS to your receiver's MAC address ***
// ---------------------------------------------------------------------------
static uint8_t RECEIVER_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------------------------------------------------------------------------
// Pin & timing constants
// ---------------------------------------------------------------------------
static constexpr int PIN_JOY_PAN = 36;  // ADC1_CH0 — input only, no pull
static constexpr int PIN_JOY_TILT = 39; // ADC1_CH3 — input only, no pull
static constexpr int PIN_RESET_BTN = 15;

static constexpr int TX_INTERVAL_MS = 20; // send rate (50 Hz)

// ADC is 12-bit: 0–4095, center ≈ 2048
static constexpr int ADC_CENTER = 2048;
static constexpr int ADC_MAX_RANGE = 2048; // distance from center to rail
static constexpr int DEADZONE = 200;       // raw ADC counts around center

// Button debounce
static constexpr uint32_t DEBOUNCE_MS = 30;

// ---------------------------------------------------------------------------
// Shared packet struct — must match receiver.ino exactly
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    int8_t pan_speed;  // -100 to 100
    int8_t tilt_speed; // -100 to 100
    bool reset;
} PanTiltPacket;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Map raw ADC value to speed in [-100, 100] with deadzone.
int8_t adcToSpeed(int raw)
{
    int offset = raw - ADC_CENTER;
    if (abs(offset) < DEADZONE)
        return 0;

    // Remove deadzone: shrink the live range to start at 0
    int sign = (offset > 0) ? 1 : -1;
    int magnitude = abs(offset) - DEADZONE;
    int liveRange = ADC_MAX_RANGE - DEADZONE; // effective travel

    int speed = (magnitude * 100) / liveRange;
    speed = constrain(speed, 0, 100) * sign;
    return (int8_t)speed;
}

// Debounced button read — returns true on the falling edge (press).
bool buttonPressed()
{
    static bool lastState = HIGH;
    static uint32_t lastChangeMs = 0;
    static bool stableState = HIGH;

    bool reading = digitalRead(PIN_RESET_BTN);
    uint32_t now = millis();

    if (reading != lastState)
    {
        lastChangeMs = now;
        lastState = reading;
    }

    if ((now - lastChangeMs) > DEBOUNCE_MS)
    {
        if (stableState == HIGH && reading == LOW)
        {
            stableState = LOW;
            return true; // falling edge — button just pressed
        }
        if (stableState == LOW && reading == HIGH)
        {
            stableState = HIGH;
        }
    }
    return false;
}

// ESP-NOW send callback — logs errors if the peer did not acknowledge.
void onDataSent(const uint8_t *mac, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS)
    {
        Serial.println("[TX] Send FAILED — check receiver is on and MAC is correct.");
    }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    pinMode(PIN_RESET_BTN, INPUT_PULLUP);
    // ADC pins (36, 39) are input-only; no pinMode needed

    WiFi.mode(WIFI_STA);
    Serial.print("[TX] Transmitter MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[TX] ESP-NOW init failed — halting.");
        while (true)
            delay(1000);
    }
    esp_now_register_send_cb(onDataSent);

    // Register receiver peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("[TX] Failed to add peer — halting.");
        while (true)
            delay(1000);
    }

    Serial.println("[TX] ESP-NOW ready. Sending packets...");
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop()
{
    static uint32_t lastTxMs = 0;
    uint32_t now = millis();
    if (now - lastTxMs < (uint32_t)TX_INTERVAL_MS)
        return;
    lastTxMs = now;

    PanTiltPacket pkt;
    pkt.reset = buttonPressed();

    if (pkt.reset)
    {
        pkt.pan_speed = 0;
        pkt.tilt_speed = 0;
        Serial.println("[TX] Reset button pressed.");
    }
    else
    {
        int rawPan = analogRead(PIN_JOY_PAN);
        int rawTilt = analogRead(PIN_JOY_TILT);

        pkt.pan_speed = adcToSpeed(rawPan);
        pkt.tilt_speed = adcToSpeed(rawTilt);

        Serial.printf("[TX] raw=(%4d,%4d)  spd=(%4d,%4d)\n",
                      rawPan, rawTilt, pkt.pan_speed, pkt.tilt_speed);
    }

    esp_now_send(RECEIVER_MAC, (uint8_t *)&pkt, sizeof(pkt));
}
