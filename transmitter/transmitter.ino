/*
 * transmitter.ino — ESP32 Pan-Tilt Head Transmitter
 *
 * Reads two potentiometers (pan and tilt) and sends their mapped
 * positions to the receiver via ESP-NOW.
 *
 * SETUP:
 *   1. Flash receiver.ino first.
 *   2. Verify receiver MAC matches RECEIVER_MAC below.
 *   3. Flash this sketch.
 *
 * WIRING:
 *   Pan potentiometer  → GPIO34 (ADC input)
 *   Tilt potentiometer → GPIO35 (ADC input)
 *   Potentiometer VCC  → 3.3V
 *   Potentiometer GND  → GND
 */

#include <WiFi.h>
#include <esp_now.h>

// ---------------------------------------------------------------------------
// *** Receiver MAC address (must match receiver's hardcoded MAC) ***
// ---------------------------------------------------------------------------
static uint8_t RECEIVER_MAC[6] = {0x08, 0xB6, 0x1F, 0xB8, 0xA3, 0xD0};

// ---------------------------------------------------------------------------
// Pin & timing constants
// ---------------------------------------------------------------------------
static constexpr int PIN_POT_PAN = 34;  // ADC input for pan pot
static constexpr int PIN_POT_TILT = 35; // ADC input for tilt pot

static constexpr int TX_INTERVAL_MS = 20; // send rate (50 Hz)

// ADC is 12-bit: 0–4095, map to servo pulse width range 1000–2000 µs
static constexpr int ADC_MIN = 0;
static constexpr int ADC_MAX = 4095;
static constexpr int PULSE_MIN = 550;  // microseconds (0°)
static constexpr int PULSE_MAX = 2650; // microseconds (180°)

// Moving average filter to reduce noise (number of samples to average)
static constexpr int FILTER_SIZE = 9;

// ---------------------------------------------------------------------------
// Shared packet struct — must match receiver.ino exactly
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    uint16_t pan_us;  // 1000–2000 microseconds (pulse width)
    uint16_t tilt_us; // 1000–2000 microseconds (pulse width)
} PanTiltPacket;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Simple moving average filter to reduce ADC noise
class MovingAverageFilter
{
private:
    int buffer[FILTER_SIZE];
    int index = 0;
    int sum = 0;
    bool filled = false;

public:
    MovingAverageFilter()
    {
        for (int i = 0; i < FILTER_SIZE; i++)
            buffer[i] = 0;
    }

    int update(int newValue)
    {
        if (filled)
            sum -= buffer[index];

        buffer[index] = newValue;
        sum += newValue;
        index = (index + 1) % FILTER_SIZE;

        if (!filled && index == 0)
            filled = true;

        return sum / FILTER_SIZE;
    }
};

// Global filter instances
MovingAverageFilter panFilter;
MovingAverageFilter tiltFilter;

// Map raw ADC value (0-4095) to servo pulse width (1000-2000 µs).
uint16_t adcToPulseWidth(int raw)
{
    // Constrain to valid ADC range
    raw = constrain(raw, ADC_MIN, ADC_MAX);

    // Linear mapping: ADC_MIN → PULSE_MIN, ADC_MAX → PULSE_MAX
    int pulseWidth = map(raw, ADC_MIN, ADC_MAX, PULSE_MIN, PULSE_MAX);
    return (uint16_t)constrain(pulseWidth, PULSE_MIN, PULSE_MAX);
}

// ESP-NOW send callback — logs errors if the peer did not acknowledge.
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
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
    delay(500);

    // ADC pins (14, 15) are input; no pinMode needed, but we can set them anyway
    pinMode(PIN_POT_PAN, INPUT);
    pinMode(PIN_POT_TILT, INPUT);

    WiFi.mode(WIFI_STA);
    Serial.print("[TX] Transmitter MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.print("[TX] Target receiver MAC: ");
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                  RECEIVER_MAC[0], RECEIVER_MAC[1], RECEIVER_MAC[2],
                  RECEIVER_MAC[3], RECEIVER_MAC[4], RECEIVER_MAC[5]);

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

    // Read potentiometers and apply moving average filter
    int rawPan = analogRead(PIN_POT_PAN);
    int rawTilt = analogRead(PIN_POT_TILT);

    int filteredPan = panFilter.update(rawPan);
    int filteredTilt = tiltFilter.update(rawTilt);

    uint16_t panUs = adcToPulseWidth(filteredPan);
    uint16_t tiltUs = adcToPulseWidth(filteredTilt);

    // Build and send packet
    PanTiltPacket pkt;
    pkt.pan_us = panUs;
    pkt.tilt_us = tiltUs;

    esp_now_send(RECEIVER_MAC, (uint8_t *)&pkt, sizeof(pkt));

    Serial.printf("[TX] raw=(%4d,%4d) filt=(%4d,%4d) us=(%4d,%4d)\n",
                  rawPan, rawTilt, filteredPan, filteredTilt, panUs, tiltUs);
}
