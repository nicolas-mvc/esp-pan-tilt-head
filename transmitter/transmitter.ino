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

// ADC is 12-bit: 0–4095, mapped to per-axis pulse width ranges below
static constexpr int ADC_MIN = 0;
static constexpr int ADC_MAX = 4095;
static constexpr int PAN_PULSE_MIN = 950;   // microseconds
static constexpr int PAN_PULSE_MAX = 2130;  // microseconds
static constexpr int TILT_PULSE_MIN = 1000; // microseconds
static constexpr int TILT_PULSE_MAX = 1500; // microseconds

// Moving average filter to reduce noise (number of samples to average)
static constexpr int FILTER_SIZE = 9;

// Secondary moving average after deadband to smooth the final output.
static constexpr int OUTPUT_FILTER_SIZE = 9;

// Deadband in pulse-width units: the output must move by at least this much
// before the transmitter treats it as a real change.
static constexpr int DEADBAND_US = 8;

// ---------------------------------------------------------------------------
// Shared packet struct — must match receiver.ino exactly
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed))
{
    uint16_t pan_us;  // intended for receiver PAN range (PAN_MIN_US..PAN_MAX_US)
    uint16_t tilt_us; // intended for receiver TILT range (TILT_MIN_US..TILT_MAX_US)
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

// Statefully suppress tiny output changes so jitter does not get transmitted.
class DeadbandFilter
{
private:
    int lastValue = 0;
    bool initialized = false;

public:
    int update(int newValue)
    {
        if (!initialized)
        {
            lastValue = newValue;
            initialized = true;
            return lastValue;
        }

        if (abs(newValue - lastValue) >= DEADBAND_US)
            lastValue = newValue;

        return lastValue;
    }
};

// Secondary moving average to soften jumps after the deadband has locked
// onto a new value.
class OutputMovingAverageFilter
{
private:
    int buffer[OUTPUT_FILTER_SIZE];
    int index = 0;
    int sum = 0;
    bool filled = false;

public:
    OutputMovingAverageFilter()
    {
        for (int i = 0; i < OUTPUT_FILTER_SIZE; i++)
            buffer[i] = 0;
    }

    int update(int newValue)
    {
        if (filled)
            sum -= buffer[index];

        buffer[index] = newValue;
        sum += newValue;
        index = (index + 1) % OUTPUT_FILTER_SIZE;

        if (!filled && index == 0)
            filled = true;

        return sum / OUTPUT_FILTER_SIZE;
    }
};

// Global filter instances
MovingAverageFilter panFilter;
MovingAverageFilter tiltFilter;
DeadbandFilter panDeadband;
DeadbandFilter tiltDeadband;
OutputMovingAverageFilter panOutputFilter;
OutputMovingAverageFilter tiltOutputFilter;

// Map raw ADC value (0-4095) to servo pulse width using provided limits.
uint16_t adcToPulseWidth(int raw, int pulseMin, int pulseMax)
{
    // Constrain to valid ADC range
    raw = constrain(raw, ADC_MIN, ADC_MAX);

    // Linear mapping: ADC_MIN → pulseMin, ADC_MAX → pulseMax
    int pulseWidth = map(raw, ADC_MIN, ADC_MAX, pulseMin, pulseMax);
    return (uint16_t)constrain(pulseWidth, pulseMin, pulseMax);
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

    // ADC pins (34, 35) are inputs; no pinMode needed, but set explicitly for clarity
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

    int panStableUs = panDeadband.update(adcToPulseWidth(filteredPan, PAN_PULSE_MIN, PAN_PULSE_MAX));
    int tiltStableUs = tiltDeadband.update(adcToPulseWidth(filteredTilt, TILT_PULSE_MIN, TILT_PULSE_MAX));

    uint16_t panUs = (uint16_t)panOutputFilter.update(panStableUs);
    uint16_t tiltUs = (uint16_t)tiltOutputFilter.update(tiltStableUs);

    // Build and send packet
    PanTiltPacket pkt;
    pkt.pan_us = panUs;
    pkt.tilt_us = tiltUs;

    esp_now_send(RECEIVER_MAC, (uint8_t *)&pkt, sizeof(pkt));

    Serial.printf("rawPan:%d\trawTilt:%d\tfinalPan:%u\tfinalTilt:%u\n",
                  rawPan, rawTilt, panUs, tiltUs);
}
