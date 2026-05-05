## Plan: ESP32 ESP-NOW Pan-Tilt Head

**Rate-control** joystick drives servo speed. Transmitter sends speed commands over ESP-NOW; receiver integrates them into position and writes to servos. One-way, Arduino IDE, two sketch folders.

---

### Project Structure

```
esp-pan-tilt-head/
├── transmitter/
│   └── transmitter.ino
└── receiver/
    └── receiver.ino
```

---

### Phase 1 — Shared Data Contract

Define this packet struct identically in **both** `.ino` files. Keeping it `__attribute__((packed))` ensures consistent byte layout over the wire:

| Field        | Type     | Range      | Meaning               |
| ------------ | -------- | ---------- | --------------------- |
| `pan_speed`  | `int8_t` | −100 … 100 | Rate of pan movement  |
| `tilt_speed` | `int8_t` | −100 … 100 | Rate of tilt movement |
| `reset`      | `bool`   | 0 / 1      | Return servos to 90°  |

---

### Phase 2 — Hardware Wiring

**Transmitter ESP32**

- Joystick X → GPIO36 (ADC1_CH0), Y → GPIO39 (ADC1_CH3), VCC → 3.3V
- Reset button → GPIO15 + internal pull-up (active LOW)

**Receiver ESP32**

- Pan servo signal → GPIO18, Tilt servo signal → GPIO19
- Servo VCC → dedicated **5V rail** (not the ESP32's 3.3V pin — servos draw too much current)
- Shared GND between ESP32 and servo power rail

---

### Phase 3 — Receiver (`receiver/receiver.ino`)

1. `WiFi.mode(WIFI_STA)` → print MAC address to Serial (needed for transmitter config)
2. `esp_now_init()` + register receive callback
3. Callback stores incoming `PanTiltPacket` into a volatile global and sets a `newData` flag
4. Main loop (runs every ~10 ms using `millis()`):
   - If `reset` flag → set `panPos = tiltPos = 90.0f`, clear flag
   - Otherwise: `panPos += pan_speed * dt * SPEED_SCALE`, clamped to `[0, 180]`; same for tilt
   - Write positions to `ESP32Servo` instances
5. **Library**: `ESP32Servo` (install from Arduino Library Manager)

---

### Phase 4 — Transmitter (`transmitter/transmitter.ino`)

1. Hardcode receiver MAC address (obtained from Phase 3 serial output)
2. `WiFi.mode(WIFI_STA)` → `esp_now_init()` → register peer
3. Main loop (every 20 ms):
   - Read ADC: 12-bit (0–4095), center ≈ 2048
   - Apply **deadzone**: if `|raw − 2048| < DEAD` → speed = 0 (prevents drift at rest)
   - Map remaining range to −100 … 100
   - Read reset button (debounced)
   - Call `esp_now_send()` with the packet
4. Optional: print speed values to Serial for debugging

---

### Phase 5 — Tuning Constants

These go in `receiver.ino` and can be adjusted at runtime via Serial or just recompiled:

| Constant         | Starting Value        | Effect                   |
| ---------------- | --------------------- | ------------------------ |
| `SPEED_SCALE`    | `0.5` (°/ms per unit) | Overall movement speed   |
| `DEAD`           | `200`                 | Joystick deadzone size   |
| `LOOP_MS`        | `10`                  | Receiver update interval |
| `TX_INTERVAL_MS` | `20`                  | Transmitter send rate    |

---

### Relevant Files to Create

- `transmitter/transmitter.ino` — ADC read, deadzone, ESP-NOW send
- `receiver/receiver.ino` — ESP-NOW receive callback, servo integration loop

---

### Verification Steps

1. Flash `receiver.ino` first → open Serial Monitor → copy the printed MAC address
2. Paste MAC into `transmitter.ino`, flash transmitter
3. Serial monitor on transmitter: confirm ADC values and `esp_now_send` returns `ESP_OK`
4. Serial monitor on receiver: confirm `pan_speed` / `tilt_speed` values arriving correctly
5. Physical: joystick deflect → servos move continuously; release → servos stop; button → both snap to 90°

---

### Decisions / Scope

- The receiver MAC is hardcoded (no pairing handshake) — simplest approach for a fixed two-device setup
- No feedback channel from receiver — if you later want current position displayed on a screen, a second ESP-NOW peer registration in reverse direction would handle it
- Servo library: `ESP32Servo` (not the standard `Servo.h` which conflicts with ESP32 timers)
