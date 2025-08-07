- `RSSI_ref`: Calibrated signal strength at 1 meter.
- `n`: Path loss exponent (empirically tuned between 2.0–2.4).

- Developed filtering stack:
- **Outlier rejection**: Rejects implausible RSSI swings >50% of prior estimate.
- **Moving Average Filter**: Stabilizes short-term readings.
- **Exponential Moving Average (EMA)**: Prioritizes recent data.
- **Kalman Filter**: Implements a probabilistic model to smooth RSSI inputs with adaptive noise covariances.

### Results
- Achieved **<0.75 meter RMSE** in outdoor tests.
- Kalman filter stabilized drift and improved consistency over standard averaging methods.

---

## 🛠️ Architecture & Design Decisions

### Synchronization Strategy
- Initial design used independent clocks on both ESP32s. This led to clock drift and cumulative time offset errors.
- Final architecture centralized the stopwatch on the Start Module. The Finish Module acts only as a trigger and reports back to the Start ESP to finalize the time computation.

### Communication Protocol
- Utilized **ESP-NOW** over MAC address pairing.
- Each event triggers structured data packets with time and event types (`START`, `STOP`).
- Packets are confirmed via callback to ensure reliability.

### Motion Detection
- Sensors: **VL53L1X** laser time-of-flight, configured in long-distance mode (up to 4m).
- Sampling rate: 30 Hz with detection threshold set at ~200 mm for sprint triggering.
- Start Module blocks all LCD output and second trigger detection until STOP is received.

### Display System
- Interface: **I2C** 16x2 LCD, address 0x27.
- Displays:
- "Ready..."
- "Timing..."
- "Time: X.XX sec"
- LCD mount designed to sit flush in case window for visibility under direct light.

### Mechanical Design
- Designed and fabricated **custom 3D-printed enclosures** with:
- Sensor ports aligned for unobstructed field of view.
- LED recesses and LCD mounts.
- Cable strain relief and sealed reset button access.

### Reset Mechanism
- Integrated momentary pushbuttons wired to `EN` and `D13` for manual resets.
- Reset signal logic:
- Normally high (idle), momentary ground pull to reset ESP.

---

## 🖼️ Circuit Diagrams

### 🏁 Finish Module Schematic

![Finish Module Circuit](images/finish_module_schematic.png)

### 🟢 Start Module Schematic (with LCD)

![Start Module Circuit](images/start_module_schematic.png)

---

## ❌ Challenges & Solutions

### ✗ Clock Drift & Timing Errors
- **Problem**: Using two clocks introduced offsets >200ms.
- **Solution**: Re-architected to centralize timing logic on a single microcontroller. Event timestamps were consolidated to one ESP.

### ✗ ESP32-S2 & OLED Failures
- **Problem**: ESP32-S2 modules failed to initialize consistent WiFi communication; OLED display unresponsive.
- **Solution**: Replaced with standard ESP32 Dev Modules and I2C LCDs. Stability and code compatibility improved.

### ✗ Sensor False Positives
- **Problem**: VL53L1X triggered from ambient objects and runners exiting FOV.
- **Solution**: Tuned detection thresholds and configured clear interrupts.

---

## 🏅 Key Engineering Achievements

- Designed and validated an end-to-end real-time sports timing system.
- Integrated signal processing techniques (Kalman filtering) in a live wireless telemetry system.
- Achieved <±1% error in time measurement under repeated trials.
- Developed calibration-free, modular setup usable without manual field prep.
- Built robust, outdoor-usable enclosures and reset logic from scratch.

---

## 📂 Repository Layout

