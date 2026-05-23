# ⚽ Eco Striker — Ball-Kicking Robot

> **Build Smart. Play Green. Score Big.**  
> A low-cost, recycled-material bipedal robot that detects a football using computer vision and executes a precise kicking motion using inverse kinematics — built for under **984 EGP (~$20)**.

---

## 📸 Overview

**Eco Striker** is a 2-DOF robotic leg that:
- Detects a ball in real time using an ESP32-CAM + YOLOv8
- Converts the ball's pixel position into real-world coordinates (cm)
- Feeds those coordinates into an Inverse Kinematics (IK) solver
- Commands two servo motors to execute a human-like kick (wind-up → power stroke → recovery)

The entire mechanical frame is built from **recycled cardboard, Styrofoam, and wooden sticks** — zero structural cost.

---

## 🏗️ Hardware

| Component | Model | Cost (EGP) |
|-----------|-------|------------|
| Microcontroller + Camera | ESP32-CAM | 750 |
| Servo Motors × 2 | 5G62R (95 EGP each) | 190 |
| Breadboard | Half-size | 35 |
| Wooden Sticks | Structural support | 5 |
| Jumper Wires × 8 | M-F and M-M | 4 |
| Cardboard + Styrofoam | 100% Recycled | 0 |
| **Total** | | **984** |

**Wiring:**
- Hip servo signal → GPIO 14
- Knee servo signal → GPIO 15
- Both servos powered at 5V (external supply)
- Shared GND with ESP32

---

## 🧮 Kinematics

### Forward Kinematics (FK)
Used to validate IK output and compute foot position from joint angles:

```
x = L1·cos(θ1) + L2·cos(θ1 + θ2)
y = L1·sin(θ1) + L2·sin(θ1 + θ2)
```

Where:
- `L1 = 15 cm` (thigh)
- `L2 = 10 cm` (shin)
- `θ1` = hip angle, `θ2` = knee angle relative to thigh

### Inverse Kinematics (IK)
Given a target foot position `(x, y)` from the vision system:

```
cos(θ2) = (x² + y² - L1² - L2²) / (2·L1·L2)      ← Law of Cosines
θ2 = acos(cos(θ2))
θ1 = atan2(y, x) - atan2(L2·sin(θ2), L1 + L2·cos(θ2))
```

FK runs at every step to validate IK output — printed to Serial Monitor. Achieved **0.0000 cm average round-trip FK error** over 644 consecutive IK solves.

### Kick Trajectory (IK Coordinates)

| Phase | IK Target (x, y) | Hip Servo | Knee Servo |
|-------|-----------------|-----------|------------|
| Relax / Stand | center | 90° | 90° |
| Wind-up | (7.5, −3.0) | 150° (back) | 30° (back) |
| Power Stroke | (7.5, 23.0) | 30° (front) | 150° (front) |
| Recovery | center | 90° | 90° |

> Coordinates were derived by running FK on calibrated servo angles to ensure physical accuracy.

---

## 👁️ Computer Vision Pipeline

**Stack:** YOLOv8n + Hough Circle refinement + colour fallback

```
ESP32-CAM stream
       ↓
  YOLOv8 detection (conf ≥ 0.15)
       ↓
  Hough Circle refinement (sub-pixel accuracy)
       ↓
  Temporal confirmation (2 consecutive frames)
       ↓
  Pixel → cm conversion
       ↓
  HTTP /move?x=...&y=... → ESP32 IK solver
       ↓
  HTTP /kick → performKick()
```

**Vision metrics:**
- Average latency per frame: **~64–70 ms**
- Detection reliability: **High** (multi-frame confirmation)
- Ball distance estimation: `distance = (D_real × focal_length) / pixel_width`

---

## 🤖 State Machine (Python)

```
SEARCHING → DETECTING → CONFIRMED → KICKING → COOLDOWN → SEARCHING
```

| State | Description |
|-------|-------------|
| SEARCHING | No ball visible, waiting |
| DETECTING | Ball seen, confirming over frames |
| CONFIRMED | Ball locked, tracking active |
| KICKING | IK coordinates sent, kick triggered |
| COOLDOWN | 2s cooldown before next kick |

A live dashboard runs at `http://localhost:8080` showing state transitions, IK targets, latency, and kick count in real time.

---

## 📁 File Structure

```
├── ball_pipeline.py            # Python vision + state machine + dashboard
├── CameraWebServer/
│   ├── CameraWebServer.ino     # ESP32 main sketch (IK, FK, servo control, WiFi)
│   ├── app_httpd.cpp           # HTTP server (/kick and /move endpoints)
│   ├── board_config.h          # Board pin definitions
│   └── camera_pins.h           # Camera pin definitions
└── README.md
```

---

## 🚀 Setup & Running

### ESP32 (Arduino IDE)

1. Install libraries: `ESP32Servo`, `esp32` board package
2. Set your WiFi credentials in `CameraWebServer.ino`:
   ```cpp
   const char *ssid     = "YOUR_SSID";
   const char *password = "YOUR_PASSWORD";
   ```
3. Upload to ESP32-CAM
4. Open Serial Monitor (115200 baud) to see IP address and IK logs

### Python Vision

```bash
pip install opencv-python ultralytics requests numpy
python ball_pipeline.py
```

Update `ESP_IP` in `ball_pipeline.py` to match your ESP32's IP address.

**Dashboard:** Open `http://localhost:8080` in your browser while the script runs.

---

## 📊 Results Summary

| Metric | Value |
|--------|-------|
| Total IK solves | 644 |
| Avg FK round-trip error | 0.0000 cm |
| Avg vision latency | ~64–70 ms |
| Min vision latency | ~40–48 ms |
| Max vision latency | ~118–128 ms |
| Total project cost | 984 EGP (~$20) |
| Structural material cost | 0 EGP (100% recycled) |

---

## 🛠️ How the Kick Works

```
1. Ball detected within camera FOV
2. Pixel position → real-world cm via focal length formula
3. Python sends /move?x=...&y=... to ESP32
4. ESP32 IK solver computes θ1 (hip) and θ2 (knee)
5. FK validates the result (Serial Monitor logs it)
6. Python sends /kick
7. Leg executes: stand → wind-up → power stroke → relax
   Total power stroke time: 150ms
```

---

## 📄 License

This project is open-source and built entirely on recycled materials and free software. Feel free to use, modify, and improve it.

> *"The future of technology lies not in consuming more resources, but in recycling them smarter."*
