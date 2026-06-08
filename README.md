# Dual Camera Raspberry Pi Hub

**Release:** v0.1.0 — 2026-06-08 — initial public import


Professional Ground Control Station for SIYI gimbal systems with dual-camera support and low-latency optimization.

## ⚡ Quick Start

### Minimum Latency (Recommended)
```bash
python3 rpi_camera_hub_siyi_same_ui.py --low-latency --kiosk
```
**Result:** 20-30ms video latency + 50 Hz gimbal control

### With Professional OSD
```bash
python3 rpi_camera_hub_siyi_same_ui.py
```
**Result:** 60-80ms video latency + telemetry overlay

### Development Mode
```bash
python3 rpi_camera_hub_siyi_same_ui.py --low-latency --dev-ui
```
**Result:** Low latency + debug information

---

## 🎯 What It Does

- ✅ **Dual RTSP camera streams** (front and rear gimbals)
- ✅ **Professional OSD** (battery, signal quality, temperature)
- ✅ **Gimbal pitch/yaw control** via joystick (UART/serial)
- ✅ **HM30 telemetry integration** (battery, link status)
- ✅ **Network connectivity gating** (black screen until network ready)
- ✅ **Low-latency mode** (optimized for real-time control)
- ✅ **Responsive UI** (Tkinter + async frame processing)
- ✅ **Kiosk deployment** (fullscreen + hidden cursor)

---

## 📊 Performance

| Feature | Normal | Low-Latency | Improvement |
|---------|--------|-------------|------------|
| Video Latency | 60-80ms | 20-30ms | **50-66% ↓** |
| Control Response | ~75ms | ~40ms | **47% ↓** |
| Gimbal Rate | 35 Hz | 50 Hz | **43% ↑** |
| OSD Rendering | 20ms | 0ms | **100% ↓** |

---

## 📖 Documentation

- **[LOW_LATENCY_SUMMARY.md](LOW_LATENCY_SUMMARY.md)** ← **START HERE** for quick overview
- **[LOW_LATENCY_MODE.md](LOW_LATENCY_MODE.md)** - Complete technical details
- **[QUICK_START.md](QUICK_START.md)** - CLI reference and options
- **[OPTIMIZATION_GUIDE.md](OPTIMIZATION_GUIDE.md)** - Performance tuning
- **[PROFESSIONAL_OSD_GUIDE.md](PROFESSIONAL_OSD_GUIDE.md)** - OSD theming and customization

---

## 🚀 Installation

```bash
python3 -m pip install -r requirements.txt
```

2. Start the GUI:

```bash
python3 rpi_gimbal.py
```

Laptop test mode
1. Run the rebuilt SDK file without real hardware:

```bash
python gimbal_sdk.py --mock
```

2. Run a quick non-GUI smoke test:

```bash
python gimbal_sdk.py --mock --headless
```

3. If you want generated video instead of RTSP, add:

```bash
python gimbal_sdk.py --mock --demo-streams
```

3. Run headless for boot:

```bash
python3 rpi_gimbal.py --headless --uart /dev/ttyAMA0 --baud 115200
```

Raspberry Pi notes
- Install `python3-tk` if Tkinter is missing.
- Install `ffmpeg` or a working OpenCV RTSP backend if streams do not open.
- Install `pygame` and connect a joystick/gamepad for stick control.

Joystick mapping
- Stick Y: pitch on the active camera
- Stick X: yaw on Skydroid, ignored on SIYI A2 mini
- Button 0: zoom in
- Button 1: zoom out
- Button 2: light toggle
- Button 3: center
- Button 4: select SIYI
- Button 5: select Skydroid

Systemd
- The included `run_gimbal.service` starts the hub headless.
- Update the `User` and `WorkingDirectory` fields for your Pi.

Protocol notes
- Skydroid zoom uses the `DZM` command from `shdcomm-master`.
- Skydroid light control uses the `EXT` command from `shdcomm-master`.
- SIYI zoom is wired through the documented `0x0F` absolute zoom command, but whether it is available depends on the camera model.
