# Threat Detection & Tracking System

## 🎯 Project Overview

This project implements a proof-of-concept (PoC) system for detecting shelling launches, calculating real-time trajectories, and visualizing flight paths. The system uses a multi-sensor fusion approach combining infrared (IR) and optical cameras to reliably detect launch events while minimizing false alarms.

### Core Capabilities

1. **Shelling Launch Detection**: Uses IR and optical cameras with Bayesian fusion
2. **Per-Frame Trajectory Calculation**: Employs an Unscented Kalman Filter (UKF) for state estimation
3. **GIS Flight Path Visualization**: Real-time map-based trajectory display

---

## 📁 Project Structure

```
btp01/
├── src/                          # Source code
│   ├── camera_manager.py         # Camera interface (production)
│   ├── ir_processor.py           # IR hotspot detection
│   ├── optical_processor.py      # Optical motion detection
│   ├── fusion_engine.py          # Bayesian belief network
│   ├── ukf_tracker.py            # Unscented Kalman Filter tracker
│   ├── coordinate_transformer.py # Pixel to GPS conversion
│   ├── gis_plotter.py            # Map visualization
│   ├── web_server.py             # Flask web interface
│   ├── main.py                   # Production main program
│   ├── test_camera_manager.py    # Simulated cameras (demo)
│   └── test_main.py              # Demo main program
│
├── run_production.py             # Production runner (Raspberry Pi)
├── run_test.py                   # Demo runner (Windows)
├── requirements.txt              # Raspberry Pi dependencies
└── requirements_windows.txt      # Windows demo dependencies
```

---

## 🖥️ Two Deployment Modes

### 1. **Production Mode** (Raspberry Pi)
- Uses **real hardware cameras** (IR + Optical)
- Deployed on Raspberry Pi 4/5
- For actual field testing and deployment

### 2. **Demo/Test Mode** (Windows)
- Uses **simulated camera feeds**
- Synthetic missile launch animation
- For presentations and algorithm validation
- Runs on Windows/Mac/Linux

---

## 🚀 Quick Start Guide

### Option A: Running the Demo on Windows (Recommended for Presentation)

#### Prerequisites
- Windows 10/11
- Python 3.9 or later
- Webcam (optional, not used but OpenCV may check)

#### Installation Steps

1. **Clone or extract the project**
```powershell
cd d:\btp01
```

2. **Create a virtual environment**
```powershell
python -m venv venv
.\venv\Scripts\activate
```

3. **Install dependencies**
```powershell
pip install -r requirements_windows.txt
```

4. **Run the demo**
```powershell
python run_test.py
```

#### What to Expect
- Two windows will open showing simulated IR and Optical camera feeds
- A synthetic missile will launch around frame 50
- The system will detect, track, and plot the trajectory
- A web server starts at `http://localhost:5000` showing the live map
- Press `q` to quit early or let it run for ~300 frames

#### Demo Output
- **Console**: Detection alerts and tracking statistics
- **Video Windows**: Annotated IR and optical feeds
- **Web Browser**: Real-time GIS map at `http://localhost:5000`
- **HTML Files**: `test_threat_map_final.html` (final trajectory)

---

### Option B: Running on Raspberry Pi (Production)

#### Hardware Requirements
- Raspberry Pi 4 (4GB+) or Raspberry Pi 5
- IR Camera Module (e.g., FLIR Lepton 3.5)
- Optical Camera (Raspberry Pi Camera Module 3 or USB webcam)
- Power supply (USB-C, 3A+)
- MicroSD card (32GB+, Class 10)

#### Software Setup

1. **Prepare Raspberry Pi**
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install -y python3-pip python3-venv
sudo apt install -y libopencv-dev python3-opencv
sudo apt install -y libatlas-base-dev libhdf5-dev
```

2. **Enable Camera Interfaces**
```bash
sudo raspi-config
# Navigate to: Interface Options -> Camera -> Enable
# Reboot if prompted
```

3. **Clone/Transfer Project**
```bash
cd ~
# Transfer your project files to the Pi
# OR clone from repository
```

4. **Create Virtual Environment**
```bash
cd ~/btp01
python3 -m venv venv
source venv/bin/activate
```

5. **Install Python Dependencies**
```bash
pip install -r requirements.txt
```

6. **Configure Camera Indices**

Edit `src/main.py` and adjust camera indices if needed:
```python
config = {
    'ir_camera_index': 1,      # Check with `ls /dev/video*`
    'optical_camera_index': 0,  # Adjust based on your setup
    ...
}
```

7. **Test Cameras**
```bash
# List available cameras
ls /dev/video*

# Test with simple capture
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera FAIL')"
```

8. **Run the Production System**
```bash
python3 run_production.py
```

#### Production Operation
- System will initialize both cameras
- Web interface available at `http://<raspberry-pi-ip>:5000`
- Access from any device on the same network
- Press `q` in the display window to shutdown
- Final map saved as `threat_map_final.html`

---

## 🧠 System Architecture

### Stage 1: Threat Detection

```
IR Camera ──────────> IR Processor ────────┐
                      (Hotspot Detection)    │
                                             ├──> Bayesian Fusion ──> Launch Decision
Optical Camera ────> Optical Processor ─────┘
                     (Motion Detection)
```

**Bayesian Belief Network**:
- Combines evidence from both sensors
- Outputs probability of launch event
- Threshold: 95% confidence (production), 90% (demo)

### Stage 2: Tracking

```
Detected Position ──> UKF Tracker ──> State Estimate (x, vx, y, vy)
                      (Predict + Update)
```

**UKF Features**:
- 4D state: [x, y, vx, vy]
- Handles non-linear projectile dynamics
- Filters measurement noise

### Stage 3: Visualization

```
Pixel Coords ──> Coordinate Transformer ──> GPS Coords ──> GIS Plotter ──> Web Map
                 (Camera calibration)        (Lat/Lon)     (Folium)
```

---

## 📊 Configuration Parameters

### Key Settings (src/main.py or src/test_main.py)

```python
config = {
    # Camera Settings
    'ir_camera_index': 1,           # IR camera device ID
    'optical_camera_index': 0,      # Optical camera device ID
    
    # Detection Thresholds
    'ir_threshold': 220,            # IR brightness threshold (0-255)
    'min_hotspot_area': 100,        # Minimum hotspot size (pixels)
    'confidence_threshold': 0.95,   # Launch detection confidence
    
    # Geographic Location
    'base_lat': 28.6139,           # Platform latitude (New Delhi)
    'base_lon': 77.2090,           # Platform longitude
    'altitude': 100.0,             # Platform height (meters)
    
    # Performance
    'fps': 30,                     # Target frame rate
    'web_server_port': 5000        # Web interface port
}
```

### Tuning Guide

**If too many false alarms:**
- Increase `ir_threshold` (220 → 230)
- Increase `min_hotspot_area` (100 → 150)
- Increase `confidence_threshold` (0.95 → 0.98)

**If missing real launches:**
- Decrease `ir_threshold` (220 → 200)
- Decrease `min_hotspot_area` (100 → 50)
- Decrease `confidence_threshold` (0.95 → 0.90)

---

## 🔧 Troubleshooting

### Demo Mode (Windows)

**Issue**: `ModuleNotFoundError: No module named 'pgmpy'`
```powershell
pip install pgmpy
```

**Issue**: Windows opens but are black
- This is normal initially, wait for the simulated launch (frame 50)

**Issue**: Web server not accessible
- Check firewall settings
- Try `http://127.0.0.1:5000` instead

### Production Mode (Raspberry Pi)

**Issue**: Camera not detected
```bash
# Check camera connections
ls /dev/video*

# Test camera
raspistill -o test.jpg  # For Pi Camera
ffmpeg -f v4l2 -i /dev/video0 -frames 1 test.jpg  # For USB camera
```

**Issue**: `ImportError: libopencv...`
```bash
sudo apt install python3-opencv libopencv-dev
```

**Issue**: Low frame rate
- Reduce `fps` in config
- Lower camera resolution
- Use Raspberry Pi 5 for better performance

**Issue**: Web server not accessible from other devices
- Ensure Pi and client on same network
- Check Pi firewall: `sudo ufw allow 5000`
- Use Pi's IP address, not `localhost`

---

## 📈 Performance Metrics

### Expected Performance

| Metric | Raspberry Pi 4 | Raspberry Pi 5 | Windows Demo |
|--------|---------------|---------------|--------------|
| Frame Rate | 15-25 fps | 25-30 fps | 30 fps |
| Detection Latency | <100ms | <50ms | <30ms |
| Tracking Accuracy | ±5 pixels | ±3 pixels | ±2 pixels |
| Map Update Rate | 2-5 fps | 5-10 fps | 10 fps |

---

## 🧪 Testing the System

### Demo Validation Checklist

1. ✅ Run `python run_test.py`
2. ✅ Verify two video windows open
3. ✅ Observe missile launch around frame 50
4. ✅ Confirm "LAUNCH DETECTED" message in console
5. ✅ Check tracking crosshairs appear
6. ✅ Open `http://localhost:5000` in browser
7. ✅ Verify trajectory appears on map
8. ✅ Confirm final map file is created

### Production Validation Checklist

1. ✅ Cameras physically connected
2. ✅ Run `python run_production.py`
3. ✅ Both camera feeds display
4. ✅ Test with heat source (candle, lighter)
5. ✅ Verify IR hotspot detection
6. ✅ Confirm visual confirmation works
7. ✅ Check web interface accessibility
8. ✅ Validate GPS coordinates are reasonable

---

## 📚 Dependencies Explained

| Library | Purpose | Production | Demo |
|---------|---------|-----------|------|
| opencv-python | Image processing & camera I/O | ✅ | ✅ |
| numpy | Numerical operations | ✅ | ✅ |
| filterpy | UKF implementation | ✅ | ✅ |
| pgmpy | Bayesian belief networks | ✅ | ✅ |
| folium | Interactive map generation | ✅ | ✅ |
| flask | Web server | ✅ | ✅ |
| scipy | Scientific computing | ✅ | ✅ |
| pyproj | Coordinate transformations | ✅ | ✅ |

---

## 🎓 Algorithm Details

### IR Hotspot Detection
1. Convert frame to grayscale
2. Apply binary threshold (>220)
3. Find contours
4. Filter by minimum area
5. Calculate centroid

### Optical Motion Detection
1. Define ROI around IR hotspot
2. Apply background subtraction
3. Count foreground pixels
4. Return true if significant motion

### Bayesian Fusion
- **Prior**: P(Launch) = 0.0001
- **Likelihood**: P(IR|Launch) = 0.99, P(Visual|Launch) = 0.95
- **Posterior**: Calculated via Variable Elimination

### UKF Tracker
- **State**: [x, vx, y, vy]
- **Process Model**: Constant velocity + gravity
- **Measurement**: [x, y] positions only
- **Update Rate**: 30 Hz

### Coordinate Transform
1. Calculate camera ground coverage
2. Convert pixels to meters
3. Apply geographic offset
4. Return (latitude, longitude)

---

## 🔒 Safety & Security

### For Raspberry Pi Deployment

1. **Network Security**
   - Use firewall: `sudo ufw enable`
   - Restrict web server access: Change `host='0.0.0.0'` to `host='127.0.0.1'`
   - Use VPN for remote access

2. **Physical Security**
   - Secure camera mounts
   - Protect from weather (if outdoor)
   - Backup power supply

3. **Data Privacy**
   - Maps saved locally only
   - No cloud upload in current version
   - Consider encryption for sensitive deployments

---

## 🛠️ Future Enhancements

### Planned Features
- [ ] Multi-object tracking
- [ ] Predicted impact point calculation
- [ ] Alert notifications (SMS/Email)
- [ ] Video recording on detection
- [ ] Machine learning-based detection
- [ ] 3D trajectory estimation
- [ ] Drone integration
- [ ] Cloud dashboard

### Hardware Upgrades
- [ ] Higher resolution cameras
- [ ] NVIDIA Jetson for ML acceleration
- [ ] GPS module for automatic positioning
- [ ] IMU for camera stabilization

---

## 📞 Support & Contribution

### Getting Help
1. Check this README thoroughly
2. Review console error messages
3. Verify all dependencies installed
4. Check camera connections (production)

### Reporting Issues
Include:
- Operating system (Windows/Raspberry Pi OS)
- Python version
- Full error traceback
- Config parameters used

---

## 📄 License

This is an academic/research project. Please cite appropriately if used in publications.

---

## 🙏 Acknowledgments

Built using:
- OpenCV (computer vision)
- FilterPy (Kalman filtering)
- pgmpy (Bayesian networks)
- Folium (mapping)

Inspired by defense tracking systems and academic research in multi-sensor fusion.

---

## 📝 Quick Reference Commands

### Windows Demo
```powershell
# Setup
python -m venv venv
.\venv\Scripts\activate
pip install -r requirements_windows.txt

# Run
python run_test.py

# View map
http://localhost:5000
```

### Raspberry Pi Production
```bash
# Setup
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Run
python3 run_production.py

# View map
http://<pi-ip-address>:5000
```

---

**Last Updated**: October 2025  
**Version**: 1.0.0  
**Status**: Proof of Concept
