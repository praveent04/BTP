# 🚀 QUICK START GUIDE

## For Windows Demo (Presentation)

### 1. Install Python
- Download Python 3.9+ from python.org
- During installation, check "Add Python to PATH"

### 2. Setup Project
Open PowerShell in the project directory:

```powershell
# Create virtual environment
python -m venv venv

# Activate it
.\venv\Scripts\activate

# Install dependencies
pip install -r requirements_windows.txt
```

### 3. Run Demo
```powershell
python run_test.py
```

### 4. Watch the Demo
- **Two windows**: Simulated IR and Optical cameras
- **Missile launches**: Around frame 50
- **Detection**: System identifies and tracks
- **Web map**: Open browser to http://localhost:5000

### 5. Exit
- Press 'q' in video windows, OR
- Press Ctrl+C in terminal

---

## For Raspberry Pi (Production)

### 1. Prepare Hardware
- Connect IR camera to CSI port (or USB)
- Connect optical camera to USB
- Ensure power supply is adequate (3A+)

### 2. Setup Software
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install -y python3-pip python3-venv python3-opencv

# Enable cameras
sudo raspi-config
# Interface Options -> Camera -> Enable -> Reboot
```

### 3. Install Project
```bash
# Navigate to project
cd ~/btp01

# Create environment
python3 -m venv venv
source venv/bin/activate

# Install requirements
pip install -r requirements.txt
```

### 4. Configure Cameras
Check camera device IDs:
```bash
ls /dev/video*
```

Edit `src/main.py` if needed:
```python
config = {
    'ir_camera_index': 1,      # Adjust based on ls output
    'optical_camera_index': 0,
    ...
}
```

### 5. Run System
```bash
python3 run_production.py
```

### 6. Access Web Interface
From any device on the same network:
```
http://<raspberry-pi-ip>:5000
```

Find Pi IP with: `hostname -I`

---

## Troubleshooting

### Demo won't start
```powershell
# Ensure virtual environment is active (see (venv) in prompt)
.\venv\Scripts\activate

# Reinstall dependencies
pip install --upgrade -r requirements_windows.txt
```

### Pi cameras not detected
```bash
# Test cameras
vcgencmd get_camera  # For Pi camera
v4l2-ctl --list-devices  # For USB cameras

# Reboot if needed
sudo reboot
```

### Can't access web interface
- Windows: Try http://127.0.0.1:5000
- Pi: Check firewall: `sudo ufw allow 5000`

---

## Next Steps

1. ✅ Run the demo successfully
2. 📖 Read DEPLOYMENT_README.md for details
3. ⚙️ Adjust configuration parameters
4. 🎯 Test with real hardware (Pi only)
5. 📊 Analyze results and refine

---

**Need help?** Check the full DEPLOYMENT_README.md
