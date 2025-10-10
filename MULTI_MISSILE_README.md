# 🚀 Multi-Missile Tracking System - Quick Reference

## What's New?

We've added **multi-missile tracking** capability to the threat detection system!

## 📁 New Files Created

| File | Purpose |
|------|---------|
| `src/test_camera_manager_multi.py` | Simulates multiple missile launches |
| `run_multi_missile_test.py` | Main runner for multi-missile demo |
| `MULTI_MISSILE_GUIDE.md` | Complete user guide |
| `DETECTION_VS_TRACKING.md` | Explains detection vs tracking pipeline |
| `MULTI_MISSILE_README.md` | This file |

## 🎯 Quick Start

### Run Single Missile Demo (Original)
```powershell
python run_test.py
```
- 1 missile
- 300 frames (10 seconds)
- Simple demonstration

### Run Multi-Missile Demo (New!)
```powershell
python run_multi_missile_test.py
```
- **Custom number of missiles** (1-10)
- **Custom duration** (or auto-detect)
- **Advanced tracking** with multiple targets

## 📊 Comparison

| Feature | Single Demo | Multi Demo |
|---------|-------------|------------|
| **Command** | `python run_test.py` | `python run_multi_missile_test.py` |
| **Missiles** | 1 (fixed) | 1-10 (customizable) |
| **Duration** | 10 seconds | Custom or auto |
| **Launch Timing** | Frame 50 | Staggered (50, 100, 150...) |
| **Tracking** | One tracker | Individual tracker per missile |
| **Map File** | `test_threat_map_final.html` | `test_multi_threat_map_final.html` |
| **Use Case** | Basic demo | Advanced testing |

## 🎬 Demo Options

### Option 1: Quick Test
```powershell
python run_multi_missile_test.py
# Input: 2
# Input: [press Enter for auto]
```

### Option 2: Full Demo
```powershell
python run_multi_missile_test.py
# Input: 5
# Input: 30
```

### Option 3: Stress Test
```powershell
python run_multi_missile_test.py
# Input: 10
# Input: 60
```

## 🔍 Understanding the System

### Detection Pipeline (Happens EVERY frame)
```
IR Hotspot → Visual Confirm → Bayesian Fusion → Decision
```

**Result:** "Should we track this? Yes/No"

### Tracking Pipeline (After detection)
```
UKF Predict → Measure → Update → GPS Convert → Map Update
```

**Result:** Smooth trajectory with position + velocity

### Complete Workflow
```
1. IR camera scans for heat
2. Optical camera verifies visually
3. Bayesian network calculates confidence
4. If confidence > 70%:
   - Assign to nearest tracker OR create new tracker
   - UKF tracks position and velocity
   - Convert to GPS coordinates
   - Update GIS map
```

**Read more:** See `DETECTION_VS_TRACKING.md`

## 📈 What You'll See

### Console Output
```
🚀 MISSILE #1 LAUNCHED!
   Frame: 50 (1.7s)

🚨 NEW MISSILE DETECTED - TRACK #1
   Confidence: 99.00%

🚀 MISSILE #2 LAUNCHED!
   Frame: 100 (3.3s)

🚨 NEW MISSILE DETECTED - TRACK #2
   Confidence: 99.00%

📊 Frame 150 | Active Tracks: 2 | Total Detections: 87

MULTI-MISSILE TRACKING STATISTICS
Total Frames:          750
Missiles Tracked:      5
Max Simultaneous:      4
```

### Video Windows
- **IR Camera:** White hotspots with red tracking circles
- **Optical Camera:** Colored missiles with green tracking circles
- **Labels:** Track IDs (T1, T2, T3...)

### Web Map (http://localhost:5000)
- **Red markers:** Launch points
- **Blue markers:** Current positions
- **Red lines:** Flight trajectories
- **Interactive:** Zoom, pan, layer control

## 🎓 Key Features

### 1. Custom Missile Count
Track anywhere from 1 to 10 missiles simultaneously

### 2. Custom Duration
- **Manual:** Specify seconds (e.g., 30)
- **Auto:** Tracks until all missiles exit or max 30s

### 3. Staggered Launches
Missiles launch at intervals for realistic multi-target scenario

### 4. Individual Tracking
Each missile gets its own:
- UKF tracker
- Trajectory history
- Color coding
- Statistics

### 5. Smart Data Association
System automatically:
- Assigns detections to correct tracker
- Creates new tracker for new missile
- Handles tracker lifecycle

## 📚 Documentation

| Document | What It Covers |
|----------|----------------|
| `MULTI_MISSILE_GUIDE.md` | Complete user guide with examples |
| `DETECTION_VS_TRACKING.md` | Pipeline explanation |
| `DEMO_READY.md` | Single-missile demo guide |
| `PRESENTATION_DEMO.md` | Presentation tips |
| `QUICKSTART.md` | Production deployment |

## 🔧 Technical Details

### Multi-Target Tracking
- **Algorithm:** Nearest neighbor data association
- **Distance threshold:** 50 pixels
- **Track creation:** Automatic when no close match
- **UKF per missile:** Independent state estimation

### Physics Simulation
- **Gravity:** 0.1 px/frame²
- **Random velocities:** 2-4 px/frame horizontal, 3-5 px/frame vertical
- **Launch positions:** Distributed (left, center, right)

### Performance
- **Frame rate:** 30 FPS
- **Processing:** Real-time
- **Map updates:** Every 30 frames (1 second)
- **Max missiles:** 10 (recommended 1-5 for clarity)

## ❓ FAQ

**Q: Which demo should I use for presentation?**
- Single missile (`run_test.py`): Clear, simple, easy to follow
- Multi-missile (`run_multi_missile_test.py`): Impressive, shows scalability

**Q: Can I track more than 10 missiles?**
- Code supports it, but display gets cluttered
- Recommend 3-5 for best visualization

**Q: What's the difference between detection and tracking?**
- **Detection:** Finds NEW missiles (every frame)
- **Tracking:** Follows EXISTING missiles (smooth trajectory)
- See `DETECTION_VS_TRACKING.md` for detailed explanation

**Q: Why do I see so many blue markers on the map?**
- Each marker = one tracking update
- Shows high precision and continuous tracking
- Normal for 100+ tracking frames per missile

**Q: Can this run on Raspberry Pi?**
- Yes! Replace `TestCameraManagerMulti` with real camera feeds
- See `QUICKSTART.md` for production deployment

## 🎯 Next Steps

### 1. Try the Single Missile Demo
```powershell
python run_test.py
```
Get familiar with basic detection and tracking

### 2. Try Multi-Missile with 2-3 Missiles
```powershell
python run_multi_missile_test.py
# Input: 3
# Input: 20
```
See how system handles multiple targets

### 3. Read the Documentation
- `DETECTION_VS_TRACKING.md` - Understand the pipeline
- `MULTI_MISSILE_GUIDE.md` - Learn all features

### 4. Check the Map
Open `test_multi_threat_map_final.html` in browser to see complete trajectories

## ✅ Summary

You now have **TWO demo modes**:

### Single Missile Demo
- **File:** `run_test.py`
- **Best for:** Clear demonstrations, explaining algorithms
- **Duration:** 10 seconds, fixed
- **Complexity:** Simple

### Multi-Missile Demo
- **File:** `run_multi_missile_test.py`
- **Best for:** Showcasing capability, stress testing
- **Duration:** Customizable
- **Complexity:** Advanced

Both implement the **complete detection → tracking pipeline** with:
- ✅ IR hotspot detection
- ✅ Visual confirmation
- ✅ Bayesian fusion
- ✅ UKF tracking
- ✅ GIS visualization

---

**Choose the demo that fits your needs and enjoy! 🚀**
