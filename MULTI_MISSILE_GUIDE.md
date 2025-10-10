# Multi-Missile Tracking System - User Guide

## 🎯 Overview

The **Multi-Missile Tracking System** is an enhanced version of the threat detection demo that supports:

- ✅ **Multiple simultaneous missile tracking** (1-10 missiles)
- ✅ **Custom tracking duration** (auto-detect or manual)
- ✅ **Full detection → tracking pipeline**
- ✅ **Individual trajectory visualization** per missile
- ✅ **Real-time GIS mapping** of all trajectories

---

## 🚀 How to Run

### Basic Usage
```powershell
python run_multi_missile_test.py
```

You'll be prompted for:
1. **Number of missiles** (1-10, default: 3)
2. **Tracking duration** (in seconds, or leave empty for auto-detect)

### Example Session
```
How many missiles to track? (1-10, default=3): 5
Tracking duration in seconds? (leave empty for auto): 25

🎯 Configuration:
   Missiles: 5
   Duration: 25s

Press Enter to start multi-missile demo...
```

---

## 📊 What It Does

### 1. Detection Phase
The system implements a complete **3-stage detection pipeline**:

```
IR Hotspot Detection → Visual Confirmation → Bayesian Fusion
```

#### Stage 1: IR Hotspot Detection
- Scans IR camera feed for thermal signatures
- Identifies potential missile launches
- Filters out false positives (buildings, etc.)

#### Stage 2: Visual Confirmation
- Optical camera verifies IR detection
- Looks for motion and visual cues (smoke trail)
- Reduces false alarms

#### Stage 3: Bayesian Fusion
- Combines IR + Optical evidence using Bayesian Belief Network
- Calculates launch probability (confidence score)
- Only tracks if confidence > 70%

### 2. Tracking Phase
Once detection occurs:

```
Detection → Tracker Assignment → UKF Tracking → GIS Update
```

#### Tracker Assignment
- System maintains **individual UKF tracker per missile**
- New detections assigned to closest existing tracker (within 50 pixels)
- If no close tracker exists, **creates new tracker** for new missile

#### UKF Tracking
- **Unscented Kalman Filter** predicts trajectory
- Handles non-linear motion (gravity, air resistance)
- Tracks position AND velocity
- Continues tracking even if detection temporarily lost

#### GIS Visualization
- Each missile gets unique color on map
- Trajectories updated in real-time
- Launch points marked with red markers
- Current positions shown with blue markers

---

## 🎬 Demo Behavior

### Missile Launches
Missiles are launched at **staggered intervals**:

```python
Missile #1: Frame 50  (1.67s)
Missile #2: Frame 100 (3.33s)
Missile #3: Frame 150 (5.00s)
...
```

Launch interval calculated as:
- **Manual duration**: Spread across first 1/3 of duration
- **Auto mode**: 50 frames between launches (~1.67s at 30 FPS)

### Missile Physics
Each missile has:
- **Random launch position** (left, center, or right)
- **Random velocity** (2-4 pixels/frame horizontal, 3-5 px/frame vertical)
- **Gravity** (0.1 px/frame² downward acceleration)
- **Unique color** for visual identification

### Auto-Detect Mode
If no duration specified:
- Tracks until all missiles exit frame or become inactive
- Maximum 30 seconds (safety limit)
- Adds 100 extra frames after last missile goes inactive

---

## 🖥️ What You'll See

### Video Windows

#### IR Camera Window
- **White hotspots** = Missile heat signatures
- **Red circles** = Tracked targets
- **Yellow labels** = Track IDs (T1, T2, etc.)
- **Status overlay** = Active track count

#### Optical Camera Window
- **Colored missiles** = Each missile has unique color
- **Gray smoke trails** = Visual confirmation cues
- **Green circles** = Tracked targets
- **Labels** = Track numbers (Track #1, #2, etc.)

### Console Output

#### Initialization
```
🎯 MULTI-MISSILE SIMULATION INITIALIZED
Number of missiles:    5
Tracking duration:     25.0 seconds (750 frames)
Launch interval:       50 frames (1.7s)
```

#### Missile Launch
```
🚀 MISSILE #1 LAUNCHED!
   Frame: 50 (1.7s)
   Position: (160, 430)
   Velocity: (3.2, -4.1) px/frame
```

#### New Track Detection
```
🚨 NEW MISSILE DETECTED - TRACK #1
   Confidence: 99.00%
   Time: 2025-10-11 00:25:43
   Position: (165, 420)
   Frame: 52
```

#### Tracking Updates
```
📊 Frame 100 | Active Tracks: 2 | Total Detections: 48
📊 Frame 120 | Active Tracks: 3 | Total Detections: 72
```

#### Final Statistics
```
MULTI-MISSILE TRACKING STATISTICS
Total Frames:          750
Total Detections:      345
Missiles Tracked:      5
Max Simultaneous:      4
Tracking Duration:     25.0s

PER-MISSILE TRACKING DETAILS
  Track #1:
    Detection Time:  2025-10-11 00:25:43
    Tracking Frames: 156
    Trajectory Pts:  156
```

### Web Interface (http://localhost:5000)

#### Map Features
- **Red markers** = Launch points for each missile
- **Blue markers** = Last known positions
- **Red trajectory lines** = Flight paths
- **Layer control** = Switch between map tiles
- **Interactive** = Zoom, pan, click for details

#### Files Generated
- `test_multi_threat_map.html` - Updated during tracking
- `test_multi_threat_map_final.html` - Complete trajectories after demo

---

## 🔬 Technical Details

### Detection → Tracking Pipeline

```
EVERY FRAME:
  1. IR Processor scans for hotspots
  2. If hotspot found:
     a. Optical Processor verifies
     b. Fusion Engine calculates confidence
     c. If confidence > 70%:
        - Find closest tracker (within 50px)
        - If found: Update existing tracker
        - If not found: Create new tracker
  3. All active trackers:
     a. Update UKF with measurement/prediction
     b. Convert to geographic coordinates
     c. Add to GIS trajectory
  4. Save map every 30 frames
```

### Multi-Target Tracking Strategy

#### Data Association
- **Nearest neighbor** approach
- Distance threshold: 50 pixels
- Prevents track swapping
- Creates new track if no close match

#### Tracker Management
```python
trackers = {
    0: MultiMissileTracker(id=0),  # Track #1
    1: MultiMissileTracker(id=1),  # Track #2
    2: MultiMissileTracker(id=2),  # Track #3
    ...
}
```

Each tracker maintains:
- Individual UKF state
- Trajectory history
- Detection timestamp
- Tracking frame count
- Color assignment

#### UKF State Vector
```python
state = [x, vx, y, vy]
# x, y  = Position in pixels
# vx, vy = Velocity in pixels/frame
```

---

## 📈 Performance Metrics

### Typical Performance (3 Missiles, 30s)

| Metric | Value |
|--------|-------|
| Frame Rate | 30 FPS |
| Total Frames | 900 |
| Detection Rate | ~50 detections/missile |
| Tracking Frames | ~150 frames/missile |
| Max Simultaneous | 3 missiles |
| Map Updates | 30 times (every 30 frames) |

### Scalability

| Missiles | Recommended Duration | Expected Tracks |
|----------|---------------------|-----------------|
| 1-2 | 15-20s | 2 |
| 3-5 | 20-30s | 3-5 |
| 6-8 | 30-40s | 4-8 |
| 9-10 | 40-60s | 5-10 |

---

## 🎓 Understanding the Output

### Why Multiple Blue Markers on Map?

Each blue marker = one tracked position update (every frame that missile is tracked).

With 150 tracking frames per missile, you get **150 blue markers per missile**.

This shows:
- ✅ Continuous tracking (no gaps)
- ✅ High precision (dense updates)
- ✅ Complete trajectory coverage

### Detection vs. Tracking

| Stage | What It Means |
|-------|---------------|
| **Detection** | IR hotspot found + visually confirmed + high confidence |
| **Tracking** | UKF actively following the target (even if temporarily not detected) |

**Key Point**: Tracking continues even if detection is lost for a few frames (UKF prediction).

### Confidence Score

```
99% confidence = IR hotspot + Visual confirmation
85% confidence = IR hotspot + Partial visual
70% confidence = IR hotspot + No visual (threshold)
<70% = Rejected (not tracked)
```

---

## 🛠️ Customization

### Modify Launch Interval
Edit `run_multi_missile_test.py`:
```python
launch_interval = 50  # Frames between launches (default: 50)
```

### Change Tracking Sensitivity
Edit configuration in `run_multi_missile_test.py`:
```python
config = {
    'confidence_threshold': 0.7,  # Lower = more sensitive (0.5-0.9)
    'ir_temp_threshold': 200,     # IR detection threshold
    'optical_motion_threshold': 25, # Motion detection threshold
}
```

### Adjust Map Update Frequency
```python
system.run(display=True, save_interval=30)  # Change 30 to 60 for less frequent
```

---

## 🎯 Use Cases

### 1. Algorithm Testing
Test detection and tracking algorithms with varying:
- Number of targets
- Launch intervals
- Tracking duration

### 2. Performance Evaluation
Measure system performance under multi-target scenarios:
- Track association accuracy
- False positive rate
- Tracking continuity

### 3. Presentation Demo
Impressive visual demonstration of:
- Multi-target tracking capability
- Real-time GIS visualization
- Bayesian fusion in action

### 4. System Validation
Verify that production system can handle:
- Multiple simultaneous threats
- Sustained tracking duration
- Complex trajectories

---

## ❓ FAQ

**Q: Why do some missiles disappear from tracking?**
A: Missiles exit the frame boundaries or tracking confidence drops below threshold.

**Q: Can I track more than 10 missiles?**
A: Code supports it, but visual display becomes cluttered. Recommended max: 10.

**Q: What if two missiles are very close?**
A: Data association uses 50px threshold. Closer missiles may be tracked as one.

**Q: How accurate is the tracking?**
A: UKF provides pixel-level accuracy (~1-2 pixel error) under ideal conditions.

**Q: Can I run this on Raspberry Pi?**
A: Yes! Replace `TestCameraManagerMulti` with real camera feeds. See production deployment guide.

---

## 🔄 Comparison: Single vs. Multi-Missile

| Feature | Single (`run_test.py`) | Multi (`run_multi_missile_test.py`) |
|---------|----------------------|----------------------------------|
| Missiles | 1 fixed | 1-10 customizable |
| Duration | 300 frames (10s) | Custom or auto |
| Tracking | One UKF tracker | Individual trackers per missile |
| Launch | Frame 50 | Staggered launches |
| Map | Single trajectory | Multiple color-coded trajectories |
| Use Case | Basic demo | Advanced testing |

---

## 🚀 Quick Start Examples

### Example 1: Quick Test (2 missiles, auto duration)
```powershell
python run_multi_missile_test.py
# Input: 2
# Input: [Enter]
```

### Example 2: Full Demo (5 missiles, 30 seconds)
```powershell
python run_multi_missile_test.py
# Input: 5
# Input: 30
```

### Example 3: Stress Test (10 missiles, 60 seconds)
```powershell
python run_multi_missile_test.py
# Input: 10
# Input: 60
```

---

## 📁 Files

### Created Files
- `src/test_camera_manager_multi.py` - Multi-missile camera simulator
- `run_multi_missile_test.py` - Main multi-missile demo runner
- `MULTI_MISSILE_GUIDE.md` - This documentation

### Generated Files
- `test_multi_threat_map.html` - Real-time map
- `test_multi_threat_map_final.html` - Final complete map

---

## ✅ System Requirements

Same as single-missile demo:
- Python 3.9+
- OpenCV, NumPy, SciPy
- FilterPy, pgmpy, Folium
- Flask, pandas

Already installed if you ran `pip install -r requirements_windows.txt`

---

**Ready to track multiple missiles? Run `python run_multi_missile_test.py` and see it in action! 🚀**
