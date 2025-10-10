# Quick Git Commands for This Project

## ✅ Current Status: CLEAN
- Git sees only **44 legitimate files**
- **1,400+ cache files** properly ignored
- Ready to commit!

---

## 🚀 Quick Start - Add Everything

```powershell
# Add all project files
git add .

# Commit with message
git commit -m "Complete threat detection system with multi-missile tracking"

# Push to GitHub
git push origin main
```

---

## 📦 Staged Approach - Add Files by Category

### 1. Add Core Documentation
```powershell
git add .gitignore README.md *.md requirements*.txt
git commit -m "Add project documentation and requirements"
```

### 2. Add Main Source Code
```powershell
git add src/ test/
git add run_*.py validate_*.py verify_*.py
git commit -m "Add main threat detection source code and tests"
```

### 3. Add ROS2 Components
```powershell
git add ros2_ws/src/
git commit -m "Add ROS2 sensor driver implementation"
```

### 4. Add Demo Outputs
```powershell
git add test_*_final.html
git commit -m "Add final demo visualization maps"
```

### 5. Push All Commits
```powershell
git push origin main
```

---

## 🔍 Verification Commands

### Check what git sees:
```powershell
git status
```

### See short status:
```powershell
git status --short
```

### Count files to add:
```powershell
git status --porcelain | Measure-Object -Line
```

### Check if cache is ignored:
```powershell
git status --porcelain | Select-String "__pycache__"
# Should return nothing (0 results)
```

### See all ignored files:
```powershell
git status --ignored
```

---

## 📝 Useful Git Commands

### See what will be added (dry run):
```powershell
git add --dry-run .
```

### Add interactively (review each file):
```powershell
git add -i
```

### Remove file from staging:
```powershell
git reset HEAD <file>
```

### Check git ignore status of specific file:
```powershell
git check-ignore -v <filename>
```

### See diff before committing:
```powershell
git diff
```

---

## 🎯 Recommended First Commit

```powershell
# Add everything
git add .

# Commit with detailed message
git commit -m "Initial commit: Multi-missile threat detection system

Features:
- IR and optical sensor processing
- Bayesian fusion engine for launch detection
- UKF tracking for multiple simultaneous missiles
- Real-time GIS visualization
- ROS2 integration for hardware deployment
- Comprehensive testing and validation
- Full documentation and deployment guides

Demo capabilities:
- Single missile demo (run_test.py)
- Multi-missile tracking (run_multi_missile_test.py)
- Production deployment on Raspberry Pi
- Web-based trajectory visualization"

# Push to remote
git push origin main
```

---

## ⚠️ Important Notes

### Files That SHOULD Be Tracked (44 items):
✅ All `.md` documentation files
✅ All `.py` source code
✅ `requirements.txt` and `requirements_windows.txt`
✅ ROS2 source files (.cpp, .hpp, .py)
✅ CMakeLists.txt and package.xml
✅ Final demo maps (*_final.html)
✅ `.gitignore` itself

### Files That Are IGNORED (automatically):
🚫 `__pycache__/` directories
🚫 `venv/` and `env/` directories
🚫 `.vscode/` and `.idea/` IDE settings
🚫 `ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`
🚫 Intermediate test maps (test_threat_map.html)
🚫 Log files (*.log)
🚫 Image data files (data/ir/*.jpg, data/optical/*.png)
🚫 OS files (Thumbs.db, .DS_Store)

---

## 🔧 If You Need to Undo

### Haven't committed yet:
```powershell
# Unstage all files
git reset

# Discard all changes
git checkout .
```

### Already committed but haven't pushed:
```powershell
# Undo last commit, keep changes
git reset --soft HEAD~1

# Undo last commit, discard changes
git reset --hard HEAD~1
```

---

## 📊 Repository Statistics

After committing, check your repo stats:

```powershell
# Count files tracked
git ls-files | Measure-Object -Line

# Repository size
git count-objects -vH

# Recent commits
git log --oneline -5
```

---

**Ready to commit? Just run:**
```powershell
git add .
git commit -m "Initial commit: Complete threat detection system"
git push origin main
```

✅ **Your repository is clean and ready!**
