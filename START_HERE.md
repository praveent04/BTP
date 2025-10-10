# 📚 Documentation Navigation Guide

**Quick Links:**
- 🚀 **[Start Here: PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)** - Complete project overview
- ⚡ **[Quick Start: QUICKSTART.md](QUICKSTART.md)** - Fast setup guide  
- 📖 **[Full Guide: DEPLOYMENT_README.md](DEPLOYMENT_README.md)** - Complete deployment
- 🏗️ **[Technical: ARCHITECTURE.md](ARCHITECTURE.md)** - System architecture

---

## 🎯 Choose Your Path

### Path 1: "I want to see the demo NOW" (5 minutes)
```
1. Open PowerShell in project directory
2. python -m venv venv
3. .\venv\Scripts\activate
4. pip install -r requirements_windows.txt
5. python run_test.py
6. Open browser: http://localhost:5000
```
More details: [QUICKSTART.md](QUICKSTART.md)

### Path 2: "I want to deploy on Raspberry Pi" (2 hours)
```
1. Read: DEPLOYMENT_README.md → Raspberry Pi section
2. Setup hardware (Pi + cameras)
3. Follow step-by-step installation
4. Run: python3 run_production.py
```
Full guide: [DEPLOYMENT_README.md](DEPLOYMENT_README.md)

### Path 3: "I want to understand how it works" (30 minutes)
```
1. Read: PROJECT_SUMMARY.md (overview)
2. Read: ARCHITECTURE.md (technical details)
3. Review: src/ code files
```
Technical docs: [ARCHITECTURE.md](ARCHITECTURE.md)

---

## 📁 What's Included

### Production Code (Raspberry Pi)
- ✅ Complete multi-sensor fusion system
- ✅ IR + Optical camera integration
- ✅ Bayesian belief network
- ✅ UKF trajectory tracker
- ✅ GIS visualization
- ✅ Web interface

### Demo Code (Windows)
- ✅ Simulated missile launch
- ✅ Same algorithms as production
- ✅ No hardware required
- ✅ Perfect for presentations

### Documentation
- ✅ Deployment guides
- ✅ Architecture details
- ✅ Troubleshooting
- ✅ Configuration tuning

---

## 🆘 Need Help?

**Problem**: Demo won't start  
**Solution**: [QUICKSTART.md](QUICKSTART.md) → Troubleshooting

**Problem**: Raspberry Pi cameras not working  
**Solution**: [DEPLOYMENT_README.md](DEPLOYMENT_README.md) → Troubleshooting

**Problem**: Want to tune detection  
**Solution**: [DEPLOYMENT_README.md](DEPLOYMENT_README.md) → Configuration

**Problem**: Understanding algorithms  
**Solution**: [ARCHITECTURE.md](ARCHITECTURE.md) → Algorithm Details

---

## ✅ Verification

Before running, verify installation:
```bash
python verify_system.py
```

---

**Project Status**: ✅ Complete and Ready for Deployment

For complete navigation: See detailed index in this directory
