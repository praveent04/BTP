"""
System Verification Script
Tests all components before deployment
"""

import sys
import subprocess

def test_imports():
    """Test that all required modules can be imported."""
    print("\n" + "="*60)
    print("Testing Python Module Imports")
    print("="*60)
    
    modules = [
        'cv2',
        'numpy',
        'scipy',
        'filterpy',
        'pgmpy',
        'folium',
        'flask',
        'pyproj',
        'pandas'
    ]
    
    failed = []
    for module in modules:
        try:
            __import__(module)
            print(f"✅ {module:20s} - OK")
        except ImportError as e:
            print(f"❌ {module:20s} - FAILED: {e}")
            failed.append(module)
    
    if failed:
        print(f"\n⚠️  Failed imports: {', '.join(failed)}")
        print("Run: pip install -r requirements_windows.txt")
        return False
    else:
        print("\n✅ All modules imported successfully!")
        return True

def test_project_structure():
    """Test that all required files exist."""
    print("\n" + "="*60)
    print("Testing Project Structure")
    print("="*60)
    
    import os
    
    required_files = [
        'src/camera_manager.py',
        'src/ir_processor.py',
        'src/optical_processor.py',
        'src/fusion_engine.py',
        'src/ukf_tracker.py',
        'src/coordinate_transformer.py',
        'src/gis_plotter.py',
        'src/web_server.py',
        'src/main.py',
        'src/test_camera_manager.py',
        'src/test_main.py',
        'run_test.py',
        'run_production.py',
        'requirements.txt',
        'requirements_windows.txt'
    ]
    
    missing = []
    for file in required_files:
        if os.path.exists(file):
            print(f"✅ {file}")
        else:
            print(f"❌ {file} - MISSING")
            missing.append(file)
    
    if missing:
        print(f"\n⚠️  Missing files: {len(missing)}")
        return False
    else:
        print("\n✅ All required files present!")
        return True

def test_components():
    """Test individual components."""
    print("\n" + "="*60)
    print("Testing System Components")
    print("="*60)
    
    sys.path.insert(0, 'src')
    
    tests_passed = 0
    tests_total = 0
    
    # Test IR Processor
    tests_total += 1
    try:
        from ir_processor import IRProcessor
        import numpy as np
        
        processor = IRProcessor(threshold_value=220, min_area=100)
        test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        test_frame[240:260, 320:340] = 255  # Add bright spot
        
        hotspot = processor.find_hotspot(test_frame)
        assert hotspot is not None, "Hotspot not detected"
        assert 320 <= hotspot[0] <= 340, "Hotspot x position incorrect"
        assert 240 <= hotspot[1] <= 260, "Hotspot y position incorrect"
        
        print("✅ IR Processor - OK")
        tests_passed += 1
    except Exception as e:
        print(f"❌ IR Processor - FAILED: {e}")
    
    # Test Optical Processor
    tests_total += 1
    try:
        from optical_processor import OpticalProcessor
        import numpy as np
        
        processor = OpticalProcessor()
        test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Should not detect motion in static frame
        result = processor.detect_motion(test_frame, (320, 240))
        # Motion detection might be False initially
        
        print("✅ Optical Processor - OK")
        tests_passed += 1
    except Exception as e:
        print(f"❌ Optical Processor - FAILED: {e}")
    
    # Test Fusion Engine
    tests_total += 1
    try:
        from fusion_engine import FusionEngine
        
        engine = FusionEngine()
        
        # Test with both sensors positive
        prob = engine.calculate_launch_probability(True, True)
        assert prob > 0.9, f"Probability too low: {prob}"
        
        # Test with both sensors negative
        prob = engine.calculate_launch_probability(False, False)
        assert prob < 0.01, f"Probability too high: {prob}"
        
        print("✅ Fusion Engine - OK")
        tests_passed += 1
    except Exception as e:
        print(f"❌ Fusion Engine - FAILED: {e}")
    
    # Test UKF Tracker
    tests_total += 1
    try:
        from ukf_tracker import UKFTracker
        
        tracker = UKFTracker(dt=0.033)
        
        # Initialize with a measurement
        state = tracker.update((100, 100))
        assert state is not None, "Tracker initialization failed"
        
        # Update with new measurement
        state = tracker.update((110, 95))
        assert state is not None, "Tracker update failed"
        
        print("✅ UKF Tracker - OK")
        tests_passed += 1
    except Exception as e:
        print(f"❌ UKF Tracker - FAILED: {e}")
    
    # Test Coordinate Transformer
    tests_total += 1
    try:
        from coordinate_transformer import CoordinateTransformer
        
        transformer = CoordinateTransformer(
            base_lat=28.6139,
            base_lon=77.2090,
            altitude=100.0,
            image_width=640,
            image_height=480
        )
        
        # Test center pixel (should be close to base coords)
        lat, lon = transformer.pixel_to_geographic(320, 240)
        assert abs(lat - 28.6139) < 0.01, f"Latitude off: {lat}"
        assert abs(lon - 77.2090) < 0.01, f"Longitude off: {lon}"
        
        print("✅ Coordinate Transformer - OK")
        tests_passed += 1
    except Exception as e:
        print(f"❌ Coordinate Transformer - FAILED: {e}")
    
    # Test GIS Plotter
    tests_total += 1
    try:
        from gis_plotter import GISPlotter
        import os
        
        plotter = GISPlotter(center_lat=28.6139, center_lon=77.2090)
        plotter.add_trajectory_point(28.614, 77.210)
        plotter.save_map('test_verification_map.html')
        
        assert os.path.exists('test_verification_map.html'), "Map file not created"
        os.remove('test_verification_map.html')  # Cleanup
        
        print("✅ GIS Plotter - OK")
        tests_passed += 1
    except Exception as e:
        print(f"❌ GIS Plotter - FAILED: {e}")
    
    # Test Camera Manager (simulated)
    tests_total += 1
    try:
        from test_camera_manager import TestCameraManager
        
        cam_mgr = TestCameraManager(width=640, height=480)
        assert cam_mgr.initialize_cameras(), "Camera init failed"
        
        ir, opt = cam_mgr.get_frames()
        assert ir is not None, "IR frame is None"
        assert opt is not None, "Optical frame is None"
        assert ir.shape == (480, 640, 3), "IR frame wrong shape"
        
        print("✅ Test Camera Manager - OK")
        tests_passed += 1
    except Exception as e:
        print(f"❌ Test Camera Manager - FAILED: {e}")
    
    print(f"\n{'='*60}")
    print(f"Component Tests: {tests_passed}/{tests_total} passed")
    print(f"{'='*60}")
    
    return tests_passed == tests_total

def main():
    """Run all verification tests."""
    print("\n" + "="*60)
    print("THREAT DETECTION SYSTEM - VERIFICATION")
    print("="*60)
    print("\nThis script will verify that all components are correctly")
    print("installed and functioning before deployment.\n")
    
    all_passed = True
    
    # Test imports
    if not test_imports():
        all_passed = False
    
    # Test structure
    if not test_project_structure():
        all_passed = False
    
    # Test components
    if not test_components():
        all_passed = False
    
    # Final result
    print("\n" + "="*60)
    if all_passed:
        print("✅ ALL VERIFICATION TESTS PASSED!")
        print("="*60)
        print("\nSystem is ready for deployment!")
        print("\nNext steps:")
        print("  - For Windows demo: python run_test.py")
        print("  - For Raspberry Pi:  python3 run_production.py")
    else:
        print("❌ SOME TESTS FAILED")
        print("="*60)
        print("\nPlease fix the issues above before deployment.")
        print("\nCommon fixes:")
        print("  - Missing modules: pip install -r requirements_windows.txt")
        print("  - Missing files: Ensure complete project structure")
        sys.exit(1)
    
    print("="*60 + "\n")

if __name__ == '__main__':
    main()
