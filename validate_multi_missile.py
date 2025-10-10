"""
Quick validation test for multi-missile system
"""

import sys
sys.path.insert(0, 'src')

print("Testing imports...")

# Test camera manager
from test_camera_manager_multi import TestCameraManagerMulti, Missile
print("✅ TestCameraManagerMulti imported")

# Test other components
from ir_processor import IRProcessor
from optical_processor import OpticalProcessor
from fusion_engine import FusionEngine
from ukf_tracker import UKFTracker
from coordinate_transformer import CoordinateTransformer
from gis_plotter import GISPlotter
from web_server import WebServer
print("✅ All components imported")

# Test instantiation
print("\nTesting instantiation...")

camera_mgr = TestCameraManagerMulti(
    width=640,
    height=480,
    fps=30,
    num_missiles=2,
    tracking_duration=10.0,
    launch_interval=50
)
print("✅ Camera manager created")

ir_proc = IRProcessor(threshold_value=220, min_area=100)
print("✅ IR Processor created")

optical_proc = OpticalProcessor(bg_subtractor_type='MOG2')
print("✅ Optical Processor created")

fusion = FusionEngine()
print("✅ Fusion Engine created")

tracker = UKFTracker(dt=1/30)
print("✅ UKF Tracker created")

coord_trans = CoordinateTransformer(
    base_lat=28.6139,
    base_lon=77.2090,
    altitude=100.0,
    camera_fov_horizontal=62.2,
    image_width=640,
    image_height=480
)
print("✅ Coordinate Transformer created")

gis = GISPlotter(center_lat=28.6139, center_lon=77.2090, zoom_start=15)
print("✅ GIS Plotter created")

# Test basic functionality
print("\nTesting basic functionality...")

camera_mgr.initialize_cameras()
print("✅ Cameras initialized")

ir_frame, optical_frame = camera_mgr.get_frames()
print(f"✅ Got frames: IR {ir_frame.shape}, Optical {optical_frame.shape}")

hotspot = ir_proc.find_hotspot(ir_frame)
print(f"✅ IR processing works: hotspot={hotspot}")

stats = camera_mgr.get_statistics()
print(f"✅ Statistics: {stats}")

camera_mgr.release_cameras()
print("✅ Cameras released")

print("\n" + "="*60)
print("✅ ALL TESTS PASSED!")
print("="*60)
print("\nThe multi-missile system is ready to run!")
print("Execute: python run_multi_missile_test.py")
