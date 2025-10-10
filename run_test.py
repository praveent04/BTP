"""
Test/Demo Runner Script for Windows
Runs the simulated threat detection system for presentations
"""

import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from test_main import main

if __name__ == '__main__':
    print("\n" + "="*70)
    print(" "*15 + "THREAT DETECTION SYSTEM - DEMO MODE")
    print("="*70)
    print("\n🎬 This is a SIMULATION for demonstration purposes.")
    print("   It will show a synthetic missile launch and track it.\n")
    print("📊 All processing algorithms are identical to production.")
    print("   Only the camera input is simulated.\n")
    print("🌐 A web server will start on http://localhost:5000")
    print("   to display the real-time trajectory map.\n")
    print("="*70)
    
    input("\nPress Enter to start the demo...")
    
    main()
