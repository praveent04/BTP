"""
Production Runner Script for Raspberry Pi
Runs the actual threat detection system with real cameras
"""

import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from main import main

if __name__ == '__main__':
    print("="*60)
    print("STARTING PRODUCTION THREAT DETECTION SYSTEM")
    print("="*60)
    print("\nThis will use REAL camera hardware.")
    print("Ensure IR and optical cameras are connected.\n")
    
    input("Press Enter to continue or Ctrl+C to abort...")
    
    main()
