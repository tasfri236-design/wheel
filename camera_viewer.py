#!/usr/bin/env python3
"""
camera_viewer.py
----------------
Simple camera feed viewer.
Starts the camera stream and displays the live feed via HTTP.

Usage:
    python3 camera_viewer.py

Then open a browser to:
    http://<pi-ip>:8000/
"""

from camera_stream import start_camera_stream, stop_camera_stream
from time import sleep

if __name__ == "__main__":
    print("🎥 Starting camera feed viewer...")
    
    try:
        # Start the camera stream
        camera = start_camera_stream()
        
        # Keep running until Ctrl+C
        print("\n✓ Camera stream started. Press Ctrl+C to stop.\n")
        while True:
            sleep(1)
    except KeyboardInterrupt:
        print("\n\n⏹️  Stopping camera stream...")
        stop_camera_stream()
        print("✓ Camera stream stopped.")
