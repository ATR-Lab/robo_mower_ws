#!/usr/bin/env python3
"""
EMERGENCY STOP for RoboClaw Motor Controller
Sends multiple stop commands to ensure motors stop immediately
"""

import serial
import time

def emergency_stop():
    try:
        # Connect directly to RoboClaw
        roboclaw = serial.Serial('/dev/ttyACM1', 38400, timeout=1)
        time.sleep(0.1)
        
        print("🚨 SENDING EMERGENCY STOP COMMANDS...")
        
        # Send multiple stop commands for redundancy
        for i in range(10):
            # Command 0: Emergency stop both motors
            roboclaw.write(b'\x80\x00\x80')
            time.sleep(0.01)
            
            # Command 6: M1 speed 0
            roboclaw.write(b'\x80\x06\x00\x00\x06')
            time.sleep(0.01)
            
            # Command 7: M2 speed 0  
            roboclaw.write(b'\x80\x07\x00\x00\x07')
            time.sleep(0.01)
            
            # Command 32: Mixed speed both 0
            roboclaw.write(b'\x80\x20\x00\x00\x00\x00\x20')
            time.sleep(0.01)
            
        roboclaw.close()
        print("✅ EMERGENCY STOP COMPLETED - Motors should be stopped")
        print("⚠️  If motors still run, DISCONNECT POWER immediately!")
        
    except Exception as e:
        print(f"❌ Failed to send emergency stop: {e}")
        print("🔥 CRITICAL: DISCONNECT POWER SOURCE NOW!")

def check_devices():
    """Check what devices are connected"""
    import os
    devices = []
    for device in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']:
        if os.path.exists(device):
            devices.append(device)
    return devices

if __name__ == "__main__":
    print("🔍 Checking connected devices...")
    devices = check_devices()
    print(f"Found devices: {devices}")
    
    emergency_stop()
