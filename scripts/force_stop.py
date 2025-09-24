#!/usr/bin/env python3

import serial
import time
import struct

def force_stop_roboclaw():
    try:
        # Connect to RoboClaw
        ser = serial.Serial('/dev/ttyACM1', 38400, timeout=0.5)
        time.sleep(0.1)
        
        print("Sending emergency stop commands...")
        
        # Send multiple different stop commands
        for i in range(20):
            # Command 0: Emergency stop
            ser.write(b'\x80\x00\x80')
            time.sleep(0.01)
            
            # Command 6: M1 speed 0
            ser.write(b'\x80\x06\x00\x00\x06')
            time.sleep(0.01)
            
            # Command 7: M2 speed 0  
            ser.write(b'\x80\x07\x00\x00\x07')
            time.sleep(0.01)
            
            # Command 32: Mixed speed both 0
            ser.write(b'\x80\x20\x00\x00\x00\x00\x20')
            time.sleep(0.01)
            
        ser.close()
        print("✅ Multiple emergency stop commands sent!")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    force_stop_roboclaw()
