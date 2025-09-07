#!/usr/bin/env python3
"""
Standalone Remote Control for Lawnmower
Works without ROS2 installation on control PC
Sends HTTP commands to robot
"""

import requests
import json
import time
import sys

class StandaloneRemoteController:
    def __init__(self, robot_ip="192.168.2.144", robot_port=8080):
        self.robot_url = f"http://{robot_ip}:{robot_port}"
        print(f"🌐 Connecting to robot at {self.robot_url}")
    
    def send_command(self, linear_x=0.0, angular_z=0.0):
        """Send velocity command via HTTP"""
        try:
            data = {
                "linear_x": linear_x,
                "angular_z": angular_z
            }
            response = requests.post(f"{self.robot_url}/cmd_vel", 
                                   json=data, timeout=2)
            if response.status_code == 200:
                print(f"✅ Command sent: Linear={linear_x}, Angular={angular_z}")
                return True
            else:
                print(f"❌ Robot responded with error: {response.status_code}")
                return False
        except requests.exceptions.RequestException as e:
            print(f"❌ Connection failed: {e}")
            print("💡 Make sure the robot is running and connected to WiFi")
            return False
    
    def test_connection(self):
        """Test if robot is reachable"""
        try:
            response = requests.get(f"{self.robot_url}/status", timeout=2)
            return response.status_code == 200
        except:
            return False

def keyboard_control_standalone():
    """Standalone keyboard control (no ROS2 needed)"""
    print("\n🎮 Standalone Lawnmower Control")
    print("📡 This works without ROS2 on your PC!")
    
    controller = StandaloneRemoteController()
    
    # Test connection
    print("🔍 Testing connection to robot...")
    if not controller.test_connection():
        print("⚠️  Cannot reach robot. Make sure:")
        print("   1. Robot is powered on")
        print("   2. Robot is connected to WiFi")
        print("   3. HTTP server is running on robot")
        print("   4. Both PCs are on same network")
        print("\n🚀 Continuing anyway - commands will be attempted...")
    else:
        print("✅ Robot connection successful!")
    
    print("\nControls:")
    print("  W - Forward")
    print("  S - Backward") 
    print("  A - Turn Left")
    print("  D - Turn Right")
    print("  Q - Quit")
    print("  SPACE - Stop")
    print("\nPress keys and ENTER:")
    
    try:
        while True:
            key = input(">>> ").strip().lower()
            
            if key == 'w':
                print("🔼 Moving Forward...")
                controller.send_command(0.3, 0.0)
            elif key == 's':
                print("🔽 Moving Backward...")
                controller.send_command(-0.3, 0.0)
            elif key == 'a':
                print("◀️ Turning Left...")
                controller.send_command(0.0, 0.5)
            elif key == 'd':
                print("▶️ Turning Right...")
                controller.send_command(0.0, -0.5)
            elif key == ' ' or key == 'stop':
                print("⏹️ Stopping...")
                controller.send_command(0.0, 0.0)
            elif key == 'q' or key == 'quit':
                print("🛑 Stopping and quitting...")
                controller.send_command(0.0, 0.0)
                print("👋 Goodbye!")
                break
            elif key == 'test':
                print("🔍 Testing connection...")
                if controller.test_connection():
                    print("✅ Robot is reachable!")
                else:
                    print("❌ Cannot reach robot")
            else:
                print("❌ Unknown command. Use W/A/S/D/SPACE/Q")
                
    except KeyboardInterrupt:
        print("\n🛑 Emergency stop...")
        controller.send_command(0.0, 0.0)
        print("\n👋 Goodbye!")

def main():
    if len(sys.argv) > 1 and sys.argv[1] == "standalone":
        keyboard_control_standalone()
    else:
        print("🖥️ Standalone Lawnmower Remote Control")
        print("")
        print("Usage:")
        print("  python3 standalone_control.py standalone")
        print("")
        print("💡 This script works without ROS2 on your control PC!")
        print("💡 Just needs Python with 'requests' library")
        print("")
        print("📦 Install requirements:")
        print("  pip install requests")

if __name__ == '__main__':
    main()
