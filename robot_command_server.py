#!/usr/bin/env python3
"""
Simple Robot Command Server
Listens for commands via netcat/telnet from VS Code
"""

import socket
import subprocess
import os
import threading
import json

class RobotCommandServer:
    def __init__(self, host='0.0.0.0', port=8888):
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Change to ROS2 workspace
        os.chdir('/home/alwinsdon/ros2_ws')
        
    def start(self):
        """Start the command server"""
        self.socket.bind((self.host, self.port))
        self.socket.listen(5)
        print(f"🚀 Robot Command Server listening on {self.host}:{self.port}")
        print("📡 Connect from VS Code with:")
        print(f"   telnet 192.168.2.144 {self.port}")
        print("🎮 Available commands:")
        print("   forward, backward, left, right, stop")
        print("   status, emergency, custom <command>")
        
        while True:
            try:
                client_socket, address = self.socket.accept()
                print(f"📞 Connection from {address}")
                
                # Handle client in separate thread
                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket, address)
                )
                client_thread.daemon = True
                client_thread.start()
                
            except KeyboardInterrupt:
                print("\n🛑 Shutting down server...")
                break
            except Exception as e:
                print(f"❌ Server error: {e}")
        
        self.socket.close()
    
    def handle_client(self, client_socket, address):
        """Handle individual client connections"""
        try:
            # Send welcome message
            welcome = "🤖 Robot Command Server Ready\n"
            welcome += "Commands: forward, backward, left, right, stop, status, emergency\n"
            welcome += "Type 'help' for more commands\n> "
            client_socket.send(welcome.encode())
            
            while True:
                # Receive command
                data = client_socket.recv(1024).decode().strip()
                if not data:
                    break
                
                print(f"📨 Command from {address}: {data}")
                
                # Process command
                response = self.process_command(data)
                
                # Send response
                client_socket.send(f"{response}\n> ".encode())
                
        except Exception as e:
            print(f"❌ Client error: {e}")
        finally:
            client_socket.close()
            print(f"📴 Disconnected: {address}")
    
    def process_command(self, command):
        """Process robot commands"""
        command = command.lower().strip()
        
        try:
            if command == "forward" or command == "w":
                return self.execute_robot_command(1.0, 0.0, "Moving forward")
            
            elif command == "backward" or command == "s":
                return self.execute_robot_command(-1.0, 0.0, "Moving backward")
            
            elif command == "left" or command == "a":
                return self.execute_robot_command(0.0, 1.5, "Turning left")
            
            elif command == "right" or command == "d":
                return self.execute_robot_command(0.0, -1.5, "Turning right")
            
            elif command == "stop" or command == "space":
                return self.execute_robot_command(0.0, 0.0, "Stopping")
            
            elif command == "emergency":
                result = subprocess.run("python3 emergency_stop.py", 
                                      shell=True, capture_output=True, text=True, timeout=10)
                return f"🚨 Emergency stop executed\nOutput: {result.stdout}"
            
            elif command == "status":
                return self.get_robot_status()
            
            elif command == "help":
                return self.get_help()
            
            elif command.startswith("custom "):
                custom_cmd = command[7:]  # Remove "custom " prefix
                if self.is_safe_command(custom_cmd):
                    result = subprocess.run(custom_cmd, shell=True, 
                                          capture_output=True, text=True, timeout=30)
                    return f"Executed: {custom_cmd}\nOutput: {result.stdout}"
                else:
                    return "❌ Command not allowed for security reasons"
            
            else:
                return f"❌ Unknown command: {command}\nType 'help' for available commands"
                
        except Exception as e:
            return f"❌ Error executing command: {e}"
    
    def execute_robot_command(self, linear, angular, description):
        """Execute robot movement command"""
        try:
            cmd = f"python3 remote_control.py {linear} {angular}"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
            return f"✅ {description}\nLinear: {linear}, Angular: {angular}\nOutput: {result.stdout}"
        except Exception as e:
            return f"❌ Failed to execute movement: {e}"
    
    def get_robot_status(self):
        """Get robot system status"""
        try:
            status = "🤖 Robot Status:\n"
            status += f"IP: 192.168.2.144\n"
            status += f"Hostname: alwinsdon-desktop\n"
            
            # Check motor driver
            motor_check = subprocess.run("ps aux | grep ros2_roboclaw_driver", 
                                       shell=True, capture_output=True, text=True)
            motor_running = "ros2_roboclaw_driver_node" in motor_check.stdout
            status += f"Motor Driver: {'🟢 Running' if motor_running else '🔴 Not Running'}\n"
            
            return status
        except Exception as e:
            return f"❌ Error getting status: {e}"
    
    def get_help(self):
        """Get help message"""
        help_text = """🤖 Robot Control Commands:
        
Movement:
  forward (w)  - Move forward
  backward (s) - Move backward  
  left (a)     - Turn left
  right (d)    - Turn right
  stop (space) - Stop movement
  
System:
  status       - Get robot status
  emergency    - Emergency stop
  help         - Show this help
  
Custom:
  custom <cmd> - Execute custom command (limited)
  
Examples:
  forward
  stop
  custom python3 emergency_stop.py
"""
        return help_text
    
    def is_safe_command(self, command):
        """Check if command is safe to execute"""
        safe_commands = [
            "python3 remote_control.py",
            "python3 emergency_stop.py", 
            "python3 force_stop.py",
            "ros2 topic list",
            "ros2 node list",
            "ps aux"
        ]
        return any(command.startswith(safe) for safe in safe_commands)

if __name__ == "__main__":
    server = RobotCommandServer()
    try:
        server.start()
    except KeyboardInterrupt:
        print("\n🛑 Server stopped")
