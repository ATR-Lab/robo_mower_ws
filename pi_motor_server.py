#!/usr/bin/env python3
"""
Pi Motor Server - Network Motor Control Server
Receives commands from remote laptops and controls the robot
Compatible with standard robot control protocols
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import threading
import json
import time
import sys

class PiMotorServer(Node):
    def __init__(self):
        super().__init__('pi_motor_server')
        
        # ROS2 publisher for motor commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Network configuration
        self.host = '0.0.0.0'  # Listen on all interfaces
        self.socket_port = 8888  # Socket server port
        
        self.get_logger().info('🤖 Pi Motor Server starting...')
        self.get_logger().info(f'📡 Socket server: {self.host}:{self.socket_port}')
        
        # Start socket server in thread
        self.socket_thread = threading.Thread(target=self.start_socket_server)
        self.socket_thread.daemon = True
        self.socket_thread.start()
        
        self.get_logger().info('✅ Pi Motor Server ready for remote control!')

    def send_motor_command(self, linear_x=0.0, angular_z=0.0):
        """Send motor command via ROS2"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'📤 Motor command: Linear={linear_x:.2f}, Angular={angular_z:.2f}')

    def start_socket_server(self):
        """Socket server for telnet/netcat connections"""
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((self.host, self.socket_port))
            server_socket.listen(5)
            
            self.get_logger().info(f'🔌 Socket server listening on port {self.socket_port}')
            
            while True:
                client_socket, addr = server_socket.accept()
                self.get_logger().info(f'📱 Client connected: {addr}')
                
                # Handle client in separate thread
                client_thread = threading.Thread(
                    target=self.handle_socket_client, 
                    args=(client_socket, addr)
                )
                client_thread.daemon = True
                client_thread.start()
                
        except Exception as e:
            self.get_logger().error(f'❌ Socket server error: {e}')

    def handle_socket_client(self, client_socket, addr):
        """Handle individual socket client"""
        try:
            # Send welcome message
            welcome = (
                "🤖 Pi Motor Server Connected!\n"
                "Available commands:\n"
                "  forward, backward, left, right, stop\n"
                "  quit, status\n"
                "  move <linear> <angular>\n"
                ">>> "
            )
            client_socket.send(welcome.encode())
            
            while True:
                data = client_socket.recv(1024).decode().strip()
                if not data:
                    break
                    
                response = self.process_command(data)
                client_socket.send(f"{response}\n>>> ".encode())
                
        except Exception as e:
            self.get_logger().warn(f'Client {addr} disconnected: {e}')
        finally:
            client_socket.close()

    def process_command(self, command):
        """Process text commands"""
        command = command.lower().strip()
        
        if command == 'forward':
            self.send_motor_command(0.5, 0.0)
            return "✅ Moving forward"
        elif command == 'backward':
            self.send_motor_command(-0.5, 0.0)
            return "✅ Moving backward"
        elif command == 'left':
            self.send_motor_command(0.0, 1.5)
            return "✅ Turning left"
        elif command == 'right':
            self.send_motor_command(0.0, -1.5)
            return "✅ Turning right"
        elif command == 'stop':
            self.send_motor_command(0.0, 0.0)
            return "⏹️ Stopped"
        elif command == 'status':
            return "🤖 Pi Motor Server - Robot ready"
        elif command.startswith('move '):
            try:
                parts = command.split()
                linear = float(parts[1])
                angular = float(parts[2])
                self.send_motor_command(linear, angular)
                return f"✅ Moving: linear={linear}, angular={angular}"
            except:
                return "❌ Invalid move command. Use: move <linear> <angular>"
        elif command == 'quit':
            return "👋 Goodbye!"
        else:
            return "❌ Unknown command"

def main(args=None):
    print("🚀 Starting Pi Motor Server...")
    
    # Configure ROS2 environment
    import os
    os.environ['ROS_DOMAIN_ID'] = '0'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['ROS_LOCALHOST_ONLY'] = '0'
    
    rclpy.init(args=args)
    
    try:
        server = PiMotorServer()
        
        print("📡 Pi Motor Server is running!")
        print("🔌 Socket connection: telnet 192.168.2.144 8888")
        print("📱 From your laptop, connect with:")
        print("   telnet 192.168.2.144 8888")
        print("   Then type: forward, backward, left, right, stop")
        print("Press Ctrl+C to stop")
        
        rclpy.spin(server)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutting down Pi Motor Server...")
    finally:
        if 'server' in locals():
            server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
