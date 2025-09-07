#!/usr/bin/env python3
"""
Robot Control Web API
Allows remote command execution via HTTP requests from VS Code
"""

from flask import Flask, request, jsonify
import subprocess
import os
import threading
import time

app = Flask(__name__)

# Change to the ROS2 workspace directory
os.chdir('/home/alwinsdon/ros2_ws')

@app.route('/robot/move', methods=['POST'])
def move_robot():
    """Send movement commands to the robot"""
    try:
        data = request.get_json()
        linear = data.get('linear', 0.0)
        angular = data.get('angular', 0.0)
        
        # Execute robot movement command
        cmd = f"python3 remote_control.py {linear} {angular}"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        
        return jsonify({
            'status': 'success',
            'command': cmd,
            'output': result.stdout,
            'linear': linear,
            'angular': angular
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500

@app.route('/robot/forward', methods=['POST'])
def move_forward():
    """Move robot forward"""
    return move_robot_with_params(1.0, 0.0)

@app.route('/robot/backward', methods=['POST'])
def move_backward():
    """Move robot backward"""
    return move_robot_with_params(-1.0, 0.0)

@app.route('/robot/left', methods=['POST'])
def turn_left():
    """Turn robot left"""
    return move_robot_with_params(0.0, 1.5)

@app.route('/robot/right', methods=['POST'])
def turn_right():
    """Turn robot right"""
    return move_robot_with_params(0.0, -1.5)

@app.route('/robot/stop', methods=['POST'])
def stop_robot():
    """Stop robot movement"""
    return move_robot_with_params(0.0, 0.0)

def move_robot_with_params(linear, angular):
    """Helper function to move robot with specific parameters"""
    try:
        cmd = f"python3 remote_control.py {linear} {angular}"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        
        return jsonify({
            'status': 'success',
            'command': cmd,
            'output': result.stdout,
            'linear': linear,
            'angular': angular
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500

@app.route('/robot/status', methods=['GET'])
def robot_status():
    """Get robot system status"""
    try:
        # Check if motor driver is running
        motor_check = subprocess.run("ps aux | grep ros2_roboclaw_driver", 
                                   shell=True, capture_output=True, text=True)
        motor_running = "ros2_roboclaw_driver_node" in motor_check.stdout
        
        return jsonify({
            'status': 'online',
            'ip_address': '192.168.2.144',
            'hostname': 'alwinsdon-desktop',
            'motor_driver_running': motor_running,
            'timestamp': time.time()
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500

@app.route('/system/execute', methods=['POST'])
def execute_command():
    """Execute custom system commands"""
    try:
        data = request.get_json()
        command = data.get('command', '')
        
        if not command:
            return jsonify({'status': 'error', 'message': 'No command provided'}), 400
        
        # Security: Only allow specific safe commands
        allowed_commands = [
            'python3 remote_control.py',
            'python3 emergency_stop.py',
            'python3 force_stop.py',
            'ros2 topic list',
            'ros2 node list',
            'ps aux | grep ros2'
        ]
        
        if not any(command.startswith(allowed) for allowed in allowed_commands):
            return jsonify({'status': 'error', 'message': 'Command not allowed'}), 403
        
        result = subprocess.run(command, shell=True, capture_output=True, text=True, timeout=30)
        
        return jsonify({
            'status': 'success',
            'command': command,
            'output': result.stdout,
            'error': result.stderr,
            'return_code': result.returncode
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'message': str(e)
        }), 500

if __name__ == '__main__':
    print("🚀 Robot Control API Starting...")
    print("📡 Access from VS Code at: http://192.168.2.144:5000")
    print("🎮 Available endpoints:")
    print("  POST /robot/forward  - Move forward")
    print("  POST /robot/backward - Move backward") 
    print("  POST /robot/left     - Turn left")
    print("  POST /robot/right    - Turn right")
    print("  POST /robot/stop     - Stop movement")
    print("  GET  /robot/status   - Get status")
    print("  POST /robot/move     - Custom movement (JSON: {linear, angular})")
    
    # Run the Flask app
    app.run(host='0.0.0.0', port=5000, debug=True)
