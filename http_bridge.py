#!/usr/bin/env python3
"""
HTTP Bridge for Robot Control
Runs on the robot to receive HTTP commands and convert to ROS2
"""

from flask import Flask, request, jsonify
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

app = Flask(__name__)

class HTTPBridge(Node):
    def __init__(self):
        super().__init__('http_bridge')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('🌐 HTTP Bridge Ready!')
        
    def send_velocity(self, linear_x, angular_z):
        """Send velocity command to motors"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'📤 HTTP->ROS: Linear={linear_x}, Angular={angular_z}')

# Global bridge instance
bridge = None

@app.route('/cmd_vel', methods=['POST'])
def cmd_vel():
    """Receive velocity commands via HTTP"""
    try:
        data = request.get_json()
        linear_x = data.get('linear_x', 0.0)
        angular_z = data.get('angular_z', 0.0)
        
        if bridge:
            bridge.send_velocity(linear_x, angular_z)
            return jsonify({"status": "success", "linear_x": linear_x, "angular_z": angular_z})
        else:
            return jsonify({"status": "error", "message": "Bridge not initialized"}), 500
            
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400

@app.route('/status', methods=['GET'])
def status():
    """Health check endpoint"""
    return jsonify({
        "status": "running",
        "robot": "lawnmower",
        "bridge": "active" if bridge else "inactive"
    })

@app.route('/', methods=['GET'])
def home():
    """Simple web interface"""
    return '''
    <html>
    <head><title>Lawnmower Robot Control</title></head>
    <body>
        <h1>🤖 Lawnmower Robot Control</h1>
        <p>Robot Status: <span style="color:green">Online</span></p>
        <p>Use the standalone control script or ROS2 commands to control the robot.</p>
        <hr>
        <h3>Quick Test Commands:</h3>
        <button onclick="sendCmd(0.3, 0)">Forward</button>
        <button onclick="sendCmd(-0.3, 0)">Backward</button>
        <button onclick="sendCmd(0, 0.5)">Left</button>
        <button onclick="sendCmd(0, -0.5)">Right</button>
        <button onclick="sendCmd(0, 0)">Stop</button>
        
        <script>
        function sendCmd(linear, angular) {
            fetch('/cmd_vel', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({linear_x: linear, angular_z: angular})
            });
        }
        </script>
    </body>
    </html>
    '''

def ros_thread():
    """Run ROS2 in separate thread"""
    global bridge
    rclpy.init()
    bridge = HTTPBridge()
    rclpy.spin(bridge)

if __name__ == '__main__':
    print("🚀 Starting HTTP Bridge for Robot Control")
    
    # Start ROS2 in background thread
    ros_thread_handle = threading.Thread(target=ros_thread, daemon=True)
    ros_thread_handle.start()
    
    # Give ROS2 time to initialize
    time.sleep(2)
    
    print("🌐 Starting HTTP server on port 8080")
    print("📱 Access web interface at: http://192.168.2.144:8080")
    
    # Start Flask server
    app.run(host='0.0.0.0', port=8080, debug=False)
