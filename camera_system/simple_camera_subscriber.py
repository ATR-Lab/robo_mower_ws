#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class SimpleCameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image_raw/compressed',
            self.image_callback,
            10
        )
        
        self.frame_count = 0
        self.get_logger().info('Camera subscriber started - subscribing to /camera/image_raw/compressed')
    
    def image_callback(self, msg):
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                self.frame_count += 1
                
                # Log every 30 frames
                if self.frame_count % 30 == 0:
                    self.get_logger().info(
                        f'Received frame {self.frame_count}, '
                        f'size: {cv_image.shape}, '
                        f'data size: {len(msg.data)} bytes'
                    )
                    
                # Optional: Save test image every 100 frames
                if self.frame_count % 100 == 0:
                    filename = f'/tmp/test_camera_frame_{self.frame_count}.jpg'
                    cv2.imwrite(filename, cv_image)
                    self.get_logger().info(f'Saved test frame: {filename}')
                    
            else:
                self.get_logger().warn('Failed to decode compressed image')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_subscriber = SimpleCameraSubscriber()
        print('Camera subscriber running - Press Ctrl+C to stop...')
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'camera_subscriber' in locals():
            camera_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
