#!/usr/bin/env python3
"""
depth_node.py
=============

First proof‑of‑concept node:
1. Subscribes to the CARLA front‑RGB camera image.
2. GUESS the pixel row where the tyre touches the ground
   (we'll replace guess_ground_contact() with YOLO later).
3. Convert that row to metric depth using the ground‑plane formula:
     Z = fy * h_c / (y_b - y_h)
4. Publish the result as DepthStamped.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from monocular_depth_pkg.msg import DepthStamped  
from monocular_depth_interfaces.msg import DepthStamped   # our custom message
from cv_bridge import CvBridge                      # image ↔ OpenCV converter
import cv2                                          # OpenCV for future bbox drawing

class MonoDepth(Node):
    def __init__(self):
        super().__init__('mono_depth')

        # Declare parameters so a launch file can override them
        self.declare_parameter('camera_height', 1.55)  # metres
        self.declare_parameter('fy', 630.0)            # focal length in pixels
        self.declare_parameter('yh', 180.0)            # horizon pixel row

        # Read params into plain variables
        self.h_c = self.get_parameter('camera_height').value
        self.fy  = self.get_parameter('fy').value
        self.yh  = self.get_parameter('yh').value

        self.bridge = CvBridge()

        # Subscribe to CARLA's RGB image topic (update string after verifying)
        self.create_subscription(
            Image,
            '/carla/ego_vehicle/rgb_front/image',   # <-- adjust later
            self.image_cb,
            10)

        # Publisher for metric depth
        self.pub = self.create_publisher(DepthStamped, 'depth/metric', 10)

        self.get_logger().info('mono_depth node initialised')

    # --------------------------------------------------------------------- #
    def image_cb(self, msg: Image):
        """Convert ROS image → OpenCV BGR → estimate depth → publish."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        y_b = self.guess_ground_contact(frame)    # bottom pixel of the box
        if y_b is None:
            return                                # nothing to compute yet

        depth = self.fy * self.h_c / (y_b - self.yh)

        # Fill + publish the DepthStamped message
        out = DepthStamped()
        out.header = msg.header    # copy stamp + frame_id
        out.depth = float(depth)
        self.pub.publish(out)

        # Print to console for quick feedback
        self.get_logger().info(f"Depth = {depth:5.2f} m (y_b={y_b})")

    # --------------------------------------------------------------------- #
    def guess_ground_contact(self, img):
        """
        Placeholder: returns the bottom row of the image = means 'object is
        touching exactly the bottom edge'. Obviously not true, but good enough
        to see numbers changing when the ego car moves.
        """
        return img.shape[0] - 1     # img.shape: (rows, cols, channels)


def main():
    rclpy.init()
    node = MonoDepth()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
