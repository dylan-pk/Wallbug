#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class NDVINode(Node):
    def __init__(self):
        super().__init__('ndvi_node')
        self.bridge = CvBridge()

        # Topics: adjust later if needed
        self.sub_rgb = self.create_subscription(Image, '/front_rgb/image_raw', self.on_rgb, 10)
        self.sub_nir = self.create_subscription(Image, '/front_nir/image_raw', self.on_nir, 10)

        self.pub_ndvi = self.create_publisher(Image, '/front_ndvi/image_raw', 10)

        self.rgb = None
        self.nir = None

    def on_rgb(self, msg):
        self.rgb = msg
        self.try_publish()

    def on_nir(self, msg):
        self.nir = msg
        self.try_publish()

    def try_publish(self):
        if self.rgb is None or self.nir is None:
            return
        rgb = self.bridge.imgmsg_to_cv2(self.rgb, desired_encoding='bgr8')
        nir = self.bridge.imgmsg_to_cv2(self.nir, desired_encoding='mono8').astype(np.float32)

        # Extract red band from RGB
        red = rgb[:, :, 2].astype(np.float32)

        # Compute NDVI = (NIR - Red) / (NIR + Red + eps)
        eps = 1e-6
        ndvi = (nir - red) / (nir + red + eps)
        ndvi = np.clip(ndvi, -1.0, 1.0)

        # Map to 0..255 and colorize for RViz
        ndvi_u8 = ((ndvi + 1.0) * 127.5).astype(np.uint8)
        ndvi_color = cv2.applyColorMap(ndvi_u8, cv2.COLORMAP_TURBO)

        msg = self.bridge.cv2_to_imgmsg(ndvi_color, encoding='bgr8')
        msg.header = self.rgb.header
        self.pub_ndvi.publish(msg)

def main():
    rclpy.init()
    node = NDVINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
