#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class NDVINode(Node):
    def __init__(self):
        super().__init__('ndvi_node')
        self.bridge = CvBridge()
        self.red = None
        self.nir = None
        self.sub_rgb = self.create_subscription(Image, '/survey3s/rgb/image_raw', self.cb_rgb, 10)
        self.sub_nir = self.create_subscription(Image, '/survey3s/nir/image_raw', self.cb_nir, 10)
        self.pub_ndvi = self.create_publisher(Image, '/survey3s/ndvi', 10)
        self.pub_ndvi_vis = self.create_publisher(Image, '/survey3s/ndvi_viz', 10)

    def cb_rgb(self, msg):
        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        # Red channel (uint8)
        self.red = rgb[:, :, 0].astype(np.float32) / 255.0
        self.try_publish(msg.header)

    def cb_nir(self, msg):
        nir_mono = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        self.nir = nir_mono.astype(np.float32) / 255.0
        self.try_publish(msg.header)

    def try_publish(self, header):
        if self.red is None or self.nir is None:
            return
        # NDVI = (NIR - RED) / (NIR + RED + eps)
        eps = 1e-6
        ndvi = (self.nir - self.red) / (self.nir + self.red + eps)
        ndvi = np.clip(ndvi, -1.0, 1.0)

        # Publish 32FC1
        ndvi_msg = self.bridge.cv2_to_imgmsg(ndvi, encoding='32FC1')
        ndvi_msg.header = header
        self.pub_ndvi.publish(ndvi_msg)

        # Colorized preview (0..255)
        ndvi_u8 = ((ndvi + 1.0) * 127.5).astype(np.uint8)  # map -1..1 -> 0..255
        ndvi_color = cv2.applyColorMap(ndvi_u8, cv2.COLORMAP_TURBO)
        ndvi_color_msg = self.bridge.cv2_to_imgmsg(ndvi_color, encoding='bgr8')
        ndvi_color_msg.header = header
        self.pub_ndvi_vis.publish(ndvi_color_msg)

def main():
    rclpy.init()
    node = NDVINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
