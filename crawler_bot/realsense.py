import rclpy
import numpy as np
import pyrealsense2 as rs
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class RealsenseNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.colorPublisher = self.create_publisher(Image, '/color_image', 10)
        self.depthPublisher = self.create_publisher(Image, '/depth_image', 10)
        self.create_timer(0.033, self.timer_callback)

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.profile = None
        self.cvBridge = CvBridge()

        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        self.profile = self.pipeline.start()

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        self.colorPublisher.publish(self.cvBridge.cv2_to_imgmsg(color_image))
        self.depthPublisher.publish(self.cvBridge.cv2_to_imgmsg(depth_image))

        self.get_logger().info(f"Timer tick: {self.get_clock().now().to_msg()}")


def main():
    rclpy.init()
    node = RealsenseNode('realsense')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
