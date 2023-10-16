import cv2
import rclpy
import numpy as np
import pyrealsense2 as rs
from crawler_bot import config, logo
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point, Twist


class RealsenseNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.colorPublisher = self.create_publisher(CompressedImage, '/color_image', 10)
        self.depthPublisher = self.create_publisher(CompressedImage, '/depth_image', 10)
        self.targetPublisher = self.create_publisher(Point, '/color_image/target', 10)
        self.velocityPublisher = self.create_publisher(Twist, '/color_image/velocity', 10)
        self.create_timer(config.TIMER_PERIOD, self.timer_callback)

        self.logoFollowerController = logo.LogoFollowerController((config.imageWidth, config.imageHeight))

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.profile = None
        self.cvBridge = CvBridge()

        self.config.enable_stream(rs.stream.color, config.imageWidth, config.imageHeight, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, config.imageWidth, config.imageHeight, rs.format.z16, 30)

        self.profile = self.pipeline.start()

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        shape = (int(config.imageWidth / 4), int(config.imageHeight / 4))
        color_image = cv2.resize(color_image, shape)
        depth_image = cv2.resize(depth_image, shape)

        self.colorPublisher.publish(self.cvBridge.cv2_to_compressed_imgmsg(color_image))
        self.depthPublisher.publish(self.cvBridge.cv2_to_compressed_imgmsg(depth_image))

        twist = Twist()
        twist.linear.x, twist.angular.z = self.logoFollowerController.control(color_image)

        point = Point()
        point.x, point.y = self.logoFollowerController.logoFollower.followerLogo.logoCenter

        self.targetPublisher.publish(point)
        self.velocityPublisher.publish(twist)


def main():
    rclpy.init()
    node = RealsenseNode('realsense')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
