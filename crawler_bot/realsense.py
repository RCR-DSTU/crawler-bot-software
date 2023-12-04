import cv2
import time
import rclpy

import numpy as np
import pyrealsense2 as rs

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


class RealsenseNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('image_width', 480)
        self.declare_parameter('image_height', 270)
        self.declare_parameter('print_fps', False)
        self.declare_parameter('frequency', 25)

        self.imageWidth = self.get_parameter('image_width').get_parameter_value().integer_value
        self.imageHeight = self.get_parameter('image_height').get_parameter_value().integer_value
        self.printFPS = self.get_parameter('print_fps').get_parameter_value().bool_value
        self.timerPeriod = 1 / self.get_parameter('frequency').get_parameter_value().integer_value

        self.colorPublisher = self.create_publisher(CompressedImage, f'/{node_name}/color_image', 10)
        self.depthPublisher = self.create_publisher(CompressedImage, f'/{node_name}/depth_image', 10)
        self.cameraTimer = self.create_timer(self.timerPeriod, self.camera_callback)
        self.reconnectTimer = self.create_timer(3, self.reconnect_callback)

        self.cameraTimer.cancel()
        self.reconnectTimer.cancel()

        self.cvBridge = CvBridge()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.profile = None

        self.config.enable_stream(rs.stream.color, self.imageWidth, self.imageHeight, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.imageWidth, self.imageHeight, rs.format.z16, 30)

        if len(rs.context().devices) != 0:
            try:
                self.profile = self.pipeline.start(self.config)
                if self.reconnectTimer.is_ready():
                    self.reconnectTimer.cancel()
                if self.cameraTimer.is_canceled():
                    self.cameraTimer.reset()
                self.get_logger().info("Color and depth streams are running...")
            except Exception:
                self.warn_messages()
                self.reconnectTimer.reset()
                self.cameraTimer.cancel()
        else:
            self.get_logger().error("Realsense disconnected!!!")
            self.reconnectTimer.reset()

    def warn_messages(self):
        self.get_logger().error("Cannot start image streams from the camera. "
                                "The streams may already be in use by a third-party application. ")
        self.get_logger().error("This error also occurs when the realsense is not connected via USB 3.0+."
                                " Check the connection port and the cable!!!")
        self.get_logger().warn("---\nIt is possible to run full-fledged streams with high resolution, \n "
                               "\tonly if 1 realsense camera is connected to 1 usb controller of computer. "
                               "However, 2 usb ports 3.0 can be output from 1 controller panel of computer. \n"
                               "\tAlso check the resolution of the stream. "
                               "For realsense D455 available 16:9 streams starting from (1280, 720) and less \n"
                               )

    def camera_callback(self):
        start_time = time.time()

        try:
            frames = self.pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            shape = (int(self.imageWidth), int(self.imageHeight))
            color_image = cv2.resize(color_image, shape)
            depth_image = cv2.resize(depth_image, shape) / 6200 * 255
            depth_image = np.array(depth_image, np.uint8)

            self.colorPublisher.publish(self.cvBridge.cv2_to_compressed_imgmsg(color_image))
            self.depthPublisher.publish(self.cvBridge.cv2_to_compressed_imgmsg(depth_image))

        except Exception:
            self.reconnectTimer.reset()
            self.cameraTimer.cancel()

        if self.printFPS:
            print("Image stream FPS: ", 1.0 / (time.time() - start_time))

    def reconnect_callback(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass

        if len(rs.context().devices) != 0:
            try:
                self.pipeline = rs.pipeline()
                self.config = rs.config()
                self.profile = None

                self.config.enable_stream(rs.stream.color, self.imageWidth, self.imageHeight, rs.format.bgr8, 30)
                self.config.enable_stream(rs.stream.depth, self.imageWidth, self.imageHeight, rs.format.z16, 30)
                self.profile = self.pipeline.start(self.config)

                self.reconnectTimer.cancel()
                self.cameraTimer.reset()
                self.get_logger().info("Color and depth streams are running...")
            except Exception:
                self.warn_messages()
                self.reconnectTimer.reset()
                self.cameraTimer.cancel()
        else:
            self.get_logger().error("Realsense disconnected!!!")
            self.reconnectTimer.reset()


def main():
    rclpy.init()
    node = RealsenseNode('realsense')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
