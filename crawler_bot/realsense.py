import cv2
import time
import rclpy
import numpy as np
import pyrealsense2 as rs
from crawler_bot import config, logo
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Point


class RealsenseNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.colorPublisher = self.create_publisher(CompressedImage, '/realsense/color_image', 10)
        self.depthPublisher = self.create_publisher(CompressedImage, '/realsense/depth_image', 10)
        self.targetPublisher = self.create_publisher(Point, '/realsense/target', 10)
        self.velocityPublisher = self.create_publisher(Twist, '/realsense/twist', 10)
        self.create_timer(config.TIMER_PERIOD, self.timer_callback)

        self.declare_parameter('average_acceleration', 0.01)

        self.logoFollowerController = logo.LogoFollowerController()

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.profile = None
        self.cvBridge = CvBridge()

        self.config.enable_stream(rs.stream.color, config.imageWidth, config.imageHeight, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, config.imageWidth, config.imageHeight, rs.format.z16, 30)

        try:
            self.profile = self.pipeline.start()
        except Exception as e:
            print("Realsense is disconnected!")
            raise e
        self.get_logger().info(f"Realsense Node was started...")

        cv2.namedWindow("Realsense Image Window", cv2.WINDOW_AUTOSIZE)
        self.cvKey = 255

    def timer_callback(self):
        start_time = time.time()

        self.logoFollowerController.averageAcceleration = (self.get_parameter('average_acceleration').
                                                           get_parameter_value().double_value)
        # noinspection PyBroadException
        try:
            frames = self.pipeline.wait_for_frames()

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            shape = (int(config.imageWidth), int(config.imageHeight))
            color_image = cv2.resize(color_image, shape)
            depth_image = cv2.resize(depth_image, shape)

            self.colorPublisher.publish(self.cvBridge.cv2_to_compressed_imgmsg(color_image))
            self.depthPublisher.publish(self.cvBridge.cv2_to_compressed_imgmsg(depth_image))

            twist = Twist()
            point = Point()
            x, z = self.logoFollowerController.control(color_image)

            if self.logoFollowerController.logoFollower.followerLogo.is_visible:
                point.x, point.y = self.logoFollowerController.logoFollower.followerLogo.logoCenter
                twist.linear.x, twist.angular.z = x, z

            self.targetPublisher.publish(point)
            self.velocityPublisher.publish(twist)

            # print("FPS: ", 1.0 / (time.time() - start_time))

            key = cv2.waitKey(1) & 0xFF
            if key != 255:
                self.cvKey = key

            if self.cvKey == ord('1'):
                # ---
                image = color_image
                image = cv2.resize(image, (1280, 720))
                for p_logo in self.logoFollowerController.logoFollower.possibleLogos:
                    image = cv2.rectangle(image, p_logo.logoP1,
                                          p_logo.logoP2,
                                          (0, 255, 255), 1)
                    image = cv2.putText(image,
                                        f"{p_logo.distanceProbability} "
                                        f"{p_logo.colorProbability}",
                                        p_logo.logoP1,
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255))

                image = cv2.putText(image,
                                    f"FPS: {int(1.0 / (time.time() - start_time))}",
                                    (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255))
                image = cv2.rectangle(image, self.logoFollowerController.logoFollower.followerLogo.logoP1,
                                      self.logoFollowerController.logoFollower.followerLogo.logoP2,
                                      (0, 255, 0), 1)
                image = cv2.putText(image,
                                    f"{self.logoFollowerController.logoFollower.followerLogo.distanceProbability} "
                                    f"{self.logoFollowerController.logoFollower.followerLogo.colorProbability}",
                                    self.logoFollowerController.logoFollower.followerLogo.logoP1,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255))

                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                cv2.imshow("Realsense Image Window", image)
                cv2.waitKey(1)
                # ---
        except Exception:
            self.get_logger().warn(f"Realsense Node can not receive or process frames!")


def main():
    rclpy.init()
    node = RealsenseNode('realsense')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
