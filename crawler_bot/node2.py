import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from crawler_bot import config, realsense


class RealsenseNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.commonLogger = config.commonLogger

        self.controlTimer = self.create_timer(0.02,
                                              callback=self.control_timer_callback)

        self.cvBridge = CvBridge()
        self.colorImage = None
        self.depthImage = None

        self.colorImagePublisher = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.depthImagePublisher = self.create_publisher(Image, "/camera/depth/image_rect_raw", 10)

        self.rsCamera = realsense.IntelRealSenseCameraD455()

    def control_timer_callback(self):
        self.rsCamera.read_frames()
        color_img = self.rsCamera.get_color_frame()
        depth_img = self.rsCamera.get_depth_frame()

        self.colorImagePublisher.publish(self.cvBridge.cv2_to_imgmsg(color_img))
        self.depthImagePublisher.publish(self.cvBridge.cv2_to_imgmsg(depth_img))


def main(args=None):
    rclpy.init(args=args)
    config.realsenseNode = RealsenseNode("realsense")
    rclpy.spin(config.realsenseNode)
    config.realsenseNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
