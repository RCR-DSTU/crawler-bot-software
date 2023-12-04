import rclpy
import numpy as np

from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from ament_index_python import get_package_prefix

from crawler_bot import debug


# Neural network model type: YOLOv8
nnModel = 'yolov8n.yaml'
# Path to the file with weight coefficients of the neural network
nnPath = f"{get_package_prefix('crawler_bot')}/share/crawler_bot/network/model.pt"


def get_dominant_color(image):
    colors, count = np.unique(image.reshape(-1, image.shape[-1]), axis=0, return_counts=True)
    return colors[count.argmax()] / 255


class Point(object):
    def __init__(self, x, y, z=0.):
        self.x = x
        self.y = y
        self.z = z


class Box(object):
    def __init__(self, name="РЦР"):
        self.targetName = name

        self.p1 = Point(0, 0)
        self.p2 = Point(0, 0)
        self.pCenter = Point(0, 0)
        self.depth = 0

        self.is_visible = False
        self.dominatingColor = np.array((0., 0., 0.), np.float64)

        self.colorProbability = 0.
        self.distanceProbability = 0.
        self.depthProbability = 0.

        self.probability = 0.

    def set_coord(self, coord):
        self.p1.x, self.p1.y = coord[0], coord[1]
        self.p2.x, self.p2.y = coord[2], coord[3]

    def set_center(self, center):
        self.pCenter.x, self.pCenter.y = center[0], center[1]


class BoxFollower(object):

    def __init__(self, box: Box, target: Point, node: Node):
        self.box = box
        self.possibleBoxes = []
        self.target = target
        self.YOLOv8Model = YOLO(nnPath).load(nnPath)
        self.node = node

    def detect_class(self, image, class_num):
        detections = []
        true_detections = []
        # noinspection PyBroadException
        try:
            detections = self.YOLOv8Model.predict(image, verbose=False)[0]
        except Exception:
            self.node.get_logger().error("The detector is not receiving an image!!!")
            self.box.is_visible = False
        # noinspection PyBroadException
        try:
            for d in detections:
                if d.boxes.cls[0].item() == class_num:
                    true_detections.append(d)
        except Exception:
            self.node.get_logger().info(f"No detectable object in the image.")
            self.box.is_visible = False
        return true_detections

    def calc_box_params(self, image: np.ndarray, depth, box_class):
        box = Box()
        coord = box_class.boxes.data.tolist()[0][:4]
        coord = [int(i) for i in coord]
        box.set_coord(coord)
        center = [int((box.p2.x + box.p1.x) / 2),
                  int((box.p2.y + box.p1.y) / 2)]
        box.set_center(center)
        box.is_visible = True
        box.depth = depth[box.pCenter.y, box.pCenter.x] / 255
        img_box = image[coord[1]:coord[3], coord[0]:coord[2]]
        dom = get_dominant_color(img_box)
        pp = np.array((0, 0, 0))
        if dom.any() != pp.any():
            box.dominatingColor = dom
        box.depthProbability = round(abs(box.depth - self.box.depth), 3) * 2
        box.colorProbability = round(sum(abs(box.dominatingColor - self.box.dominatingColor)), 3)
        box.distanceProbability = round(abs(box.pCenter.x - self.box.pCenter.x) / image.shape[0], 3)
        if box.depth > 0.15:
            box.probability = box.colorProbability * box.distanceProbability * box.depthProbability
        else:
            box.probability = box.colorProbability * box.distanceProbability
        return box

    def detect(self, image: np.ndarray, depth, class_num):
        detected_boxes = self.detect_class(image, class_num)
        self.possibleBoxes.clear()
        if len(detected_boxes) == 0:
            self.box.is_visible = False
        elif len(detected_boxes) == 1:
            self.box = self.calc_box_params(image, depth, detected_boxes[0])
        else:
            for box_class in detected_boxes:
                box = self.calc_box_params(image, depth, box_class)
                self.possibleBoxes.append(box)
            true_box = self.possibleBoxes[0]
            for box in self.possibleBoxes[1:]:
                if box.probability < true_box.probability:
                    true_box = box
            self.possibleBoxes.remove(true_box)
            self.box = true_box

    def calculate_deltas(self):
        delta_x = (self.box.pCenter.x - self.target.x)
        delta_y = (self.box.depth * 255 - self.target.z)

        return delta_x, delta_y


class BoxDetector(Node):
    CLASSES = {
        'person': 0,
    }

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('detecting_class', 'person')
        self.declare_parameter('target_x', 240)
        self.declare_parameter('target_z', 1.)
        self.declare_parameter('linear_proportional_term', 1.5)
        self.declare_parameter('angular_proportional_term', 1.0)
        self.declare_parameter('frequency', 25)

        self.detectingClassName = self.get_parameter('detecting_class').get_parameter_value().string_value
        self.targetX = self.get_parameter('target_x').get_parameter_value().integer_value
        self.targetZ = self.get_parameter('target_z').get_parameter_value().integer_value
        self.pLinearRatio = - self.get_parameter('linear_proportional_term').get_parameter_value().double_value
        self.pAngularRatio = self.get_parameter('angular_proportional_term').get_parameter_value().double_value
        self.timerPeriod = 1 / self.get_parameter('frequency').get_parameter_value().integer_value

        self.detectingClassNumber = self.CLASSES.get(self.detectingClassName)
        self.follower = BoxFollower(box=Box(self.detectingClassName),
                                    target=Point(self.targetX, 0, self.targetZ * 42.5),
                                    node=self)
        self.cvBridge = CvBridge()

        self.linearDelta = 0.0
        self.angularDelta = 0.0

        self.image = None
        self.depth = None
        self.targetMsg = Twist()
        self.rawTwistMsg = Twist()
        self.colorImageSubscriber = self.create_subscription(CompressedImage,
                                                             '/realsense/color_image',
                                                             self.color_image_callback,
                                                             10)
        self.colorImageSubscriber = self.create_subscription(CompressedImage,
                                                             '/realsense/depth_image',
                                                             self.depth_image_callback,
                                                             10)
        self.rawTwistPublisher = self.create_publisher(Twist,
                                                       '/detector/raw_auto_twist',
                                                       10)
        self.targetPublisher = self.create_publisher(Twist,
                                                     '/detector/target_box',
                                                     10)
        self.mainTimer = self.create_timer(self.timerPeriod, self.timer_callback)

    def timer_callback(self):

        self.targetZ = self.get_parameter('target_z').get_parameter_value().double_value
        self.follower.target.z = self.targetZ

        if self.depth is None or self.image is None:
            self.get_logger().error("No image message received!!!")
        else:
            x, z = self.calculate_velocity_delta(self.image, self.depth, self.detectingClassNumber)
            self.rawTwistMsg.linear.x = x
            self.rawTwistMsg.angular.z = z
            self.rawTwistPublisher.publish(self.rawTwistMsg)

            self.targetMsg.linear.x = float(self.follower.box.p1.x)
            self.targetMsg.linear.y = float(self.follower.box.p1.y)
            self.targetMsg.angular.x = float(self.follower.box.p2.x)
            self.targetMsg.angular.y = float(self.follower.box.p2.y)
            self.targetPublisher.publish(self.targetMsg)

            # debug.show(self)

    def color_image_callback(self, msg):
        self.image = self.cvBridge.compressed_imgmsg_to_cv2(msg)

    def depth_image_callback(self, msg):
        self.depth = self.cvBridge.compressed_imgmsg_to_cv2(msg)

    def calculate_velocity_delta(self, image, depth, class_num):
        self.follower.detect(image, depth, class_num)
        delta_x, delta_y = self.follower.calculate_deltas()

        self.linearDelta = (delta_y / image.shape[1]) * self.pLinearRatio
        self.angularDelta = (delta_x / image.shape[0]) * self.pAngularRatio

        return self.linearDelta, self.angularDelta


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector("detector")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
