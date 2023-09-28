import cv2
import numpy as np

from ultralytics import YOLO
from ament_index_python import get_package_prefix

from crawler_bot import config


class Logo(object):
    def __init__(self, name="Логотип РЦР"):

        self.logoName = name

        self.logoP1 = (0, 0)
        self.logoP2 = (0, 0)

        self.logoCenter = (0, 0)

        self.is_visible = False

    def set_logo_coord(self, coord: list[int]):
        self.logoP1 = (coord[0], coord[1])
        self.logoP2 = (coord[2], coord[3])

    def set_logo_center(self, center: list[int]):
        self.logoCenter = (center[0], center[1])


class LogoFollower(object):
    """
    Класс детекции логотипа на изображении
    """

    def __init__(self, logo: Logo, target: tuple[int, int], image: np.ndarray = None):
        self.Logger = config.commonLogger
        self.followerLogo = logo
        self.followerImage = image
        self.followerTarget = target
        self.followerYOLOv8Model = YOLO(config.nnPath).load(config.nnPath)

    def detect_class(self, image):
        detections = []
        # noinspection PyBroadException
        try:
            detections = self.followerYOLOv8Model.predict(image)[0]
            self.followerImage = image
        except Exception:
            self.Logger.error(f" No Image!")
            self.followerLogo.is_visible = False
            self.followerImage = None
        # noinspection PyBroadException
        try:
            for d in detections:
                cls = int(d.boxes.cls[0].item())
                if cls == config.detectedClasses:
                    self.followerLogo.is_visible = True
                    return d
                else:
                    self.followerLogo.is_visible = False
        except Exception:
            self.Logger.error(f" No logo on image!")
            self.followerLogo.is_visible = False

    def detect_logo(self, image: np.ndarray):
        detected_class = self.detect_class(image)
        if self.followerLogo.is_visible:
            coord = detected_class.boxes.data.tolist()[0][:4]
            self.followerLogo.set_logo_coord([int(i) for i in coord])
            center = [int((self.followerLogo.logoP2[0] + self.followerLogo.logoP1[0]) / 2),
                      int((self.followerLogo.logoP2[1] + self.followerLogo.logoP1[1]) / 2),
                      ]
            self.followerLogo.set_logo_center(center)
        else:
            self.Logger.warning(f" No logo on image! \n"
                                f" \t Using last coordinates to find logo...")

    def calculate_deltas(self):
        delta_x = self.followerLogo.logoCenter[0] - self.followerTarget[0]
        delta_y = self.followerLogo.logoCenter[1] - self.followerTarget[1]

        return delta_x, delta_y


class LogoFollowerController(object):
    def __init__(self,
                 image_shape: tuple,
                 max_linear_velocity: float = 1.0,
                 max_angular_velocity: float = 1.0,
                 min_linear_velocity: float = -1.0,
                 min_angular_velocity: float = -1.0,
                 ):
        self.Logger = config.commonLogger
        self.logoFollower = LogoFollower(
            Logo("Логотип РЦР"),
            (int(image_shape[0] / 2), int(image_shape[1] / 2))
        )
        self.followerImageShape = image_shape

        self.maxLinearVelocity = max_linear_velocity
        self.maxAngularVelocity = max_angular_velocity
        self.minLinearVelocity = min_linear_velocity
        self.minAngularVelocity = min_angular_velocity

        self.linearDelta = 0.0
        self.angularDelta = 0.0

    def calculate_velocity_delta(self, image):
        self.logoFollower.detect_logo(image)
        d_x, d_y = self.logoFollower.calculate_deltas()

        self.linearDelta = d_y / image.shape[1]
        self.angularDelta = d_x / image.shape[0]

    def control(self, image):
        self.calculate_velocity_delta(image)
        return self.linearDelta, self.angularDelta


def main():
    image = cv2.imread(config.testImagePath)
    logo_follower_controller = LogoFollowerController(image_shape=image.shape[:2])
    logo_follower_controller.control(image)


if __name__ == '__main__':
    main()
