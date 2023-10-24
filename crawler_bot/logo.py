import logging
import cv2
import numpy as np

from ultralytics import YOLO

from crawler_bot import config


class Logo(object):
    def __init__(self, name="Логотип РЦР"):

        self.logoName = name

        self.logoP1 = (0, 0)
        self.logoP2 = (0, 0)

        self.logoCenter = (.0, .0)

        self.is_visible = False

    def set_logo_coord(self, coord):
        self.logoP1 = (coord[0], coord[1])
        self.logoP2 = (coord[2], coord[3])

    def set_logo_center(self, center):
        self.logoCenter = (float(center[0]), float(center[1]))


class LogoFollower(object):
    """
    Класс детекции логотипа на изображении
    """

    def __init__(self, logo: Logo, target, image: np.ndarray = None):
        self.Logger = logging.Logger("logger")
        self.followerLogo = logo
        self.followerImage = image
        self.followerTarget = target
        self.followerYOLOv8Model = YOLO(config.nnPath).load(config.nnPath)

    def detect_class(self, image):
        detections = []
        true_detections = []
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
                if cls == config.detectingClass:
                    true_detections.append(d)
                else:
                    self.followerLogo.is_visible = False
        except Exception:
            self.Logger.error(f" No logo on image!")
            self.followerLogo.is_visible = False
        if not true_detections:
            return None
        self.followerLogo.is_visible = True
        if len(true_detections) == 1:
            return true_detections[0]
        true_logo = true_detections[0]
        true_coord = true_logo.boxes.data.tolist()[0][:4]
        true_center = [(true_coord[2] + true_coord[0]) / 2, (true_coord[1] + true_coord[3]) / 2]
        true_dx = abs(true_center[0] - self.followerLogo.logoCenter[0])
        for logo in true_detections[1:]:
            coord = logo.boxes.data.tolist()[0][:4]
            center = [(coord[2] + coord[0]) / 2, (coord[1] + coord[3]) / 2]
            dx = abs(center[0] - self.followerLogo.logoCenter[0])
            if dx < true_dx:
                true_dx = dx
        return true_logo

    def detect_logo(self, image: np.ndarray):
        detected_class = self.detect_class(image)
        if self.followerLogo.is_visible and detected_class is not None:
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
        delta_x = (self.followerLogo.logoCenter[0] - self.followerTarget[0])
        delta_y = (self.followerLogo.logoCenter[1] - self.followerTarget[1])

        return delta_x, delta_y


class LogoFollowerController(object):
    def __init__(self):
        self.Logger = logging.Logger('logger')
        self.logoFollower = LogoFollower(
            Logo(config.detectingLogoName),
            (int(config.imageWidth / 2), int(config.imageHeight / 2))
        )
        self.followerImageShape = (config.imageWidth, config.imageHeight)

        self.maxLinearVelocity = config.max_linear_velocity
        self.maxAngularVelocity = config.max_angular_velocity
        self.minLinearVelocity = config.min_linear_velocity
        self.minAngularVelocity = config.min_angular_velocity

        self.pLinearRatio = -1.5
        self.pAngularRatio = 1.5

        self.linearDelta = 0.0
        self.angularDelta = 0.0

    def calculate_velocity_delta(self, image):
        self.logoFollower.detect_logo(image)
        delta_x, delta_y = self.logoFollower.calculate_deltas()

        self.linearDelta = (delta_y / image.shape[1] + 0.85) * self.pLinearRatio
        self.angularDelta = (delta_x / image.shape[0] + 2.7) * self.pAngularRatio

    def control(self, image):
        self.calculate_velocity_delta(image)
        return self.linearDelta, self.angularDelta


def main():
    image = cv2.imread(config.testImagePath)
    logo_follower_controller = LogoFollowerController()
    logo_follower_controller.control(image)


if __name__ == '__main__':
    main()
