import logging
import time

import cv2
import numpy as np

from ultralytics import YOLO

from crawler_bot import config


def get_dominant_color(image):
    colors, count = np.unique(image.reshape(-1, image.shape[-1]), axis=0, return_counts=True)
    return colors[count.argmax()] / 255


class Logo(object):
    def __init__(self, name="Логотип РЦР"):
        self.logoName = name

        self.logoP1 = (0, 0)
        self.logoP2 = (0, 0)

        self.logoCenter = (.0, .0)

        self.is_visible = False

        self.dominatingColor = np.array((0., 0., 0.), np.float64)

        self.colorProbability = 0.
        self.distanceProbability = 0.

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
        self.possibleLogos = []
        self.followerImage = image
        self.followerTarget = target
        self.followerYOLOv8Model = YOLO(config.nnPath).load(config.nnPath)

    def detect_class(self, image):
        detections = []
        true_detections = []
        # noinspection PyBroadException
        try:
            detections = self.followerYOLOv8Model.predict(image)[0]
        except Exception:
            self.Logger.error(f" No Image!")
            self.followerLogo.is_visible = False
        # noinspection PyBroadException
        try:
            for d in detections:
                if d.boxes.cls[0].item() == config.detectingClass:
                    true_detections.append(d)
        except Exception:
            self.Logger.error(f" No logo on image!")
            self.followerLogo.is_visible = False
        return true_detections

        # if not true_detections:
        #     return None
        # self.followerLogo.is_visible = True
        # if len(true_detections) == 1:
        #     return true_detections[0]
        # true_logo = true_detections[0]
        # dom_colors = []
        # for logo in true_detections:
        #     coord = logo.boxes.data.tolist()[0][:4]
        #     coord = [int(c) for c in coord]
        #     img = image[coord[1]:coord[3], coord[0]:coord[2]]
        #     dom_colors.append(get_dominant_color(img))
        # best_delta = sum(abs(dom_colors[0] - self.followerLogo.dominatingColor))
        # best_color = dom_colors[0]
        # for color, logo in zip(dom_colors[1:], true_detections[1:]):
        #     delta = sum(abs(color - self.followerLogo.dominatingColor))
        #     if delta < best_delta:
        #         true_logo = logo
        #         best_color = color
        # self.followerLogo.dominatingColor = best_color
        # return true_logo

        # Определение среди всех распознанных объектов того, который был ближе всего к объекту на предыдущем кадре
        # true_logo = true_detections[0]
        # true_coord = true_logo.boxes.data.tolist()[0][:4]
        # true_center = [(true_coord[2] + true_coord[0]) / 2, (true_coord[1] + true_coord[3]) / 2]
        # true_dx = abs(true_center[0] - self.followerLogo.logoCenter[0])
        # for logo in true_detections[1:]:
        #     coord = logo.boxes.data.tolist()[0][:4]
        #     center = [(coord[2] + coord[0]) / 2, (coord[1] + coord[3]) / 2]
        #     dx = abs(center[0] - self.followerLogo.logoCenter[0])
        #     if dx < true_dx:
        #         true_dx = dx
        # return true_logo

    def calc_logo_params(self, image, logo_class):
        logo = Logo()
        coord = logo_class.boxes.data.tolist()[0][:4]
        coord = [int(i) for i in coord]
        logo.set_logo_coord(coord)
        center = [int((logo.logoP2[0] + logo.logoP1[0]) / 2),
                  int((logo.logoP2[1] + logo.logoP1[1]) / 2)]
        logo.set_logo_center(center)
        logo.is_visible = True
        img_box = image[coord[1]:coord[3], coord[0]:coord[2]]
        logo.dominatingColor = get_dominant_color(img_box)
        logo.colorProbability = round(sum(abs(logo.dominatingColor - self.followerLogo.dominatingColor)), 3)
        logo.distanceProbability = round(abs(logo.logoCenter[0] - self.followerLogo.logoCenter[0]) / config.imageWidth,
                                         3)
        return logo

    def detect_logo(self, image: np.ndarray):
        detected_class_logos = self.detect_class(image)
        self.possibleLogos.clear()
        if detected_class_logos is not None:
            if len(detected_class_logos) == 1:
                self.followerLogo = self.calc_logo_params(image, detected_class_logos[0])
            else:
                for logo_class in detected_class_logos:
                    logo = self.calc_logo_params(image, logo_class)
                    self.possibleLogos.append(logo)
                true_logo = self.possibleLogos[0]
                for logo in self.possibleLogos[1:]:
                    probability = logo.colorProbability * logo.distanceProbability
                    true_probability = true_logo.colorProbability * true_logo.distanceProbability
                    if probability < true_probability:
                        true_logo = logo
                self.followerLogo = true_logo
        else:
            self.followerLogo = Logo()
            self.Logger.warning(f" No logo on image! \n"
                                f" \t Waiting for logo in frame.")

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

        self.averageAcceleration = config.averageAcceleration

        self.pLinearRatio = -config.pLinearRatio
        self.pAngularRatio = config.pAngularRatio

        self.linearDelta = 0.0
        self.angularDelta = 0.0

    def calculate_velocity_delta(self, image):
        self.logoFollower.detect_logo(image)
        delta_x, delta_y = self.logoFollower.calculate_deltas()

        self.linearDelta = (delta_y / image.shape[1]) * self.pLinearRatio
        self.angularDelta = (delta_x / image.shape[0]) * self.pAngularRatio

        print("")

    def control(self, image):
        past_linear, past_angular = self.linearDelta, self.angularDelta
        self.calculate_velocity_delta(image)
        if self.linearDelta > past_linear:
            self.linearDelta = past_linear + self.averageAcceleration
        elif self.linearDelta < past_linear:
            self.linearDelta = past_linear - self.averageAcceleration
        if self.angularDelta > past_angular:
            self.angularDelta = past_angular + self.averageAcceleration
        elif self.angularDelta < past_angular:
            self.angularDelta = past_angular - self.averageAcceleration

        if self.linearDelta > self.maxLinearVelocity:
            self.linearDelta = self.maxLinearVelocity
        elif self.linearDelta < self.minLinearVelocity:
            self.linearDelta = self.minLinearVelocity

        if self.angularDelta > self.maxAngularVelocity:
            self.angularDelta = self.maxAngularVelocity
        elif self.angularDelta < self.minAngularVelocity:
            self.angularDelta = self.minAngularVelocity

        return self.linearDelta, self.angularDelta


def main():
    image = cv2.imread(config.testImagePath)
    logo_follower_controller = LogoFollowerController()
    logo_follower_controller.control(image)


if __name__ == '__main__':
    main()
