import cv2
import numpy as np


class Painter(object):
    MINIMAP_SCALE = 0.5

    WHITE_COLOR = (255, 255, 255)
    GRAY_COLOR = (128, 128, 128)
    BLACK_COLOR = (0, 0, 0)

    def __init__(self):
        self.painterImage = None

        self.minimapSize = (10, 10)
        self.minimapRectangle = [0, 0, 0, 0]

    def draw_minimap(self, image):
        null_image = np.zeros(image.shape, np.uint8) * 255
        null_image = cv2.rectangle(null_image,
                                   self.minimapRectangle[:2],
                                   self.minimapRectangle[2:4],
                                   self.GRAY_COLOR,
                                   -1,
                                   )
        image = cv2.addWeighted(image,
                                1.,
                                null_image,
                                1.5,
                                1.)
        self.painterImage = image
        return self.painterImage

    def draw_logo_target(self, image, target):
        t = (target.x, target.y)
        image = cv2.drawMarker(image, t, self.BLACK_COLOR, 0)
        return image

    def refresh_image(self, image):
        image = np.array(image, np.uint8)
        self.painterImage = image

        mw = int(image.shape[1] * self.MINIMAP_SCALE)
        mh = int(image.shape[0] * self.MINIMAP_SCALE)
        self.minimapSize = (mw, mh)
        self.minimapRectangle = [10, image.shape[0] - mh - 10, 10 + mw, image.shape[0] - 10]
