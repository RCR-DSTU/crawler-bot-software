import cv2
import realsense
import numpy as np

from crawler_bot import config


class Painter(object):

    MINIMAP_SCALE = 0.5

    WHITE_COLOR = (255, 255, 255)
    GRAY_COLOR = (128, 128, 128)
    BLACK_COLOR = (0, 0, 0)

    def __init__(self):
        self.Logger = config.commonLogger

        self.painterImage = None
        self.minimapSize = (10, 10)
        self.minimapRectangle = [0, 0, 0, 0]

    def draw_minimap(self, dx, dy):
        null_image = np.zeros(self.painterImage.shape, np.uint8) * 255
        null_image = cv2.rectangle(null_image,
                                   self.minimapRectangle[:2],
                                   self.minimapRectangle[2:4],
                                   self.GRAY_COLOR,
                                   -1,
                                   )
        self.painterImage = cv2.addWeighted(self.painterImage,
                                            1.,
                                            null_image,
                                            1.5,
                                            1.)

        return self.painterImage

    def refresh_image(self, image):
        image = np.array(image, np.uint8)
        self.painterImage = image

        mw = int(image.shape[1] * self.MINIMAP_SCALE)
        mh = int(image.shape[0] * self.MINIMAP_SCALE)
        self.minimapSize = (mw, mh)
        self.minimapRectangle = [10, image.shape[0] - mh - 10, 10 + mw, image.shape[0] - 10]


camera = realsense.IntelRealSenseCameraD455()
camera.read_frames()
color_frame = camera.get_color_frame()

painter = Painter()
painter.refresh_image(color_frame)
# painterImage = painter.draw_minimap(0, 0)

while 1:
    camera.read_frames()
    color_frame = camera.get_color_frame()
    painter.refresh_image(color_frame)
    painterImage = painter.draw_minimap(0, 0)
    cv2.imshow("Image", painterImage)
    cv2.waitKey(1)
