import cv2
import time
import logging
import numpy as np
import pyrealsense2 as rs

from crawler_bot import config


class IntelRealSenseCamera(object):
    """
    Класс камеры Intel RealSense
    Подключение к камере, запуск потоков цвета и/или глубины, считывание кадров изображения
    """

    FRONT = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3

    CAMERA_POSITIONS = {
        0: 'Front',
        1: 'Back',
        2: 'Left',
        3: 'Right',
    }

    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480

    colorEnabled = True
    depthEnabled = True

    def __init__(self,
                 camera_id: str = None,
                 camera_pos: int = FRONT,
                 color_enabled: bool = colorEnabled,
                 depth_enabled: bool = depthEnabled,
                 ):

        self.configLogger = config.commonLogger

        self.is_connected = False
        self.currentFrames = None

        self.cameraId = camera_id
        self.cameraPosition = camera_pos
        self.colorEnabled = color_enabled
        self.depthEnabled = depth_enabled

        self.streamProfile = None
        self.cameraConfig = rs.config()
        self.cameraPipeline = rs.pipeline()

        self.connect_device_by_id()

    def init_rs_instances(self):
        self.streamProfile = None
        self.cameraConfig = rs.config()
        self.cameraPipeline = rs.pipeline()

    def find_rs_devices(self):
        if len(rs.context().devices) > 0:
            num = 0
            for d in rs.context().devices:
                self.configLogger.info(f' Found IntelRealSense devices: \n '
                                       f' \t{num})'
                                       f' Model - {d.get_info(rs.camera_info.name)} |'
                                       f' ID - {d.get_info(rs.camera_info.serial_number)}')
            return True
        else:
            self.configLogger.info(f" No IntelRealSense Device connected!")
            return False

    def check_connection(self):
        try:
            for d in rs.context().devices:
                if d.get_info(rs.camera_info.serial_number) == self.cameraId:
                    return True
            self.configLogger.error(f"Can not find camera device with ID: {self.cameraId}")
            self.is_connected = False
            self.init_rs_instances()
            return False
        except Exception:
            self.configLogger.error(f"Camera was disconnected!")
            self.is_connected = False
            self.init_rs_instances()
            return False

    def connect_device_by_id(self):
        len_devices = len(rs.context().devices)
        if len_devices == 0:
            self.configLogger.error("No connected camera devices!")
            self.is_connected = False
            return False
        else:
            if self.cameraId is None:
                self.configLogger.warning(f"Camera ID is not specified. Trying to connect first available.")
                self.cameraId = rs.context().devices[0].get_info(rs.camera_info.serial_number)
                self.configLogger.info(f"Connected to Realsense Camera ID: {self.cameraId}")
                self.is_connected = True
                self.start_streams()
                return True
            else:
                for d in rs.context().devices:
                    if d.get_info(rs.camera_info.serial_number) == self.cameraId:
                        self.is_connected = True
                        self.start_streams()
                        return True

    def start_streams(self):
        try:
            self.cameraConfig.enable_device(self.cameraId)
            if self.colorEnabled:
                self.cameraConfig.enable_stream(rs.stream.color,
                                                self.CAMERA_WIDTH,
                                                self.CAMERA_HEIGHT,
                                                rs.format.bgr8, 30)
                self.configLogger.info(f" Color stream enabled.")
            if self.depthEnabled:
                self.cameraConfig.enable_stream(rs.stream.depth,
                                                self.CAMERA_WIDTH,
                                                self.CAMERA_HEIGHT,
                                                rs.format.z16, 30)
                self.configLogger.info(" Depth stream enabled.")
            self.streamProfile = self.cameraPipeline.start(self.cameraConfig)
            self.configLogger.info(f" Streams was started.")
        except Exception:
            self.configLogger.info(f" Streams is already running. \n"
                                   f"     If you want to start another stream, "
                                   f"first finish all the current ones and restart with a new \n"
                                   f"         Use class param <color_enabled> or/and <depth_enabled> when "
                                   f"initialization")

    def stop_all_stream(self):
        self.cameraConfig.disable_all_streams()
        logging.info(f" Camera streams was stopped. \n"
                     "  Color and Depth stream disabled.")

    def read_frames(self):
        if self.check_connection():
            if self.is_connected:
                self.currentFrames = self.cameraPipeline.wait_for_frames()
            else:
                if self.connect_device_by_id():
                    self.currentFrames = self.cameraPipeline.wait_for_frames()

    def get_color_frame(self):
        color_frame = self.currentFrames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def get_depth_frame(self):
        depth_frame = self.currentFrames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        return depth_image


class IntelRealSenseCameraD415(IntelRealSenseCamera):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)


class IntelRealSenseCameraD455(IntelRealSenseCamera):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)


def main():
    camera = IntelRealSenseCameraD455(color_enabled=True, depth_enabled=True)
    while True:
        camera.read_frames()
        color_img = camera.get_color_frame()
        depth_img = camera.get_depth_frame()
        cv2.imshow("Color Image", color_img)
        cv2.imshow("Depth Image", depth_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit()


if __name__ == '__main__':
    main()
