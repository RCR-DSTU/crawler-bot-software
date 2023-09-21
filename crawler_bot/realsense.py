import cv2
import time
import logging
import numpy as np
import pyrealsense2 as rs

from folder.crawler_bot import config


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

    ColorEnabled = True
    DepthEnabled = True

    def __init__(self,
                 camera_id: str = None,
                 camera_pos: int = FRONT,
                 color_enabled: bool = ColorEnabled,
                 depth_enabled: bool = DepthEnabled,
                 ):

        self.Logger = config.common_logger

        self.CameraDevice = None
        self.CameraPosition = camera_pos
        self.ColorEnabled = color_enabled
        self.DepthEnabled = depth_enabled

        self.StreamProfile = None
        self.Context = rs.context()
        self.PipeLine = rs.pipeline()
        self.StreamConfig = rs.config()
        self.CurrentFrames = None

        self.connect_device_by_id(camera_id)
        self.start_streams()

    def connect_device_by_id(self, camera_id: str):
        if camera_id is None:
            if self.find_rs_devices():
                if len(self.Context.devices) == 1:
                    self.CameraDevice = self.Context.devices[0]
                else:
                    while self.CameraDevice is None:

                        num = input(f' Please choose device to connect from listed above: \n'
                                    f' Enter the number: >> ')
                        try:
                            num = int(num)
                        except Exception:
                            self.Logger.error(f" <{num}> is not integer!")
                            time.sleep(0.5)
                            print(" Try again.")
                        try:
                            self.CameraDevice = self.Context.devices[num]
                            self.Logger.info(
                                f" Device with index <{num}> | "
                                f"{self.CameraDevice.get_info(rs.camera_info.name)} | "
                                f"ID - {self.CameraDevice.get_info(rs.camera_info.serial_number)} was connected.")
                        except Exception:
                            self.Logger.error(
                                f" Device with index <{num}> is not exist!")
                            time.sleep(0.5)
                            print(" Try again.")
            else:
                self.Logger.error(f" Please connect devices and restart the program!")
                exit()
        else:
            num = 0
            for d in self.Context.devices:
                if d.get_info(rs.camera_info.serial_number) == camera_id:
                    self.CameraDevice = self.Context.devices[num]
                    break
                num += 1
            if self.CameraDevice is None:
                self.Logger.error(f" Device with id <{camera_id}> is not exist. \n"
                                  f" \tPlease correct device id and restart the program!")
                exit()
        self.StreamConfig.enable_device(self.CameraDevice.get_info(rs.camera_info.serial_number))

    def find_rs_devices(self):
        if len(self.Context.devices) > 0:
            num = 0
            for d in self.Context.devices:
                self.Logger.info(f' Found IntelRealSense devices: \n '
                                 f' \t{num})'
                                 f' Model - {d.get_info(rs.camera_info.name)} |'
                                 f' ID - {d.get_info(rs.camera_info.serial_number)}')
            return True
        else:
            self.Logger.info(f" No IntelRealSense Device connected!")
            return False

    def start_streams(self):
        try:
            if self.ColorEnabled:
                self.StreamConfig.enable_stream(rs.stream.color,
                                                self.CAMERA_WIDTH,
                                                self.CAMERA_HEIGHT,
                                                rs.format.bgr8, 30)
                self.Logger.info(f" Color stream enabled.")
            if self.DepthEnabled:
                self.StreamConfig.enable_stream(rs.stream.depth,
                                                self.CAMERA_WIDTH,
                                                self.CAMERA_HEIGHT,
                                                rs.format.z16, 30)
                self.Logger.info(" Depth stream enabled.")
            self.StreamProfile = self.PipeLine.start(self.StreamConfig)
            self.Logger.info(f" Streams was started.")
        except Exception:
            self.Logger.info(f" Streams is already running. \n"
                             f"     If you want to start another stream, "
                             f"first finish all the current ones and restart with a new \n"
                             f"         Use class param <color_enabled> or/and <depth_enabled> when initialization")

    def stop_all_stream(self):
        self.StreamConfig.disable_all_streams()
        logging.info(f" Camera streams was stopped. \n"
                     "  Color and Depth stream disabled.")

    def read_frames(self):
        self.CurrentFrames = self.PipeLine.wait_for_frames()

    def get_color_frame(self):
        if self.ColorEnabled:
            color_frame = self.CurrentFrames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            return color_image
        else:
            self.Logger.error(f" Can not get color frame. Color stream was disabled when initialization!")

    def get_depth_frame(self):
        if self.DepthEnabled:
            depth_frame = self.CurrentFrames.get_depth_frame()
            depth_image = np.asanyarray(depth_frame.get_data())
            return depth_image
        else:
            self.Logger.error(f" Can not get color frame. Color stream was disabled when initialization!")


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
