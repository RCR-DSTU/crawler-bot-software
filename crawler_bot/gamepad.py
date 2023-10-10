import threading
import time
from threading import Thread
from pyPS4Controller.controller import Controller
from rclpy.parameter import Parameter

from crawler_bot import config


class Gamepad(Controller):
    """
    Инициализация экземпляра класса контроллера (геймпада) Playstation 4 (Dualshock 4) для управления роботом в
    ручном режиме. Работа слушателя осуществляется в параллельном потоке все время существования экземпляра после
    инициализации.
    """
    def __init__(self, timeout: int = 1, **kwargs):
        Controller.__init__(self, **kwargs)
        self.timeOut = timeout
        time.sleep(2)

        self.commonLogger = config.commonLogger
        self.commonLogger.info(" Gamepad initialization started.")

        self.linear_velocity = 0.
        self.angular_velocity = 0.
        self.rosParameters = []

        self.ratio = 0.000030488

        self.threadListener = Thread(target=self.start_listening)
        self.threadListener.start()

    def on_x_press(self):
        pass

    def on_x_release(self):
        pass

    def on_triangle_press(self):
        pass

    def on_triangle_release(self):
        pass

    def on_circle_press(self):
        pass

    def on_circle_release(self):
        pass

    def on_square_press(self):
        pass

    def on_square_release(self):
        pass

    def on_L1_press(self):
        pass

    def on_L1_release(self):
        pass

    def on_L2_press(self, value):
        self.linear_velocity = -(0.5 - (- value) * self.ratio * 0.5)

    def on_L2_release(self):
        self.linear_velocity = 0.

    def on_R1_press(self):
        pass

    def on_R1_release(self):
        pass

    def on_R2_press(self, value):
        self.linear_velocity = 0.5 - (- value) * self.ratio * 0.5

    def on_R2_release(self):
        self.linear_velocity = 0.
        pass

    def on_up_arrow_press(self):
        self.rosParameters = [Parameter('operating_mode', Parameter.Type.INTEGER, 2)]

    def on_up_down_arrow_release(self):
        pass

    def on_down_arrow_press(self):
        self.rosParameters = [Parameter('operating_mode', Parameter.Type.INTEGER, 0)]

    def on_left_arrow_press(self):
        pass

    def on_left_right_arrow_release(self):
        pass

    def on_right_arrow_press(self):
        pass

    def on_R3_up(self, value):
        # Значения меняются от 0 до -32800
        # self.linear_velocity = (- value) * self.ratio
        pass

    def on_R3_down(self, value):
        # Значения меняются от 0 до 32800
        # self.linear_velocity = (- value) * self.ratio
        pass

    def on_R3_left(self, value):
        # Значения меняются от 0 до -32800
        self.angular_velocity = value * self.ratio

    def on_R3_right(self, value):
        # Значения меняются от 0 до 32800
        self.angular_velocity = value * self.ratio

    def on_R3_y_at_rest(self):
        self.linear_velocity = 0.

    def on_R3_x_at_rest(self):
        self.angular_velocity = 0.

    def on_L3_up(self, value):
        pass

    def on_L3_down(self, value):
        pass

    def on_L3_left(self, value):
        pass

    def on_L3_right(self, value):
        pass

    def on_L3_y_at_rest(self):
        pass

    def on_L3_x_at_rest(self):
        pass

    def on_L3_press(self):
        pass

    def on_L3_release(self):
        pass

    def on_R3_press(self):
        pass

    def on_R3_release(self):
        pass

    def on_options_press(self):
        pass

    def on_options_release(self):
        pass

    def on_share_press(self):
        if config.usingCamera:
            self.rosParameters = [Parameter('use_camera', Parameter.Type.BOOL, False)]
        else:
            self.rosParameters = [Parameter('use_camera', Parameter.Type.BOOL, True)]

    def on_share_release(self):
        pass

    def on_playstation_button_press(self):
        try:
            config.mainNode.manualTimer.reset()
        except Exception:
            self.commonLogger.error("Can not reset timer in main node! Fatal!")

    def on_playstation_button_release(self):
        pass

    def start_listening(self):
        self.listen(timeout=self.timeOut)
