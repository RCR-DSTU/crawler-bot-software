import cv2
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist

from crawler_bot import realsense, gamepad, config, logo


class LogoFollowerNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.Logger = config.common_logger

        self.ModeSwitchFlag = True
        self.debugTimer = self.create_timer(timer_period_sec=1.,
                                            callback=self.debug_timer_callback)

        self.manualTimer = self.create_timer(timer_period_sec=0.05,
                                             callback=self.manual_timer_callback)

        self.autoTimer = self.create_timer(1.,
                                           callback=self.auto_timer_callback)

        self.cameraTimer = self.create_timer(0.05,
                                             callback=self.camera_timer_callback)

        self.debugTimer.cancel()
        self.manualTimer.cancel()
        self.autoTimer.cancel()
        self.cameraTimer.cancel()

        self.controlTimer = self.create_timer(1.,
                                              callback=self.control_timer_callback)

        self.declare_parameter('linear_velocity_x', 0.0)
        self.declare_parameter('linear_velocity_y', 0.0)
        self.declare_parameter('linear_velocity_z', 0.0)
        self.declare_parameter('angular_velocity_x', 0.0)
        self.declare_parameter('angular_velocity_y', 0.0)
        self.declare_parameter('angular_velocity_z', 0.0)

        self.declare_parameter('operating_mode', config.OperatingMode)
        self.declare_parameter('use_camera', config.use_camera)

        self.SpeedTwistPublisher = self.create_publisher(msg_type=Twist,
                                                         topic="/crawler_bot/twist",
                                                         qos_profile=10,
                                                         )

        self.Gamepad = gamepad.Gamepad(interface=config.gamepad_interface, connecting_using_ds4drv=False)
        self.camera = None
        self.controller = None

        self.SpeedTwist = Twist()

    def control_timer_callback(self):
        """
        Функция, контролирующая все переключения в программе.
        :return:
        """

        """Считываем параметр <operating_mode> из экосистемы ROS2, если на геймпаде произошло нажатие клавиши 
        <Arrow_up> или <Arrow_down>, то в параметр экосистемы запишется новое значение, которое будет использовано 
        программой в следующей итерации глобального цикла. Сообщаем экземпляру класса геймпада, что переключение с 
        кнопки было учитано, обнуляя флаг switchOperatingMode."""

        # Считываем параметр <operating_mode> из экосистемы ROS2
        config.OperatingMode = self.get_parameter('operating_mode').get_parameter_value().integer_value

        # Проверяем нажатие кнопки переключения режима на геймпаде
        if self.Gamepad.switchOperatingMode:
            # Если кнопка нажималась, то меняем параметр режима на новый
            self.set_parameters([rclpy.parameter.Parameter('operating_mode',
                                                           rclpy.parameter.Parameter.Type.INTEGER,
                                                           self.Gamepad.operatingModeParams)])
            # Опускаем флажок переключения режима на геймпаде
            self.Gamepad.switchOperatingMode = False

        # По параметру режима работы определяем какой таймер активировать, а какие отключить
        if config.OperatingMode == config.DEBUG:
            if self.manualTimer.is_canceled() and self.autoTimer.is_canceled():
                if self.ModeSwitchFlag:
                    self.debugTimer.reset()
                    self.ModeSwitchFlag = False
                    self.Logger.info("Debug mode has been enabled")
            else:
                self.manualTimer.cancel()
                self.autoTimer.cancel()
                self.Logger.info("Manual mode has been disabled")
                self.Logger.info("Auto mode has been disabled")
                self.ModeSwitchFlag = True
        elif config.OperatingMode == config.MANUAL:
            if self.debugTimer.is_canceled() and self.autoTimer.is_canceled():
                if self.ModeSwitchFlag:
                    self.manualTimer.reset()
                    self.ModeSwitchFlag = False
                    self.Logger.info("Manual mode has been enabled")
            else:
                self.debugTimer.cancel()
                self.autoTimer.cancel()
                self.ModeSwitchFlag = True
                self.Logger.info("Debug mode has been disabled")
                self.Logger.info("Auto mode has been disabled")
        elif config.OperatingMode == config.AUTO:
            if self.debugTimer.is_canceled() and self.manualTimer.is_canceled():
                if self.ModeSwitchFlag:
                    self.autoTimer.reset()
                    self.ModeSwitchFlag = False
                    self.Logger.info("Auto mode has been enabled")
            else:
                self.debugTimer.cancel()
                self.manualTimer.cancel()
                self.ModeSwitchFlag = True
                self.Logger.info("Debug mode has been disabled")
                self.Logger.info("Manual mode has been disabled")

        """Считываем параметр <use_camera> из экосистемы ROS2, если на геймпаде произошло нажатие клавиши <Share>, 
        то в параметр экосистемы запишется новое значение, которое будет использовано программой в следующей итерации 
        глобального цикла. Сообщаем экземпляру класса геймпада, что переключение с кнопки было учитано, обнуляя флаг 
        switchCamera. Далее, если камера используется, то активируем таймер камеры."""

        # Считываем параметр <use_camera> из экосистемы ROS2
        config.use_camera = self.get_parameter('use_camera').get_parameter_value().bool_value

        # Проверяем нажатие кнопки переключения камеры на геймпаде
        if self.Gamepad.switchCamera:
            # Если кнопка нажималась, то меняем параметр камеры на новый
            self.set_parameters([rclpy.parameter.Parameter('use_camera',
                                                           rclpy.parameter.Parameter.Type.BOOL,
                                                           self.Gamepad.cameraParams)])
            # Опускаем флажок переключения камеры на геймпаде
            self.Gamepad.switchCamera = False

        # Если по параметру нужно включить камеру, запускаем таймер ROS2, связанный с запуском камеры, если - нет,
        # то останавливаем таймер
        if config.use_camera:
            if self.cameraTimer.is_canceled():
                self.cameraTimer.reset()
        else:
            if self.cameraTimer.is_ready():
                self.cameraTimer.cancel()

    def manual_timer_callback(self):
        if self.Gamepad.is_connected:
            self.SpeedTwist.linear.x = self.Gamepad.linear_velocity
            self.SpeedTwist.angular.z = self.Gamepad.angular_velocity
            self.SpeedTwistPublisher.publish(self.SpeedTwist)
        else:
            self.Logger.error(" Can not start manual control with gamepad! \n"
                              " \tGamepad is disconnected!")
            self.manualTimer.cancel()

    def camera_timer_callback(self):
        if self.camera is None:
            self.Logger.info("Trying to connect camera device...")
            self.camera = realsense.IntelRealSenseCameraD455()
        else:
            self.camera.read_frames()

    def auto_timer_callback(self):
        if config.use_camera:
            if self.camera is None:
                return
            else:
                color_frame = self.camera.get_color_frame()

            if self.controller is None:
                self.controller = logo.LogoFollowerController(color_frame.shape[:2])

            linear_velocity, angular_velocity = self.controller.control(color_frame)

            self.SpeedTwist.linear.x = linear_velocity
            self.SpeedTwist.angular.z = angular_velocity
            self.SpeedTwistPublisher.publish(self.SpeedTwist)
        else:
            self.Logger.error(" The camera option is disabled! Enable the option to continue. \n"
                              " \tYou can turn it on using <ros2 param set /crawler_bot use_camera true>. \n"
                              " \tOr you can press <Share> button on gamepad.")

    def debug_timer_callback(self):

        l_x = self.get_parameter('linear_velocity_x').get_parameter_value().double_value
        l_y = self.get_parameter('linear_velocity_y').get_parameter_value().double_value
        l_z = self.get_parameter('linear_velocity_z').get_parameter_value().double_value
        a_x = self.get_parameter('angular_velocity_x').get_parameter_value().double_value
        a_y = self.get_parameter('angular_velocity_y').get_parameter_value().double_value
        a_z = self.get_parameter('angular_velocity_z').get_parameter_value().double_value

        if self.SpeedTwist.linear.x != l_x:
            self.Logger.info(f"Linear X Speed was change {self.SpeedTwist.linear.x} -> {l_x}")
            self.SpeedTwist.linear.x = l_x
        else:
            pass
        if self.SpeedTwist.linear.y != l_y:
            self.Logger.info(f"Linear Y Speed was change {self.SpeedTwist.linear.y} -> {l_y}")
            self.SpeedTwist.linear.y = l_y
        else:
            pass
        if self.SpeedTwist.linear.z != l_z:
            self.Logger.info(f"Linear Z Speed was change {self.SpeedTwist.linear.z} -> {l_z}")
            self.SpeedTwist.linear.z = l_z
        else:
            pass
        if self.SpeedTwist.angular.x != a_x:
            self.Logger.info(f"Angular X Speed was change {self.SpeedTwist.angular.x} -> {a_x}")
            self.SpeedTwist.angular.x = a_x
        else:
            pass
        if self.SpeedTwist.angular.y != a_y:
            self.Logger.info(f"Angular Y Speed was change {self.SpeedTwist.angular.y} -> {a_y}")
            self.SpeedTwist.angular.y = a_y
        else:
            pass
        if self.SpeedTwist.angular.z != a_z:
            self.Logger.info(f"Angular Z Speed was change {self.SpeedTwist.angular.z} -> {a_z}")
            self.SpeedTwist.angular.z = a_z
        else:
            pass

        self.SpeedTwistPublisher.publish(self.SpeedTwist)


def main(args=None):
    rclpy.init(args=args)
    node = LogoFollowerNode("crawler_bot")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
