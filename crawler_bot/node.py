import time

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from crawler_bot import realsense, gamepad, config, logo, paint


class LogoFollowerNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.commonLogger = config.commonLogger

        self.modeSwitchFlag = True

        self.debugTimer = self.create_timer(timer_period_sec=1.,
                                            callback=self.debug_timer_callback)

        self.manualTimer = self.create_timer(timer_period_sec=0.05,
                                             callback=self.manual_timer_callback)

        self.autoTimer = self.create_timer(1.,
                                           callback=self.auto_timer_callback)
        self.cameraTimer = self.create_timer(0.033,
                                             callback=self.camera_timer_callback)

        self.debugTimer.cancel()
        self.manualTimer.cancel()
        self.autoTimer.cancel()

        self.controlTimer = self.create_timer(1.,
                                              callback=self.control_timer_callback)

        self.declare_parameter('linear_velocity_x', 0.0)
        self.declare_parameter('angular_velocity_z', 0.0)
        self.declare_parameter('operating_mode', config.operatingMode)
        self.declare_parameter('use_camera', config.usingCamera)

        self.speedTwistPublisher = self.create_publisher(msg_type=Twist,
                                                         topic="/crawler_bot/twist",
                                                         qos_profile=10,
                                                         )
        self.cvBridge = CvBridge()
        self.colorImage = None
        self.depthImage = None
        self.colorImageSubscription = self.create_subscription(msg_type=Image,
                                                               topic="/camera/color/image_raw",
                                                               callback=self.camera_color_callback,
                                                               qos_profile=10,
                                                               )
        self.depthImageSubscription = self.create_subscription(msg_type=Image,
                                                               topic="/camera/depth/image_rect_raw",
                                                               callback=self.camera_depth_callback,
                                                               qos_profile=10,
                                                               )

        self.controlGamepad = gamepad.Gamepad(interface=config.gamepadInterface, connecting_using_ds4drv=False)
        self.logoController = logo.LogoFollowerController((config.colorImageWidth, config.colorImageHeight))
        self.imagePainter = paint.Painter()

        self.speedTwist = Twist()

    def control_timer_callback(self):
        """
        Callback функция основного таймера, контролирующая работу всех остальных таймеров в программе. Переключение
        режимов работы робота, активация камеры. :return:
        """
        # Считываем параметр <operating_mode> из экосистемы ROS2
        config.operatingMode = self.get_parameter('operating_mode').get_parameter_value().integer_value
        # Считываем параметр <use_camera> из экосистемы ROS2
        config.usingCamera = self.get_parameter('use_camera').get_parameter_value().bool_value
        # По параметру режима работы определяем какой таймер активировать, а какие отключить
        if config.operatingMode == config.DEBUG:
            if self.manualTimer.is_canceled() and self.autoTimer.is_canceled():
                if self.modeSwitchFlag:
                    self.debugTimer.reset()
                    self.modeSwitchFlag = False
                    self.commonLogger.info("Debug mode has been enabled")
            else:
                self.manualTimer.cancel()
                self.autoTimer.cancel()
                self.modeSwitchFlag = True
                # self.commonLogger.info("Manual mode has been disabled")
                # self.commonLogger.info("Auto mode has been disabled")
        elif config.operatingMode == config.MANUAL:
            if self.debugTimer.is_canceled() and self.autoTimer.is_canceled():
                if self.modeSwitchFlag:
                    self.manualTimer.reset()
                    self.modeSwitchFlag = False
                    self.commonLogger.info("Manual mode has been enabled")
            else:
                self.debugTimer.cancel()
                self.autoTimer.cancel()
                self.modeSwitchFlag = True
                # self.commonLogger.info("Debug mode has been disabled")
                # self.commonLogger.info("Auto mode has been disabled")
        elif config.operatingMode == config.AUTO:
            if self.debugTimer.is_canceled() and self.manualTimer.is_canceled():
                if self.modeSwitchFlag:
                    self.autoTimer.reset()
                    self.modeSwitchFlag = False
                    self.commonLogger.info("Auto mode has been enabled")
            else:
                self.debugTimer.cancel()
                self.manualTimer.cancel()
                self.modeSwitchFlag = True
                # self.commonLogger.info("Debug mode has been disabled")
                # self.commonLogger.info("Manual mode has been disabled")

    def debug_timer_callback(self):
        """
        Callback функция таймера управления роботом в режиме отладки. Получает скорости робота из параметров среды
        ROS2 и отправляет сообщения в среду ROS2. :return:
        """
        l_x = self.get_parameter('linear_velocity_x').get_parameter_value().double_value
        a_z = self.get_parameter('angular_velocity_z').get_parameter_value().double_value
        if self.speedTwist.linear.x != l_x:
            self.commonLogger.info(f"Linear X Speed was change {self.speedTwist.linear.x} -> {l_x}")
            self.speedTwist.linear.x = l_x
        else:
            pass
        if self.speedTwist.angular.z != a_z:
            self.commonLogger.info(f"Angular Z Speed was change {self.speedTwist.angular.z} -> {a_z}")
            self.speedTwist.angular.z = a_z
        else:
            pass
        self.speedTwistPublisher.publish(self.speedTwist)

    def manual_timer_callback(self):
        """
        Callback функция таймера ручного управления роботом с геймпада. Отправляет параметры и сообщения,
        управляемые с геймпада, в среду ROS2. Если геймпад не подключен, создается экземпляр класса геймпада,
        начинается подключение к нему, таймер, к которому пренадлежит эта функция ставится на паузу,
        пока не подключится геймпад :return:
        """
        if self.controlGamepad.is_connected:
            if self.controlGamepad.rosParameters:
                self.set_parameters(self.controlGamepad.rosParameters)
                self.controlGamepad.rosParameters.clear()
            self.speedTwist.linear.x = self.controlGamepad.linear_velocity
            self.speedTwist.angular.z = self.controlGamepad.angular_velocity
            self.speedTwistPublisher.publish(self.speedTwist)
        else:
            self.commonLogger.error(" Can not start manual control with gamepad! Gamepad is disconnected!")
            self.controlGamepad = gamepad.Gamepad(interface=config.gamepadInterface, connecting_using_ds4drv=False)
            self.manualTimer.cancel()

    def auto_timer_callback(self):
        """
        Callback функция таймера автономного управления роботом. Управляет детектором логотипа на изображении с
        камеры робота и отправляет сообщения в среду ROS2. Если нет изображения, то таймер ставится на паузу,
        активируется таймер режима отладки. :return:
        """
        if config.usingCamera:
            try:
                self.logoController.calculate_velocity_delta(self.colorImage)
                self.commonLogger.info(f" Twist velocities: \n "
                                       f" \t x: {self.logoController.linearDelta} "
                                       f" alpha: {self.logoController.angularDelta}")
            except Exception:
                self.commonLogger.error(" Camera frame is empty! \n"
                                        " \tCheck realsense node is up.")
                self.set_parameters([Parameter('operating_mode', Parameter.Type.INTEGER, config.DEBUG)])
        else:
            self.commonLogger.error(" The camera option is disabled! \n"
                                    " \tEnable the option and restart operating mode to continue.")
            self.set_parameters([Parameter('operating_mode', Parameter.Type.INTEGER, config.DEBUG)])

    def camera_timer_callback(self):
        if config.usingCamera:
            targeted_image = self.imagePainter.draw_logo_target(self.colorImage,
                                                                self.logoController.logoFollower.followerLogo)
            cv2.imshow("Image", targeted_image)
            cv2.waitKey(1)

    def camera_color_callback(self, msg):
        self.colorImage = self.cvBridge.imgmsg_to_cv2(msg)

    def camera_depth_callback(self, msg):
        self.depthImage = self.cvBridge.imgmsg_to_cv2(msg)


def main(args=None):
    rclpy.init(args=args)
    config.mainNode = LogoFollowerNode("crawler_bot")
    rclpy.spin(config.mainNode)
    config.mainNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
