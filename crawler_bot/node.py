import cv2
import rclpy
import time

from cv_bridge import CvBridge
from rclpy import Parameter
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import CompressedImage

from crawler_bot import gamepad, config, paint


class LogoFollowerNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.commonLogger = self.get_logger()
        self.modeSwitchFlag = True
        self.colorImageSubscription = self.create_subscription(msg_type=CompressedImage,
                                                               topic="/color_image",
                                                               callback=self.camera_color_callback,
                                                               qos_profile=10,
                                                               )
        self.depthImageSubscription = self.create_subscription(msg_type=CompressedImage,
                                                               topic="/depth_image",
                                                               callback=self.camera_depth_callback,
                                                               qos_profile=10,
                                                               )
        self.targetPointSubscription = self.create_subscription(msg_type=Point,
                                                                topic='/color_image/target',
                                                                callback=self.camera_target_callback,
                                                                qos_profile=10)
        self.targetSpeedSubscription = self.create_subscription(msg_type=Twist,
                                                                topic='/color_image/velocity',
                                                                callback=self.camera_velocity_callback,
                                                                qos_profile=10)
        self.debugTimer = self.create_timer(timer_period_sec=config.TIMER_PERIOD,
                                            callback=self.debug_timer_callback)
        self.manualTimer = self.create_timer(timer_period_sec=config.TIMER_PERIOD,
                                             callback=self.manual_timer_callback)
        self.autoTimer = self.create_timer(timer_period_sec=config.TIMER_PERIOD,
                                           callback=self.auto_timer_callback)
        self.cameraTimer = self.create_timer(timer_period_sec=config.TIMER_PERIOD,
                                             callback=self.camera_timer_callback)
        self.controlTimer = self.create_timer(timer_period_sec=config.TIMER_PERIOD,
                                              callback=self.control_timer_callback)
        self.debugTimer.cancel()
        self.manualTimer.cancel()
        self.autoTimer.cancel()
        self.declare_parameter('linear_velocity', 0.0)
        self.declare_parameter('angular_velocity', 0.0)
        self.declare_parameter('operating_mode', config.operatingMode)
        self.declare_parameter('use_camera', config.usingCamera)
        self.speedTwistPublisher = self.create_publisher(msg_type=Twist,
                                                         topic="/CrawlerBot/twist",
                                                         qos_profile=10,
                                                         )
        self.cvBridge = CvBridge()
        self.cameraTarget = Point()
        self.colorImage = None
        self.depthImage = None
        self.cameraVelocity = Twist()
        self.controlGamepad = gamepad.Gamepad(interface=config.gamepadInterface, connecting_using_ds4drv=False)
        self.imagePainter = paint.Painter()
        self.speedTwist = Twist()

        self.checkAutoModeTimer = time.time()

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

    def debug_timer_callback(self):
        """
        Callback функция таймера управления роботом в режиме отладки. Получает скорости робота из параметров среды
        ROS2 и отправляет сообщения в среду ROS2. :return:
        """
        l_x = self.get_parameter('linear_velocity').get_parameter_value().double_value
        a_z = self.get_parameter('angular_velocity').get_parameter_value().double_value
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
            self.controlGamepad = gamepad.Gamepad(timeout=5000000,
                                                  interface=config.gamepadInterface,
                                                  connecting_using_ds4drv=False)
            self.manualTimer.cancel()

    def auto_timer_callback(self):
        """
        Callback функция таймера автономного управления роботом. Управляет детектором логотипа на изображении с
        камеры робота и отправляет сообщения в среду ROS2. Если нет изображения, то таймер ставится на паузу,
        активируется таймер режима отладки. :return:
        """
        if self.checkAutoModeTimer + config.AUTO_MODE_TIMEOUT < time.time():
            self.commonLogger.info("Realsense node messages did not arrive with in 5 seconds \n"
                                   "    Turning Auto Mode off.")
            self.autoTimer.cancel()
            self.set_parameters([Parameter('operating_mode', Parameter.Type.INTEGER, 0)])
            self.modeSwitchFlag = True
        self.cameraVelocity.linear.x = - self.cameraVelocity.linear.x
        self.cameraVelocity.angular.z = - self.cameraVelocity.angular.z
        self.speedTwistPublisher.publish(self.cameraVelocity)

    def camera_timer_callback(self):
        """
        Callback функция таймера активации окна камеры робота. Если режим автономный, то на изображении дорисовывется
        маркер цели. :return:
        """
        if config.usingCamera:
            if config.operatingMode == config.AUTO:
                result_image = self.imagePainter.draw_logo_target(self.colorImage, self.cameraTarget)
            else:
                result_image = self.colorImage
            try:
                cv2.imshow("Image", cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB))
            except:
                pass
            cv2.waitKey(1)

    def camera_color_callback(self, msg):
        self.colorImage = self.cvBridge.compressed_imgmsg_to_cv2(msg)

    def camera_depth_callback(self, msg):
        self.depthImage = self.cvBridge.compressed_imgmsg_to_cv2(msg)

    def camera_target_callback(self, msg):
        self.cameraTarget = msg

    def camera_velocity_callback(self, msg):
        self.checkAutoModeTimer = time.time()
        self.cameraVelocity = msg


def main(args=None):
    rclpy.init(args=args)
    config.mainNode = LogoFollowerNode("crawler_bot")
    rclpy.spin(config.mainNode)
    config.mainNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
