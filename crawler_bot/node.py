import cv2
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist

from crawler_bot import realsense, gamepad, config, logo


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

        self.cameraTimer = self.create_timer(0.05,
                                             callback=self.camera_timer_callback)

        self.debugTimer.cancel()
        self.manualTimer.cancel()
        self.autoTimer.cancel()
        self.cameraTimer.cancel()

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

        self.controlGamepad = gamepad.Gamepad(interface=config.gamepadInterface, connecting_using_ds4drv=False)

        self.speedTwist = Twist()

    def control_timer_callback(self):
        """
        Функция, контролирующая все переключения в программе.
        :return:
        """
        if self.controlGamepad.rosParameters:
            self.set_parameters(self.controlGamepad.rosParameters)
            self.controlGamepad.rosParameters.clear()

        # Считываем параметр <operating_mode> из экосистемы ROS2
        config.operatingMode = self.get_parameter('operating_mode').get_parameter_value().integer_value

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
                self.commonLogger.info("Manual mode has been disabled")
                self.commonLogger.info("Auto mode has been disabled")
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
                self.commonLogger.info("Debug mode has been disabled")
                self.commonLogger.info("Auto mode has been disabled")
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
                self.commonLogger.info("Debug mode has been disabled")
                self.commonLogger.info("Manual mode has been disabled")

        # Считываем параметр <use_camera> из экосистемы ROS2
        config.usingCamera = self.get_parameter('use_camera').get_parameter_value().bool_value

        if config.usingCamera:
            if self.cameraTimer.is_canceled():
                self.cameraTimer.reset()
        else:
            if self.cameraTimer.is_ready():
                self.cameraTimer.cancel()
                self.commonLogger.info("Turn off camera")

    def manual_timer_callback(self):
        if self.controlGamepad.is_connected:
            self.speedTwist.linear.x = self.controlGamepad.linear_velocity
            self.speedTwist.angular.z = self.controlGamepad.angular_velocity
            self.speedTwistPublisher.publish(self.speedTwist)
        else:
            self.commonLogger.error(" Can not start manual control with gamepad! \n"
                                    " \tGamepad is disconnected!")
            self.manualTimer.cancel()

    def camera_timer_callback(self):
        pass

    def auto_timer_callback(self):
        if config.usingCamera:
            pass

        else:
            self.commonLogger.error(" The camera option is disabled! Enable the option to continue. \n"
                                    " \tYou can turn it on using <ros2 param set /crawler_bot use_camera true>. \n"
                                    " \tOr you can press <Share> button on gamepad.")

    def debug_timer_callback(self):

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


def main(args=None):
    rclpy.init(args=args)
    config.mainNode = LogoFollowerNode("crawler_bot")
    rclpy.spin(config.mainNode)
    config.mainNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
