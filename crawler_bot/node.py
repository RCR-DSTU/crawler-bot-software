import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist

from folder.crawler_bot import realsense, gamepad
from folder.crawler_bot import config, logo


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

        self.debugTimer.cancel()
        self.manualTimer.cancel()
        self.autoTimer.cancel()

        self.controlTimer = self.create_timer(2.,
                                              callback=self.control_timer_callback)

        self.declare_parameter('linear_x_velocity', 0.0)
        self.declare_parameter('linear_y_velocity', 0.0)
        self.declare_parameter('linear_z_velocity', 0.0)
        self.declare_parameter('angular_x_velocity', 0.0)
        self.declare_parameter('angular_y_velocity', 0.0)
        self.declare_parameter('angular_z_velocity', 0.0)

        self.declare_parameter('operating_mode', config.OperatingMode)

        self.SpeedTwistPublisher = self.create_publisher(msg_type=Twist,
                                                         topic="/logo_follower/twist",
                                                         qos_profile=10,
                                                         )

        self.Gamepad = gamepad.Gamepad(interface=config.gamepad_interface, connecting_using_ds4drv=False)
        self.camera = None
        self.controller = None

        self.SpeedTwist = Twist()

    def control_timer_callback(self):

        config.OperatingMode = self.get_parameter('operating_mode').get_parameter_value().integer_value

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

    def manual_timer_callback(self):
        self.SpeedTwist.linear.x = self.Gamepad.linear_velocity
        self.SpeedTwist.angular.z = self.Gamepad.angular_velocity
        self.SpeedTwistPublisher.publish(self.SpeedTwist)

    def auto_timer_callback(self):
        if self.camera is None:
            self.camera = realsense.IntelRealSenseCameraD455()

        self.camera.read_frames()
        color_frame = self.camera.get_color_frame()

        if self.controller is None:
            self.controller = logo.LogoFollowerController(color_frame.shape[:2])

        linear_velocity, angular_velocity = self.controller.control(color_frame)

        self.SpeedTwist.linear.x = linear_velocity
        self.SpeedTwist.angular.z = angular_velocity
        self.SpeedTwistPublisher.publish(self.SpeedTwist)

    def debug_timer_callback(self):

        l_x = self.get_parameter('linear_x_velocity').get_parameter_value().double_value
        l_y = self.get_parameter('linear_y_velocity').get_parameter_value().double_value
        l_z = self.get_parameter('linear_z_velocity').get_parameter_value().double_value
        a_x = self.get_parameter('angular_x_velocity').get_parameter_value().double_value
        a_y = self.get_parameter('angular_y_velocity').get_parameter_value().double_value
        a_z = self.get_parameter('angular_z_velocity').get_parameter_value().double_value

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
    node = LogoFollowerNode("logo_follower_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
