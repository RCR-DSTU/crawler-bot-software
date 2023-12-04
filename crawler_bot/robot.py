import rclpy

from rclpy import Parameter
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('operating_mode', "debug")
        self.declare_parameter('auto_mode_timeout', 5)
        self.declare_parameter('manual_mode_timeout', 5)
        self.declare_parameter('frequency', 25)
        self.declare_parameter('debug_linear_velocity', 0.1)
        self.declare_parameter('debug_angular_velocity', 0.1)

        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('min_linear_velocity', 0.0)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('min_angular_velocity', 0.0)

        self.operatingMode = self.get_parameter('operating_mode').get_parameter_value().string_value
        self.autoModeTimeout = self.get_parameter('auto_mode_timeout').get_parameter_value().integer_value
        self.manualModeTimeout = self.get_parameter('manual_mode_timeout').get_parameter_value().integer_value
        self.timerPeriod = 1 / self.get_parameter('frequency').get_parameter_value().integer_value

        self.maxLinearVelocity = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.maxAngularVelocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.minLinearVelocity = self.get_parameter('min_linear_velocity').get_parameter_value().double_value
        self.minAngularVelocity = self.get_parameter('min_angular_velocity').get_parameter_value().double_value

        self.commonLogger = self.get_logger()
        self.modeSwitchFlag = True

        self.autoTwistSubscription = self.create_subscription(msg_type=Twist,
                                                              topic="/detector/raw_auto_twist",
                                                              callback=self.auto_twist_callback,
                                                              qos_profile=10,
                                                              )
        self.manualTwistSubscription = self.create_subscription(msg_type=Twist,
                                                                topic="/controller/raw_manual_twist",
                                                                callback=self.manual_twist_callback,
                                                                qos_profile=10,
                                                                )

        self.debugTimer = self.create_timer(timer_period_sec=self.timerPeriod,
                                            callback=self.debug_timer_callback)
        self.manualTimer = self.create_timer(timer_period_sec=self.timerPeriod,
                                             callback=self.manual_timer_callback)
        self.autoTimer = self.create_timer(timer_period_sec=self.timerPeriod,
                                           callback=self.auto_timer_callback)
        self.controlTimer = self.create_timer(timer_period_sec=self.timerPeriod,
                                              callback=self.control_timer_callback)

        self.debugTimer.cancel()
        self.manualTimer.cancel()
        self.autoTimer.cancel()

        self.robotTwistPublisher = self.create_publisher(msg_type=Twist,
                                                         topic="/CrawlerBot/twist",
                                                         qos_profile=10,
                                                         )
        self.robotTwist = Twist()
        self.manualTwist = Twist()
        self.autoTwist = Twist()

        self.checkManualModeTimer = self.get_clock().now().to_msg().sec
        self.checkAutoModeTimer = self.get_clock().now().to_msg().sec

    def control_timer_callback(self):
        """
        Callback функция основного таймера, контролирующая работу всех остальных таймеров в программе. Переключение
        режимов работы робота, активация камеры. :return:
        """
        # Считываем параметр <operating_mode> из экосистемы ROS2
        self.operatingMode = self.get_parameter('operating_mode').get_parameter_value().string_value

        # По параметру режима работы определяем какой таймер активировать, а какие отключить
        if self.operatingMode == 'debug':
            if self.manualTimer.is_canceled() and self.autoTimer.is_canceled():
                if self.modeSwitchFlag:
                    self.debugTimer.reset()
                    self.modeSwitchFlag = False
                    self.commonLogger.info("Debug mode has been enabled")
            else:
                self.manualTimer.cancel()
                self.autoTimer.cancel()
                self.modeSwitchFlag = True
        elif self.operatingMode == 'manual':
            if self.debugTimer.is_canceled() and self.autoTimer.is_canceled():
                if self.modeSwitchFlag:
                    self.manualTimer.reset()
                    self.modeSwitchFlag = False
                    self.commonLogger.info("Manual mode has been enabled")
                    self.checkManualModeTimer = self.get_clock().now().to_msg().sec
            else:
                self.debugTimer.cancel()
                self.autoTimer.cancel()
                self.modeSwitchFlag = True
        elif self.operatingMode == 'auto':
            if self.debugTimer.is_canceled() and self.manualTimer.is_canceled():
                if self.modeSwitchFlag:
                    self.autoTimer.reset()
                    self.modeSwitchFlag = False
                    self.commonLogger.info("Auto mode has been enabled")
                    self.checkAutoModeTimer = self.get_clock().now().to_msg().sec
            else:
                self.debugTimer.cancel()
                self.manualTimer.cancel()
                self.modeSwitchFlag = True

    def debug_timer_callback(self):
        """
        Callback функция таймера управления роботом в режиме отладки. Получает скорости робота из параметров среды
        ROS2 и отправляет сообщения в среду ROS2. :return:
        """
        l_x = self.get_parameter('debug_linear_velocity').get_parameter_value().double_value
        a_z = self.get_parameter('debug_angular_velocity').get_parameter_value().double_value
        if self.robotTwist.linear.x != l_x:
            self.commonLogger.info(f"Linear X Speed was change {self.robotTwist.linear.x} -> {l_x}")
            self.robotTwist.linear.x = l_x
        else:
            pass
        if self.robotTwist.angular.z != a_z:
            self.commonLogger.info(f"Angular Z Speed was change {self.robotTwist.angular.z} -> {a_z}")
            self.robotTwist.angular.z = a_z
        else:
            pass
        self.robotTwistPublisher.publish(self.robotTwist)

    def manual_timer_callback(self):
        """
        Callback функция таймера ручного управления роботом с геймпада. Отправляет параметры и сообщения,
        управляемые с геймпада, в среду ROS2. Если геймпад не подключен, создается экземпляр класса геймпада,
        начинается подключение к нему, таймер, к которому пренадлежит эта функция ставится на паузу,
        пока не подключится геймпад :return:
        """
        if self.checkManualModeTimer + self.manualModeTimeout < self.get_clock().now().to_msg().sec:
            self.commonLogger.error(f"Controller node messages did not arrive with in {self.manualModeTimeout}"
                                    f" seconds \n"
                                    "    Turning Manual Mode off.")
            self.manualTimer.cancel()
            self.set_parameters([Parameter('operating_mode', Parameter.Type.STRING, 'debug')])
            self.modeSwitchFlag = True
        else:
            self.robotTwist.linear.x = self.manualTwist.linear.x
            self.robotTwist.angular.z = self.manualTwist.angular.z
            self.robotTwistPublisher.publish(self.robotTwist)

    def auto_timer_callback(self):
        """
        Callback функция таймера автономного управления роботом. Управляет детектором логотипа на изображении с
        камеры робота и отправляет сообщения в среду ROS2. Если нет изображения, то таймер ставится на паузу,
        активируется таймер режима отладки. :return:
        """
        if self.checkAutoModeTimer + self.autoModeTimeout < self.get_clock().now().to_msg().sec:
            self.commonLogger.error(f"Detector node messages did not arrive with in {self.autoModeTimeout}"
                                    f" seconds \n"
                                    "    Turning Auto Mode off.")
            self.autoTimer.cancel()
            self.set_parameters([Parameter('operating_mode', Parameter.Type.STRING, 'debug')])
            self.modeSwitchFlag = True
        else:
            self.robotTwist.linear.x = - self.autoTwist.linear.x
            self.robotTwist.angular.z = - self.autoTwist.angular.z
            self.robotTwistPublisher.publish(self.robotTwist)

    def auto_twist_callback(self, msg):
        self.autoTwist = msg
        self.checkAutoModeTimer = self.get_clock().now().to_msg().sec

    def manual_twist_callback(self, msg):
        self.manualTwist = msg
        self.checkManualModeTimer = self.get_clock().now().to_msg().sec

    # def control(self):
    #     if self.linearDelta > 0:
    #         if self.linearDelta > self.maxLinearVelocity:
    #             self.linearDelta = self.maxLinearVelocity
    #         elif self.linearDelta < self.minLinearVelocity:
    #             self.linearDelta = self.minLinearVelocity
    #     else:
    #         if self.linearDelta < -self.maxLinearVelocity:
    #             self.linearDelta = -self.maxLinearVelocity
    #         elif self.linearDelta > -self.minLinearVelocity:
    #             self.linearDelta = -self.minLinearVelocity
    #
    #     if self.angularDelta > 0:
    #         if self.angularDelta > self.maxAngularVelocity:
    #             self.angularDelta = self.maxAngularVelocity
    #         elif self.angularDelta < self.minAngularVelocity:
    #             self.angularDelta = self.minAngularVelocity
    #     else:
    #         if self.angularDelta < -self.maxAngularVelocity:
    #             self.angularDelta = -self.maxAngularVelocity
    #         elif self.angularDelta > -self.minAngularVelocity:
    #             self.angularDelta = -self.minAngularVelocity


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode("robot")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
