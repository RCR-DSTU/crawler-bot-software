import time
import rclpy

from threading import Thread

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from rclpy.parameter import Parameter
from pyPS4Controller.controller import Controller


class Gamepad(Controller):
    def __init__(self, timeout: int = 1, **kwargs):
        Controller.__init__(self, **kwargs)
        self.timeOut = timeout
        time.sleep(2)

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
        if self.linear_velocity < 0:
            self.linear_velocity = 0.

    def on_R1_press(self):
        pass

    def on_R1_release(self):
        pass

    def on_R2_press(self, value):
        self.linear_velocity = 0.5 - (- value) * self.ratio * 0.5

    def on_R2_release(self):
        if self.linear_velocity > 0:
            self.linear_velocity = 0.

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
        pass

    def on_R3_x_at_rest(self):
        pass

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
        pass

    def on_share_release(self):
        pass

    def on_playstation_button_press(self):
        pass

    def on_playstation_button_release(self):
        pass

    def start_listening(self):
        self.listen(timeout=self.timeOut)


class ControllerNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('frequency', 25)

        self.timerPeriod = 1 / self.get_parameter('frequency').get_parameter_value().integer_value

        self.rawTwistPublisher = self.create_publisher(Twist,
                                                       '/controller/raw_manual_twist',
                                                       10)
        self.mainTimer = self.create_timer(self.timerPeriod, self.main_timer_callback)
        self.rawTwistMsg = Twist()

        self.connectionService = self.create_service(SetBool, 'controller_connection_service',
                                                     self.check_connection)

        self.gamepad = Gamepad(timeout=100000000,
                               interface="/dev/input/js0",
                               connecting_using_ds4drv=False)

    def main_timer_callback(self):
        if self.gamepad.is_connected:
            self.rawTwistMsg.linear.x = self.gamepad.linear_velocity
            self.rawTwistMsg.angular.z = self.gamepad.angular_velocity
            self.rawTwistPublisher.publish(self.rawTwistMsg)
        else:
            pass

    def check_connection(self, request, response):
        response.success = self.gamepad.is_connected
        return response


def main():
    rclpy.init()
    node = ControllerNode('controller')
    rclpy.spin(node)
    node.gamepad.threadListener.join()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
