import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from elkapod_msgs.msg import LegPositions
from enum import Enum
from copy import deepcopy


class Mode(Enum):
    FREE = 0
    ACCURATE = 1


class ElkapodLegJoyTranslator(Node):
    def __init__(self):
        super().__init__(node_name="elkapod_leg_joy_translator")

        self._joy_subscriber = self.create_subscription(Joy, "/joy", self._position_callback, 10)
        self._leg_publisher = self.create_publisher(LegPositions, "/elkapod_legs_goals", 10)

        self._default_position = [0.3, 0.0, 0.0]
        self._current_position = deepcopy(self._default_position)
        self._mode = Mode.FREE

    def _position_callback(self, msg: Joy):
        step = 0.01

        axes = msg.axes.tolist()
        buttons = msg.buttons.tolist()

        if buttons[1] and self._mode == Mode.FREE:
            self._mode = Mode.ACCURATE
        elif buttons[1] and self._mode == Mode.ACCURATE:
            self._mode = Mode.FREE

        if self._mode == Mode.FREE:
            self._current_position[0] += step * axes[1]
            self._current_position[1] += step * axes[0]
        else:
            self._current_position[0] += step * axes[7]
            self._current_position[1] += step * axes[6]

        if buttons[4]:
            self._current_position[2] += step
        elif buttons[0]:
            self._current_position[2] -= step

        if buttons[3]:
            self._current_position = deepcopy(self._default_position)

        message = LegPositions()
        message.header.frame_id = "root"
        message.header.stamp = self.get_clock().now().to_msg()
        message.leg_positions = [self._list_to_point(self._current_position) for _ in range(6)]

        self._leg_publisher.publish(message)

    @staticmethod
    def _list_to_point(point_data):
        return Point(x=point_data[0], y=point_data[1], z=point_data[2])


def main(args=None):
    rclpy.init(args=args)
    node = ElkapodLegJoyTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
