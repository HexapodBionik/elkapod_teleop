from enum import Enum
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import Joy
import math
import numpy as np

class GaitType(Enum):
    WAVE = 0
    RIPPLE = 1
    TRIPOD = 2

class SpeedCommand:
    def __init__(self):
        self.vx = 0
        self.vy = 0
        self.omega = 0

    def norm(self):
        return math.sqrt(self.vx**2 + self.vy**2)

class ElkapodControllerNode(Node):
    def __init__(self):
        super().__init__("ElkapodControllerGui")

        self._joy_subscriber = self.create_subscription(Joy, "/joy", self._joystick_callback, 10)
        self._command_timer = self.create_timer(0.05, self._send_commands_callback)

        self._cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._cmd_gait_type_publisher = self.create_publisher(Int32, "/cmd_gait_type", 10)
        self._cmd_base_height_publisher = self.create_publisher(Float64, "/cmd_base_height", 10)
        self._cmd_pitch_publisher = self.create_publisher(Float64, "/pitch_setpoint", 10)
        self._cmd_roll_publisher = self.create_publisher(Float64, "/roll_setpoint", 10)

        self._base_height = 0.12
        self._speed = SpeedCommand()
        self._roll = 0.0
        self._pitch = 0.0
        self._gait_type = GaitType.TRIPOD

        self._max_angular_speed = 0.5
        self._max_linear_speed = 0.1


    def _send_commands_callback(self):
        pass

    def send_gait_type_command(self, gait_type: GaitType):
        msg = Int32()
        match gait_type:
            case GaitType.WAVE:
                msg.data = 0
            case GaitType.RIPPLE:
                msg.data = 1
            case GaitType.TRIPOD:
                msg.data = 2
            case _:
                msg.data = 2
        self._cmd_gait_type_publisher.publish(msg)

    def send_base_height_command(self, base_height: float):
        msg = Float64()
        msg.data = base_height
        self._cmd_base_height_publisher.publish(msg)

    def send_vel_command(self, speed_command: SpeedCommand):
        msg = Twist()
        msg.linear.x = float(speed_command.vx)
        msg.linear.y = float(speed_command.vy)
        msg.angular.z = float(speed_command.omega)

        self._cmd_vel_publisher.publish(msg)

    def send_pitch_command(self, pitch: float):
        msg = Float64()
        msg.data = pitch
        self._cmd_pitch_publisher.publish(msg)

    def send_roll_command(self, roll: float):
        msg = Float64()
        msg.data = roll
        self._cmd_roll_publisher.publish(msg)

    def _joystick_callback(self, msg: Joy):
        # Get axes and positions
        axes = msg.axes.tolist()
        buttons = msg.buttons.tolist()

        pitch_axis = axes[3]
        roll_axis = axes[2]
        pitch_roll_reset_btn = buttons[14]
        base_height_increment_btn = buttons[4]
        base_height_decrement_btn = buttons[0]
        change_walk_mode_btn = buttons[3]

        # Linear velocity value and direction
        vval = np.sqrt(np.power(-axes[0], 2) + np.power(axes[1], 2))
        vdir = np.arctan2(axes[0], axes[1])

        self._speed.vx = math.cos(vdir) * vval
        self._speed.vy = math.sin(vdir) * vval


        angular_clockwise_trigger = (1 - round(max(0.001, axes[5]), 2))
        angular_anticlockwise_trigger = (1 - round(max(0.001, axes[4]), 2))

        if angular_clockwise_trigger > 0.0:
            self._speed.omega = angular_clockwise_trigger * self._max_angular_speed
            self._speed.vx = 0.0
            self._speed.vy = 0.0
        elif angular_anticlockwise_trigger > 0.0:
            self._speed.omega = - angular_anticlockwise_trigger * self._max_angular_speed
            self._speed.vx = 0.0
            self._speed.vy = 0.0

        self.send_vel_command(self._speed)


        # if buttons[14]:
        #     self._trajectory_parameters.pitch = 0.0
        #     self._trajectory_parameters.yaw = 0.0
        #     self.get_logger().info("Roll and pitch of the main body moved to zero positions")

        # Base height
        # if buttons[0]:
        #     self._trajectory_parameters.height -= self._BASE_HEIGHT_STEP
        # elif buttons[4]:
        #     self._trajectory_parameters.height += self._BASE_HEIGHT_STEP

        # Change Walk mode
        # if buttons[3] and (
        #         self._walk_mode == 0 or self._walk_mode == 2) and self._trajectory_parameters.gait == "STAND":
        #     self._trajectory_parameters.gait = "3POINT"
        #     self._walk_mode_string = "3POINT"
        #     self._walk_mode = 1
        #     self.get_logger().info("Changed walk mode to 3POINT")
        # elif buttons[3] and (
        #         self._walk_mode == 0 or self._walk_mode == 1) and self._trajectory_parameters.gait == "STAND":
        #     self._trajectory_parameters.gait = "MECHATRONIC"
        #     self._walk_mode_string = "MECHATRONIC"
        #     self._walk_mode = 2
        #     self.get_logger().info("Changed walk mode to MECHATRONIC")


def main(args=None):
    rclpy.init(args=args)
    node = ElkapodControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()