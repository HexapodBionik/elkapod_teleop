import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from elkapod_msgs.msg import TrajectoryParameters
import math
import numpy as np
from enum import Enum


class Mode(Enum):
    HEIGHT = 0
    BASE_ORIENTATION = 1


class ElkapodJoyController(Node):
    _ROLL_PITCH_STEP = 0.005
    _BASE_HEIGHT_STEP = 0.001
    _WALK_MODES = ["3POINT", "MECHATRONIC"]

    def __init__(self):
        super().__init__(node_name="elkapod_joy_controller")

        self._joy_subscriber = self.create_subscription(Joy, "/joy", self._new_position_callback, 10)
        self._leg_publisher = self.create_publisher(TrajectoryParameters, "/elkapod_trajectory_parameters", 10)

        self._trajectory_parameters = TrajectoryParameters()
        self._trajectory_parameters.leg_spacing = 0.6
        self._trajectory_parameters.height = 0.07
        self._trajectory_parameters.vdir = math.pi / 2
        self._trajectory_parameters.vval = 0.
        self._trajectory_parameters.omega = 0.
        self._trajectory_parameters.yaw = 0.
        self._trajectory_parameters.pitch = 0.
        self._trajectory_parameters.roll = 0.
        self._trajectory_parameters.step_height = 0.1
        self._trajectory_parameters.corpus_position = [0., 0.]
        self._trajectory_parameters.cycle_time = 6.
        self._trajectory_parameters.supportive_legs = [True for _ in range(6)]
        self._trajectory_parameters.gait = "3POINT"

        self._vel_max = 0.02
        self._vel_min = 0.0
        self._vel = 0.025

        self._min_height = 0.075
        self._max_height = 0.15

        self._euler_max = 0.15
        self._euler_min = -0.15

        self._mode = Mode.HEIGHT
        self._clicked = False
        self._walk_mode = 0
        self._walk_mode_string = "3POINT"

        self.get_logger().info("Elkapod Teleop Joy Controller started successfully!")

    def _position_callback(self, msg: Joy):
        step = 0.001

        axes = msg.axes.tolist()
        buttons = msg.buttons.tolist()

        self._trajectory_parameters.vval = self._vel * (1-round(max(0.001, axes[4]), 2))
        self._trajectory_parameters.omega = self._vel * (1 - round(max(0.001, axes[5]), 2))

        if self._trajectory_parameters.vval > self._vel_max:
            self._trajectory_parameters.vval = self._vel_max

        if self._trajectory_parameters.vval < self._vel_min:
            self._trajectory_parameters.vval = self._vel_min

        if self._trajectory_parameters.vval <= 1e-4 and self._trajectory_parameters.omega <= 1e-4:
            self._trajectory_parameters.gait = "STAND"
        else:
            self._trajectory_parameters.gait = "3POINT"

        # Change mode
        if self._mode == Mode.HEIGHT and buttons[15] and not self._clicked:
            self._mode = Mode.BASE_ORIENTATION
            self._clicked = True
        elif self._mode == Mode.BASE_ORIENTATION and buttons[15] and not self._clicked:
            self._mode = Mode.HEIGHT
            self._clicked = True
        else:
            self._clicked = False

        # Height
        if self._mode == Mode.HEIGHT:
            self._trajectory_parameters.height += step * axes[7]
            if self._trajectory_parameters.height >= self._max_height:
                self._trajectory_parameters.height = self._max_height
            elif self._trajectory_parameters.height <= self._min_height:
                self._trajectory_parameters.height = self._min_height

        elif self._mode == Mode.BASE_ORIENTATION:
            self._trajectory_parameters.pitch += step * axes[7]
            self._trajectory_parameters.yaw += step * axes[6]

            if self._trajectory_parameters.pitch >= self._euler_max:
                self._trajectory_parameters.pitch = self._euler_max
            elif self._trajectory_parameters.pitch <= self._euler_min:
                self._trajectory_parameters.pitch = self._euler_min

            if self._trajectory_parameters.yaw >= self._euler_max:
                self._trajectory_parameters.yaw = self._euler_max
            elif self._trajectory_parameters.yaw <= self._euler_min:
                self._trajectory_parameters.yaw = self._euler_min

            if buttons[13]:
                self._trajectory_parameters.pitch = 0.0
                self._trajectory_parameters.yaw = 0.0

        if axes[0] == 0 and axes[1] == 0:
            self._trajectory_parameters.vdir = math.pi/2
        else:
            self._trajectory_parameters.vdir = np.arctan2(axes[1], -axes[0])


        # Roll

        self._leg_publisher.publish(self._trajectory_parameters)

    def _new_position_callback(self, msg: Joy):
        # Get axes and positions
        axes = msg.axes.tolist()
        buttons = msg.buttons.tolist()

        # Set current walk type
        self._trajectory_parameters.gait = self._walk_mode_string

        # Linear velocity value and direction
        v_lin_percentage = np.sqrt(np.power(-axes[0], 2) + np.power(axes[1], 2))
        v_lin_dir = np.arctan2(axes[1], -axes[0])
        self._trajectory_parameters.vval = v_lin_percentage * self._vel_max
        self._trajectory_parameters.vdir = v_lin_dir

        # Angular velocity
        if (1 - round(max(0.001, axes[5]))):
            self._trajectory_parameters.omega = self._vel * (1 - round(max(0.001, axes[5]), 2))
            self._trajectory_parameters.step_height = 0.02
        elif (1 - round(max(0.001, axes[4]))):
            self._trajectory_parameters.omega = -self._vel * (1 - round(max(0.001, axes[4]), 2))
        else:
            self._trajectory_parameters.omega = 0.0
            self._trajectory_parameters.step_height = 0.07

        # Roll and Pitch control
        self._trajectory_parameters.pitch += self._ROLL_PITCH_STEP * axes[3]
        self._trajectory_parameters.yaw -= self._ROLL_PITCH_STEP * axes[2]

        if self._trajectory_parameters.pitch >= self._euler_max:
            self._trajectory_parameters.pitch = self._euler_max
        elif self._trajectory_parameters.pitch <= self._euler_min:
            self._trajectory_parameters.pitch = self._euler_min

        if self._trajectory_parameters.yaw >= self._euler_max:
            self._trajectory_parameters.yaw = self._euler_max
        elif self._trajectory_parameters.yaw <= self._euler_min:
            self._trajectory_parameters.yaw = self._euler_min

        if buttons[14]:
            self._trajectory_parameters.pitch = 0.0
            self._trajectory_parameters.yaw = 0.0
            self.get_logger().info("Roll and pitch of the main body moved to zero positions")

        # Base height
        if buttons[0]:
            self._trajectory_parameters.height -= self._BASE_HEIGHT_STEP
        elif buttons[4]:
            self._trajectory_parameters.height += self._BASE_HEIGHT_STEP

        if self._trajectory_parameters.height >= self._max_height:
            self._trajectory_parameters.height = self._max_height
        elif self._trajectory_parameters.height <= self._min_height:
            self._trajectory_parameters.height = self._min_height

        # When pad is IDLE
        if self._trajectory_parameters.vval <= 1e-4 and abs(self._trajectory_parameters.omega) <= 1e-4:
            self._trajectory_parameters.gait = "STAND"
        else:
            self._trajectory_parameters.gait = self._walk_mode_string

        # Change Walk mode
        if buttons[3] and (
                self._walk_mode == 0 or self._walk_mode == 2) and self._trajectory_parameters.gait == "STAND":
            self._trajectory_parameters.gait = "3POINT"
            self._walk_mode_string = "3POINT"
            self._walk_mode = 1
            self.get_logger().info("Changed walk mode to 3POINT")
        elif buttons[3] and (
                self._walk_mode == 0 or self._walk_mode == 1) and self._trajectory_parameters.gait == "STAND":
            self._trajectory_parameters.gait = "MECHATRONIC"
            self._walk_mode_string = "MECHATRONIC"
            self._walk_mode = 2
            self.get_logger().info("Changed walk mode to MECHATRONIC")

        self._leg_publisher.publish(self._trajectory_parameters)


def main(args=None):
    rclpy.init(args=args)
    node = ElkapodJoyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
