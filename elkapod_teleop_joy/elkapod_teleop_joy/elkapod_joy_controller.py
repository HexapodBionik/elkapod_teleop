from enum import Enum
import rclpy
from rclpy.node import Node, ParameterDescriptor
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

class ElkapodJoyNode(Node):
    _ROLL_PITCH_STEP = 0.005
    _BASE_HEIGHT_STEP = 0.001

    def __init__(self):
        super().__init__("elkapod_joy")
        self.declare_parameter("base_height.min", None, ParameterDescriptor(description="Min base height", dynamic_typing=True))
        self.declare_parameter("base_height.max", None, ParameterDescriptor(description="Max base height", dynamic_typing=True))
        self.declare_parameter("base_height.default",None, ParameterDescriptor(description="Default base height", dynamic_typing=True))
        self.declare_parameter("roll.max_rad", None, ParameterDescriptor(description="Max roll value in rad", dynamic_typing=True))
        self.declare_parameter("pitch.max_rad", None, ParameterDescriptor(description="Max pitch value in rad", dynamic_typing=True))
        self.declare_parameter("max_vel.tripod", None, ParameterDescriptor(description="Max linear velocity for tripod gait", dynamic_typing=True))
        self.declare_parameter("max_vel.wave", None, ParameterDescriptor(description="Max linear velocity for wave gait", dynamic_typing=True))
        self.declare_parameter("max_vel.ripple", None, ParameterDescriptor(description="Max linear velocity for ripple gait", dynamic_typing=True))
        self.declare_parameter("max_angular_vel.tripod", None, ParameterDescriptor(description="Max angular velocity for tripod gait", dynamic_typing=True))
        self.declare_parameter("max_angular_vel.wave", None, ParameterDescriptor(description="Max angular velocity for wave gait", dynamic_typing=True))
        self.declare_parameter("max_angular_vel.ripple", None, ParameterDescriptor(description="Max angular velocity for ripple gait", dynamic_typing=True))
        self.declare_parameter("gamepad_model", "xbox-series-x", ParameterDescriptor(description="Model of gamepad use for control", dynamic_typing=True))

        self._joy_subscriber = self.create_subscription(Joy, "/joy", self._joystick_callback, 10)
        self._command_timer = self.create_timer(0.05, self._send_commands_callback)

        self._cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._cmd_gait_type_publisher = self.create_publisher(Int32, "/cmd_gait_type", 10)
        self._cmd_base_height_publisher = self.create_publisher(Float64, "/cmd_base_height", 10)
        self._cmd_pitch_publisher = self.create_publisher(Float64, "/pitch_setpoint", 10)
        self._cmd_roll_publisher = self.create_publisher(Float64, "/roll_setpoint", 10)

        self._base_height_min = self.get_parameter("base_height.min").get_parameter_value().double_value
        self._base_height_max = self.get_parameter("base_height.max").get_parameter_value().double_value
        self._base_height_default = self.get_parameter("base_height.default").get_parameter_value().double_value
        self._roll_max_rad = self.get_parameter("roll.max_rad").get_parameter_value().double_value
        self._pitch_max_rad = self.get_parameter("pitch.max_rad").get_parameter_value().double_value
        self._max_vel_tripod = self.get_parameter("max_vel.tripod").get_parameter_value().double_value
        self._max_vel_wave = self.get_parameter("max_vel.wave").get_parameter_value().double_value
        self._max_vel_ripple = self.get_parameter("max_vel.ripple").get_parameter_value().double_value
        self._max_angular_vel_tripod = self.get_parameter("max_angular_vel.tripod").get_parameter_value().double_value
        self._max_angular_vel_wave = self.get_parameter("max_angular_vel.wave").get_parameter_value().double_value
        self._max_angular_vel_ripple = self.get_parameter("max_angular_vel.ripple").get_parameter_value().double_value
        self._gamepad_model = self.get_parameter("gamepad_model").get_parameter_value().string_value

        self._supported_gamepad_models = ["xbox-series-x", "logitech-f710"]

        if self._gamepad_model not in self._supported_gamepad_models:
            raise ValueError(f"Declared gamepad model is not supported! Supported models: {self._supported_gamepad_models}")

        self._base_height = self._base_height_default
        self._speed = SpeedCommand()
        self._roll = 0.0
        self._pitch = 0.0
        self._gait_type = GaitType.TRIPOD

        self._max_angular_speed = self._max_angular_vel_tripod
        self._max_linear_speed = self._max_vel_tripod

        self._change_walk_pressed = False

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

        if self._gamepad_model == "xbox-series-x":
            vlin_axis1 = axes[0]
            vlin_axis2 = axes[1]
            angular_cw_axis = axes[5]
            angular_ccw_axis = axes[4]
            pitch_axis = axes[3]
            roll_axis = axes[2]
            pitch_roll_reset_btn = buttons[14]
            base_height_increment_btn = buttons[4]
            base_height_decrement_btn = buttons[0]
            change_walk_mode_btn = buttons[3]
        elif self._gamepad_model == "logitech-f710":
            vlin_axis1 = axes[0]
            vlin_axis2 = axes[1]
            angular_cw_axis = axes[2]
            angular_ccw_axis = axes[5]
            pitch_axis = axes[4]
            roll_axis = axes[3]
            pitch_roll_reset_btn = buttons[10]
            base_height_increment_btn = buttons[3]
            base_height_decrement_btn = buttons[0]
            change_walk_mode_btn = buttons[2]


        # Linear velocity value and direction
        vval = np.sqrt(np.power(-vlin_axis1, 2) + np.power(vlin_axis2, 2))
        vdir = np.arctan2(vlin_axis1, vlin_axis2)

        self._speed.vx = math.cos(vdir) * vval
        self._speed.vy = math.sin(vdir) * vval


        angular_clockwise_trigger = (1 - round(max(0.001, angular_cw_axis), 2))
        angular_anticlockwise_trigger = (1 - round(max(0.001, angular_ccw_axis), 2))

        if angular_clockwise_trigger > 0.0:
            self._speed.omega = 0.5*pow(angular_clockwise_trigger, 3) * self._max_angular_speed
            self._speed.vx = 0.0
            self._speed.vy = 0.0
        elif angular_anticlockwise_trigger > 0.0:
            self._speed.omega = - 0.5*pow(angular_anticlockwise_trigger, 3) * self._max_angular_speed
            self._speed.vx = 0.0
            self._speed.vy = 0.0
        elif angular_clockwise_trigger == 0.0 or angular_anticlockwise_trigger == 0.0:
            self._speed.omega = 0.0

        if base_height_decrement_btn and self._base_height - self._BASE_HEIGHT_STEP >= self._base_height_min:
            self._base_height -= self._BASE_HEIGHT_STEP
        elif base_height_increment_btn and self._base_height + self._BASE_HEIGHT_STEP <= self._base_height_max:
            self._base_height += self._BASE_HEIGHT_STEP


        if pitch_roll_reset_btn:
            self._roll = 0.0
            self._pitch = 0.0
            self.get_logger().info("Roll and pitch of the main body moved to zero positions")
        
        new_pitch = self._pitch + self._ROLL_PITCH_STEP * pitch_axis
        new_roll = self._roll - self._ROLL_PITCH_STEP * roll_axis

        x = pow(new_roll, 2) / pow(self._roll_max_rad, 2) + pow(new_pitch, 2) / pow(self._pitch_max_rad, 2) 

        if x <= 1:
            self._roll = new_roll
            self._pitch = new_pitch
            self.send_pitch_command(self._pitch)
            self.send_roll_command(self._roll)

        if change_walk_mode_btn and self._speed.norm() == 0.0 and not self._change_walk_pressed:
            self._change_walk_pressed = True
            match self._gait_type:
                case GaitType.WAVE:
                    self._gait_type = GaitType.RIPPLE
                    self._max_angular_speed = self._max_angular_vel_ripple
                    self._max_linear_speed = self._max_angular_vel_ripple
                    self.get_logger().info("Changed WALK to RIPPLE")
                case GaitType.RIPPLE:
                    self._gait_type = GaitType.TRIPOD
                    self._max_angular_speed = self._max_angular_vel_tripod
                    self._max_linear_speed = self._max_angular_vel_tripod
                    self.get_logger().info("Changed WALK to TRIPOD")
                case GaitType.TRIPOD:
                    self._gait_type = GaitType.WAVE
                    self._max_angular_speed = self._max_angular_vel_wave
                    self._max_linear_speed = self._max_angular_vel_wave
                    self.get_logger().info("Changed WALK to WAVE")
            self.send_gait_type_command(self._gait_type)
        
        if not change_walk_mode_btn and self._change_walk_pressed:
            self._change_walk_pressed = False

        self.send_vel_command(self._speed)
        self.send_base_height_command(self._base_height)


def main(args=None):
    rclpy.init(args=args)
    node = ElkapodJoyNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()