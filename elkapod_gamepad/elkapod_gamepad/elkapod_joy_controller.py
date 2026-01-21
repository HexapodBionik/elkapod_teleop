import math
from enum import Enum

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from elkapod_msgs.action import MotionManagerTrigger
from elkapod_msgs.srv import GenerateFeedback
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger


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


class RobotState(Enum):
    INIT = 0
    IDLE_LOWERED = 1
    IDLE = 2
    WALKING = 3


class ElkapodJoyNode(Node):
    _ROLL_PITCH_STEP = 0.005
    _BASE_HEIGHT_STEP = 0.001

    def __init__(self):
        super().__init__("elkapod_joy")
        self.declare_parameter(
            "base_height.min_base_height",
            None,
            ParameterDescriptor(description="Min base height", dynamic_typing=True),
        )
        self.declare_parameter(
            "base_height.max_base_height",
            None,
            ParameterDescriptor(description="Max base height", dynamic_typing=True),
        )
        self.declare_parameter(
            "base_height.default_base_height",
            None,
            ParameterDescriptor(description="Default base height", dynamic_typing=True),
        )
        self.declare_parameter(
            "roll.max_rad",
            None,
            ParameterDescriptor(description="Max roll value in rad", dynamic_typing=True),
        )
        self.declare_parameter(
            "pitch.max_rad",
            None,
            ParameterDescriptor(description="Max pitch value in rad", dynamic_typing=True),
        )
        self.declare_parameter(
            "max_vel.tripod",
            None,
            ParameterDescriptor(description="Max linear velocity for tripod gait", dynamic_typing=True),
        )
        self.declare_parameter(
            "max_vel.wave",
            None,
            ParameterDescriptor(description="Max linear velocity for wave gait", dynamic_typing=True),
        )
        self.declare_parameter(
            "max_vel.ripple",
            None,
            ParameterDescriptor(description="Max linear velocity for ripple gait", dynamic_typing=True),
        )
        self.declare_parameter(
            "max_angular_vel.tripod",
            None,
            ParameterDescriptor(description="Max angular velocity for tripod gait", dynamic_typing=True),
        )
        self.declare_parameter(
            "max_angular_vel.wave",
            None,
            ParameterDescriptor(description="Max angular velocity for wave gait", dynamic_typing=True),
        )
        self.declare_parameter(
            "max_angular_vel.ripple",
            None,
            ParameterDescriptor(description="Max angular velocity for ripple gait", dynamic_typing=True),
        )
        self.declare_parameter(
            "gamepad_model",
            "xbox-series-x",
            ParameterDescriptor(description="Model of gamepad use for control", dynamic_typing=True),
        )

        self._joy_subscriber = self.create_subscription(Joy, "/joy", self._joystick_callback, 10)

        self._cmd_vel_publisher = self.create_publisher(Twist, "/joy_vel", 10)
        self._cmd_gait_type_publisher = self.create_publisher(Int32, "/cmd_gait_type", 10)
        self._cmd_base_height_publisher = self.create_publisher(Float64, "/cmd_base_height", 10)
        self._cmd_pitch_publisher = self.create_publisher(Float64, "/pitch_setpoint", 10)
        self._cmd_roll_publisher = self.create_publisher(Float64, "/roll_setpoint", 10)

        self._motion_manager_transition_client = ActionClient(self, MotionManagerTrigger, "/motion_manager_transition")
        self._motion_manager_walk_enable_client = self.create_client(
            Trigger,
            "/motion_manager_walk_enable",
        )
        self._motion_manager_walk_disable_client = self.create_client(Trigger, "/motion_manager_walk_disable")

        self._generate_joy_feedback_client = self.create_client(GenerateFeedback, "/joy_generate_feedback")

        self.subscription = self.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostic_callback, 10)

        self._base_height_min = self.get_parameter("base_height.min_base_height").get_parameter_value().double_value
        self._base_height_max = self.get_parameter("base_height.max_base_height").get_parameter_value().double_value
        self._base_height_default = (
            self.get_parameter("base_height.default_base_height").get_parameter_value().double_value
        )
        self._roll_max_rad = self.get_parameter("roll.max_rad").get_parameter_value().double_value
        self._pitch_max_rad = self.get_parameter("pitch.max_rad").get_parameter_value().double_value
        self._max_vel_tripod = self.get_parameter("max_vel.tripod").get_parameter_value().double_value
        self._max_vel_wave = self.get_parameter("max_vel.wave").get_parameter_value().double_value
        self._max_vel_ripple = self.get_parameter("max_vel.ripple").get_parameter_value().double_value
        self._max_angular_vel_tripod = self.get_parameter("max_angular_vel.tripod").get_parameter_value().double_value
        self._max_angular_vel_wave = self.get_parameter("max_angular_vel.wave").get_parameter_value().double_value
        self._max_angular_vel_ripple = self.get_parameter("max_angular_vel.ripple").get_parameter_value().double_value
        self._gamepad_model = self.get_parameter("gamepad_model").get_parameter_value().string_value

        self._supported_gamepad_models = ["xbox-series-x"]

        if self._gamepad_model not in self._supported_gamepad_models:
            raise ValueError(
                f"Declared gamepad model is not supported! Supported models: {self._supported_gamepad_models}"
            )

        self._base_height = self._base_height_default
        self._speed = SpeedCommand()
        self._roll = 0.0
        self._pitch = 0.0
        self._gait_type = GaitType.TRIPOD

        self._max_angular_speed = self._max_angular_vel_tripod
        self._max_linear_speed = self._max_vel_tripod

        self._robot_state = RobotState.INIT
        self._next_state = RobotState.IDLE_LOWERED
        self._transition_flag = False
        self._last_transition_time = self.get_clock().now()

        self._change_walk_pressed = False

    def _diagnostic_callback(self, msg: DiagnosticArray):
        for status in msg.status:
            if status.name == "ElkapodMotionManager: State":
                for kv in status.values:
                    if kv.key == "state":
                        self._robot_state = self.string_to_state(kv.value)

    def _send_idle_transition(self):
        if self._robot_state == RobotState.IDLE_LOWERED:
            self._send_motion_manager_transition("stand_up")
        elif self._robot_state == RobotState.WALKING:
            self.send_walk_disable_cmd()
        self._next_state = RobotState.IDLE
        self.get_logger().info(f"Starting transition to {self.state_to_string(self._next_state)} ...")

    def _send_walk_transition(self):
        self.send_walk_enable_cmd()
        self._next_state = RobotState.WALKING
        self.get_logger().info(f"Starting transition to {self.state_to_string(self._next_state)} ...")

    def _send_init_transition(self):
        if self._robot_state == RobotState.INIT:
            self._send_motion_manager_transition("init")
        elif self._robot_state == RobotState.IDLE:
            self._send_motion_manager_transition("lower")
        self._next_state = RobotState.IDLE_LOWERED

    @staticmethod
    def state_to_string(state: RobotState):
        state_to_str = {
            RobotState.INIT: "init",
            RobotState.IDLE_LOWERED: "idle_lowered",
            RobotState.IDLE: "idle",
            RobotState.WALKING: "walking",
        }
        return state_to_str.get(state)

    @staticmethod
    def string_to_state(state: str):
        str_to_state = {
            "init": RobotState.INIT,
            "idle_lowered": RobotState.IDLE_LOWERED,
            "idle": RobotState.IDLE,
            "walking": RobotState.WALKING,
        }
        return str_to_state.get(state)

    def _on_transition_result(self, result: bool):
        if result:
            self.get_logger().info(f"Transition to {self.state_to_string(self._next_state)} succeded!")
            self._robot_state = self._next_state
            self._next_state = None
        else:
            self.get_logger().warn(f"Transition to {self.state_to_string(self._next_state)} failed!")
            self._next_state = None

    def _send_motion_manager_transition(self, transition: str):
        possible_transitions = ["init", "stand_up", "lower"]

        if not self._transition_flag:
            if transition in possible_transitions:
                result = self._motion_manager_transition_client.wait_for_server(timeout_sec=5.0)
                if result:
                    goal_msg = MotionManagerTrigger.Goal()
                    goal_msg.transition = transition
                    self._send_goal_future = self._motion_manager_transition_client.send_goal_async(goal_msg)
                    self._send_goal_future.add_done_callback(self._goal_response_callback)
                else:
                    self.get_logger().error("Transition failed! Motion manager transition client not available!")
        else:
            self.get_logger().warn("Other transition ongoing, transition goal rejected")

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected :(")
        else:
            self._transition_flag = True
            self._done_future = goal_handle.get_result_async()
            self._done_future.add_done_callback(self._done_callback)

    def _done_callback(self, future):
        self._on_transition_result(future.result().result)
        result = self._generate_joy_feedback_client.wait_for_service(timeout_sec=1.0)
        if result:
            goal = GenerateFeedback.Request()
            goal.duration = 0.3
            goal.intensivity = 1.0
            self._generate_joy_feedback_client.call_async(goal)
        self._transition_flag = False
        self._last_transition_time = self.get_clock().now()

    def _service_done_callback(self, future):
        self._on_transition_result(future.result().success)
        result = self._generate_joy_feedback_client.wait_for_service(timeout_sec=1.0)
        if result:
            goal = GenerateFeedback.Request()
            goal.duration = 0.3
            goal.intensivity = 1.0
            self._generate_joy_feedback_client.call_async(goal)

        self._transition_flag = False
        self._last_transition_time = self.get_clock().now()

    def send_walk_enable_cmd(self):
        if not self._transition_flag:
            result = self._motion_manager_walk_enable_client.wait_for_service(timeout_sec=5.0)
            if result:
                goal = Trigger.Request()
                self._send_goal_future = self._motion_manager_walk_enable_client.call_async(goal)
                self._send_goal_future.add_done_callback(self._service_done_callback)
            else:
                self.get_logger().error("Transition failed! Motion manager transition client not available!")
        else:
            self.get_logger().warn("Other transition ongoing, transition goal rejected")

    def send_walk_disable_cmd(self):
        if not self._transition_flag:
            result = self._motion_manager_walk_disable_client.wait_for_service(timeout_sec=5.0)
            if result:
                goal = Trigger.Request()
                self._send_goal_future = self._motion_manager_walk_disable_client.call_async(goal)
                self._send_goal_future.add_done_callback(self._service_done_callback)
            else:
                self.get_logger().error("Transition failed! Motion manager transition client not available!")
        else:
            self.get_logger().warn("Other transition ongoing, transition goal rejected")

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
            sprint_btn = buttons[13]
            pitch_roll_reset_btn = buttons[14]
            transition_trigger_up = buttons[4]
            transition_trigger_down = buttons[0]
            body_height_axis = axes[7]
            change_walk_mode_btn = buttons[3]

        if not self._transition_flag and (self.get_clock().now() - self._last_transition_time).nanoseconds >= 1.0 * 1e9:
            if transition_trigger_up:
                match self._robot_state:
                    case RobotState.INIT:
                        self._send_init_transition()
                    case RobotState.IDLE_LOWERED:
                        self._send_idle_transition()
                    case RobotState.IDLE:
                        self._send_walk_transition()
            elif transition_trigger_down:
                match self._robot_state:
                    case RobotState.IDLE:
                        self._send_init_transition()
                    case RobotState.WALKING:
                        self.send_walk_disable_cmd()
                        self._next_state = RobotState.IDLE

        # Linear velocity value and direction
        vval = np.sqrt(np.power(-vlin_axis1, 2) + np.power(vlin_axis2, 2))
        vdir = np.arctan2(vlin_axis1, vlin_axis2)

        scaling = 1.0
        if not sprint_btn:
            scaling = 0.5

        self._speed.vx = math.cos(vdir) * vval * self._max_linear_speed * scaling
        self._speed.vy = math.sin(vdir) * vval * self._max_linear_speed * scaling

        angular_clockwise_trigger = 1 - round(max(0.001, angular_cw_axis), 2)
        angular_anticlockwise_trigger = 1 - round(max(0.001, angular_ccw_axis), 2)

        if angular_clockwise_trigger > 0.0:
            self._speed.omega = 0.5 * pow(angular_clockwise_trigger, 3) * self._max_angular_speed
            self._speed.vx = 0.0
            self._speed.vy = 0.0
        elif angular_anticlockwise_trigger > 0.0:
            self._speed.omega = -0.5 * pow(angular_anticlockwise_trigger, 3) * self._max_angular_speed
            self._speed.vx = 0.0
            self._speed.vy = 0.0
        elif angular_clockwise_trigger == 0.0 or angular_anticlockwise_trigger == 0.0:
            self._speed.omega = 0.0

        self._base_height += body_height_axis * self._BASE_HEIGHT_STEP
        self._base_height = max(self._base_height_min, min(self._base_height, self._base_height_max))

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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
