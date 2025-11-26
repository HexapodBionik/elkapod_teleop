from enum import Enum
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from elkapod_msgs.action import MotionManagerTrigger
from rclpy.action import ActionClient
from PySide6.QtCore import QObject, Signal
import math


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


class ROS2QtBridge(QObject):
    send_async_cmd_signal = Signal(bool)


class ElkapodControllerGui(Node):
    def __init__(self):
        super().__init__("ElkapodControllerGui")

        self._cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._cmd_gait_type_publisher = self.create_publisher(
            Int32, "/cmd_gait_type", 10
        )
        self._cmd_base_height_publisher = self.create_publisher(
            Float64, "/cmd_base_height", 10
        )
        self._cmd_pitch_publisher = self.create_publisher(
            Float64, "/pitch_setpoint", 10
        )
        self._cmd_roll_publisher = self.create_publisher(Float64, "/roll_setpoint", 10)
        self._motion_manager_transition_client = ActionClient(
            self, MotionManagerTrigger, "/motion_manager_transition"
        )
        self._motion_manager_walk_enable_client = self.create_client(
            Trigger,
            "/motion_manager_walk_enable",
        )
        self._motion_manager_walk_disable_client = self.create_client(
            Trigger, "/motion_manager_walk_disable"
        )

        self._slam_pause_client = self.create_client(Trigger, "/rtabmap/pause")
        self._slam_resume_client = self.create_client(Trigger, "/rtabmap/resume")
        self._slam_restart_client = self.create_client(Trigger, "/rtabmap/reset")

        self._slam_set_mapping_client = self.create_client(
            Trigger, "/rtabmap/set_mode_mapping"
        )
        self._slam_set_localization_client = self.create_client(
            Trigger, "/rtabmap/set_mode_localization"
        )

        self._odom_pause_client = self.create_client(Trigger, "/pause_odom")
        self._odom_resume_client = self.create_client(Trigger, "/resume_odom")
        self._odom_restart_client = self.create_client(Trigger, "/reset_odom")
        # self._odom_subscriber = self.create_subscription(
        #     Odometry, '/icp_odom', self._odometry_callback, 10)
        # self.odom_function_handler: function = None
        self._send_goal_future = None
        self.ros2_qt_bridge = ROS2QtBridge()

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

    def send_motion_manager_transition(self, transition: str):
        possible_transitions = ["init", "stand_up", "lower"]

        if transition in possible_transitions:
            result = self._motion_manager_transition_client.wait_for_server(
                timeout_sec=5.0
            )
            if not result:
                self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            else:
                goal_msg = MotionManagerTrigger.Goal()
                goal_msg.transition = transition
                self._send_goal_future = (
                    self._motion_manager_transition_client.send_goal_async(goal_msg)
                )
                self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            self._done_future = goal_handle.get_result_async()
            self._done_future.add_done_callback(self._done_callback)

    def _done_callback(self, future):
        result = future.result().result
        if result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(True)
        else:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)

    def _service_done_callback(self, future):
        if future.result().success:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(True)
        else:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)

    def send_walk_enable_cmd(self):
        result = self._motion_manager_walk_enable_client.wait_for_service(
            timeout_sec=5.0
        )
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = self._motion_manager_walk_enable_client.call_async(
                goal
            )
            self._send_goal_future.add_done_callback(self._service_done_callback)

    def send_walk_disable_cmd(self):
        result = self._motion_manager_walk_disable_client.wait_for_service(
            timeout_sec=5.0
        )
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = (
                self._motion_manager_walk_disable_client.call_async(goal)
            )
            self._send_goal_future.add_done_callback(self._service_done_callback)

    def send_slam_pause_cmd(self):
        result = self._slam_pause_client.wait_for_service(timeout_sec=5.0)
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = self._slam_pause_client.call_async(goal)
            self._send_goal_future.add_done_callback(self._service_done_callback)

    def send_slam_resume_cmd(self):
        result = self._slam_resume_client.wait_for_service(timeout_sec=5.0)
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = self._slam_resume_client.call_async(goal)
            self._send_goal_future.add_done_callback(self._service_done_callback)

    def send_slam_restart_cmd(self):
        result = self._slam_restart_client.wait_for_service(timeout_sec=5.0)
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = self._slam_restart_client.call_async(goal)
            self._send_goal_future.add_done_callback(self._service_done_callback)

    def send_odom_pause_cmd(self):
        result = self._odom_pause_client.wait_for_service(timeout_sec=5.0)
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = self._odom_pause_client.call_async(goal)
            self._send_goal_future.add_done_callback(self._service_done_callback)

    def send_odom_resume_cmd(self):
        result = self._odom_resume_client.wait_for_service(timeout_sec=5.0)
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = self._odom_resume_client.call_async(goal)
            self._send_goal_future.add_done_callback(self._service_done_callback)

    def send_odom_restart_cmd(self):
        result = self._odom_restart_client.wait_for_service(timeout_sec=5.0)
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = self._odom_restart_client.call_async(goal)
            self._send_goal_future.add_done_callback(self._service_done_callback)

    def send_slam_mapping_cmd(self):
        result = self._slam_set_mapping_client.wait_for_service(timeout_sec=5.0)
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = self._slam_set_mapping_client.call_async(goal)
            self._send_goal_future.add_done_callback(self._service_done_callback)

    def send_slam_localization_cmd(self):
        result = self._slam_set_localization_client.wait_for_service(timeout_sec=5.0)
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = self._slam_set_localization_client.call_async(goal)
            self._send_goal_future.add_done_callback(self._service_done_callback)

    # def _odometry_callback(self, msg):
    #     if self.odom_function_handler is None:
    #         print(f'{__name__} has no logic registered')
    #         print(f'Don\'t know what to do with {msg}')
    #         return
    #     self.odom_function_handler(msg)

    # def register_callback_handler(self, function_handler, new_function):
    #     print(function_handler)
    #     atr = getattr(self, str(function_handler), None)
    #     if atr:
    #         atr = new_function
    #     else:
    #         raise AttributeError(
    #             f"Attribute {str(function_handler)}, cannot be found in {self}")
