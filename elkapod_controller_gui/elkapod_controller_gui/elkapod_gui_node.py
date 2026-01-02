from enum import Enum
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger, Empty
from nav_msgs.msg import Odometry
from elkapod_msgs.action import MotionManagerTrigger
from rclpy.action import ActionClient
from PySide6.QtCore import QObject, Signal
from tf_transformations import euler_from_quaternion
# from nav2_simple_commander.robot_navigator import BasicNavi   ator
from rtabmap_msgs.msg import Info
from rtabmap_msgs.srv import PublishMap
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
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
        return math.sqrt(self.vx ** 2 + self.vy ** 2)


class ROS2QtBridge(QObject):
    send_async_cmd_signal = Signal(bool)
    odometry_received_signal = Signal(str)
    map_info_received_signal = Signal(str)


class ElkapodControllerGui(Node):
    def __init__(self):
        super().__init__("ElkapodControllerGui")
        self._last_emit_time = 0.0
        self._emit_period = 0.2
        self.declare_parameter("odom_topic", "/icp_odom")
        self.declare_parameter("map_info_topic", "/info")

        # self.nav_commander = BasicNavigator(namespace='/navigation')

        odom_topic = self.get_parameter("odom_topic").value
        map_info_topic = self.get_parameter("map_info_topic").value
        self._cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._cmd_gait_type_publisher = self.create_publisher(
            Int32, "/cmd_gait_type", 10)
        self._cmd_base_height_publisher = self.create_publisher(
            Float64, "/cmd_base_height", 10)
        self._cmd_pitch_publisher = self.create_publisher(
            Float64, "/pitch_setpoint", 10)
        self._cmd_roll_publisher = self.create_publisher(
            Float64, "/roll_setpoint", 10)
        self._motion_manager_transition_client = ActionClient(
            self, MotionManagerTrigger, "/motion_manager_transition")
        self._motion_manager_walk_enable_client = self.create_client(
            Trigger, "/motion_manager_walk_enable", )
        self._motion_manager_walk_disable_client = self.create_client(
            Trigger, "/motion_manager_walk_disable")

        self.odom_subscriber = self.create_subscription(Odometry,odom_topic,self._odometry_callback,10)
        self.map_info_subscriber = self.create_subscription(Info, map_info_topic, self._map_info_callback,10)

        self.resume_odom_client = self.create_client(Empty, "/resume_odom")
        self.pause_odom_client = self.create_client(Empty, "/pause_odom")
        self.reset_odom_client = self.create_client(Empty, "/reset_odom")

        self.pause_slam_client = self.create_client(Empty, "/rtabmap/pause")
        self.resume_slam_client = self.create_client(Empty, "/rtabmap/resume")
        self.reset_slam_client = self.create_client(Empty, "/rtabmap/reset")

        self.set_mode_localization_client = self.create_client(Empty, "/rtabmap/set_mode_localization")
        self.set_mode_mapping_client = self.create_client(Empty, "/rtabmap/set_mode_mapping")

        self.publish_map_client = self.create_client(PublishMap, "/rtabmap/publish_map")
        self.create_new_map_client = self.create_client(Empty, "/rtabmap/trigger_new_map")
        
        self._cancel_client = self.create_client(
            CancelGoal, '/navigation/navigate_to_pose/_action/cancel_goal')
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
                timeout_sec=5.0)
            if not result:
                self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            else:
                goal_msg = MotionManagerTrigger.Goal()
                goal_msg.transition = transition
                self._send_goal_future = self._motion_manager_transition_client.send_goal_async(
                    goal_msg)
                self._send_goal_future.add_done_callback(
                    self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
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
            timeout_sec=5.0)
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = self._motion_manager_walk_enable_client.call_async(
                goal)
            self._send_goal_future.add_done_callback(
                self._service_done_callback)

    def send_walk_disable_cmd(self):
        result = self._motion_manager_walk_disable_client.wait_for_service(
            timeout_sec=5.0)
        if not result:
            self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
        else:
            goal = Trigger.Request()
            self._send_goal_future = self._motion_manager_walk_disable_client.call_async(
                goal)
            self._send_goal_future.add_done_callback(
                self._service_done_callback)

    def send_odom_pause_cmd(self):
        result = self.pause_odom_client.wait_for_service(timeout_sec=3.0)
        if not result:
            # self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            print("Failed accessing for odom pause service")
        else:
            goal = Empty.Request()
            self._send_goal_future = self.pause_odom_client.call_async(goal)
#            self._send_goal_future.add_done_callback(
#                self._service_done_callback)

    def send_odom_resume_cmd(self):
        result = self.resume_odom_client.wait_for_service(timeout_sec=3.0)
        if not result:
            # self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            print("Failed accessing for odom resume service")
        else:
            goal = Empty.Request()
            self._send_goal_future = self.resume_odom_client.call_async(goal)
#            self._send_goal_future.add_done_callback(
#                self._service_done_callback)

    def send_odom_restart_cmd(self):
        result = self.reset_odom_client.wait_for_service(timeout_sec=3.0)
        if not result:
            # self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            print("Failed accessing for odom reset service")
        else:
            goal = Empty.Request()
            self._send_goal_future = self.reset_odom_client.call_async(goal)
#            self._send_goal_future.add_done_callback(
#                self._service_done_callback)

    def send_slam_pause_cmd(self):
        result = self.pause_slam_client.wait_for_service(timeout_sec=3.0)
        if not result:
            # self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            print("Failed accessing for slam pause service")
        else:
            goal = Empty.Request()
            self._send_goal_future = self.pause_slam_client.call_async(goal)
#            self._send_goal_future.add_done_callback(
#                self._service_done_callback)

    def send_slam_resume_cmd(self):
        result = self.resume_slam_client.wait_for_service(timeout_sec=3.0)
        if not result:
            # self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            print("Failed accessing for slam resume service")
        else:
            goal = Empty.Request()
            self._send_goal_future = self.resume_slam_client.call_async(goal)
#            self._send_goal_future.add_done_callback(
#                self._service_done_callback)

    def send_slam_restart_cmd(self):
        result = self.reset_slam_client.wait_for_service(timeout_sec=3.0)
        if not result:
            # self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            print("Failed accessing for slam reset service")
        else:
            goal = Empty.Request()
            self._send_goal_future = self.reset_slam_client.call_async(goal)
#            self._send_goal_future.add_done_callback(
#                self._service_done_callback)

    def send_slam_mapping_mode_cmd(self):
        result = self.set_mode_mapping_client.wait_for_service(timeout_sec=3.0)
        if not result:
            # self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            print("Failed accessing for slam mode service")
        else:
            goal = Empty.Request()
            self._send_goal_future = self.set_mode_mapping_client.call_async(goal)
#            self._send_goal_future.add_done_callback(
#                self._service_done_callback)

    def send_slam_localization_mode_cmd(self):
        result = self.set_mode_localization_client.wait_for_service(timeout_sec=3.0)
        if not result:
            # self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            print("Failed accessing for slam mode service")
        else:
            goal = Empty.Request()
            self._send_goal_future = self.set_mode_localization_client.call_async(goal)
#            self._send_goal_future.add_done_callback(
#                self._service_done_callback)


    def send_publish_map_cmd(self):
        result = self.publish_map_client.wait_for_service(timeout_sec=3.0)
        if not result:
            # self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            print("Failed accessing for publish map service")
        else:
            goal = PublishMap.Request()
            goal.global_map = True
            goal.optimized = True
            goal.graph_only = False
            self._send_goal_future = self.publish_map_client.call_async(goal)
#            self._send_goal_future.add_done_callback(
#                self._service_done_callback)

    def send_new_map_cmd(self):
        result = self.create_new_map_client.wait_for_service(timeout_sec=3.0)
        if not result:
            # self.ros2_qt_bridge.send_async_cmd_signal.emit(False)
            print("Failed accessing for create mew map service")
        else:
            goal = Empty.Request()
            self._send_goal_future = self.create_new_map_client.call_async(goal)
#            self._send_goal_future.add_done_callback(
#                self._service_done_callback)

    def _odometry_callback(self, msg:Odometry):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if timestamp - self._last_emit_time < self._emit_period:
            return
        self._last_emit_time = timestamp

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        
        #                                                  theta
        data = f"[{timestamp:.2f}] x: {x:.2f}, y: {y:.2f}, \u03b8: {yaw:.2f}"
        self.ros2_qt_bridge.odometry_received_signal.emit(data)


    def _map_info_callback(self, msg:Info):
        stats = dict(zip(msg.stats_keys, msg.stats_values))
        wm_size = stats.get('Memory/Working_memory_size/', -1)
        distance_travelled = stats.get('Memory/Distance_travelled/m', None)
        
        data = f"Nodes {wm_size}, travelled {distance_travelled:.2f}m"
        self.ros2_qt_bridge.map_info_received_signal.emit(data)

    def abort_task(self):
        """Call this on button click"""
        if not self._cancel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Nav2 Cancel Service not available!")
            return

        request = CancelGoal.Request()
        self.get_logger().info("Sending Kill Signal to Nav2...")
        future = self._cancel_client.call_async(request)
        future.add_done_callback(self._cancel_callback)

    def _cancel_callback(self, future):
        try:
            response = future.result()

            if len(response.goals_canceling) > 0:
                self.get_logger().info(f"Success! Canceling {len(response.goals_canceling)} goals.")
            else:
                self.get_logger().warn("No active goals found to cancel.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")