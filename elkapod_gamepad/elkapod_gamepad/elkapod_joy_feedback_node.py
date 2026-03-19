import rclpy
from elkapod_msgs.srv import (
    GenerateFeedback,
    GenerateFeedback_Request,
    GenerateFeedback_Response,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JoyFeedback


class ElkapodJoyFeedbackNode(Node):
    def __init__(self):
        super().__init__("elkapod_joy_feedback_node")
        self._feedback_generated_flag = False

        self._joy_feedback_pub = self.create_publisher(JoyFeedback, "/joy/set_feedback", 10)

        self._init_time = self.get_clock().now()
        self._duration = 0.0
        self._intensivity = 0.0

        self._feedback_ack_trigger = self.create_service(
            GenerateFeedback,
            "/joy_generate_feedback",
            self._start_generator_callback,
        )
        self._feedback_timer = self.create_timer(0.1, self._feedback_timer_callback)  # 10 Hz
        self.get_logger().info("Feedback controller node started!")

    def _start_generator_callback(self, request: GenerateFeedback_Request, response: GenerateFeedback_Response):
        if not self._feedback_generated_flag:
            self._duration = request.duration
            self._intensivity = request.intensivity
            self._feedback_generated_flag = True
            self._init_time = self.get_clock().now()
            response.success = True
        else:
            response.success = False
        return response

    def _publish_feedback(self, intensity):
        feedback_msg = JoyFeedback()

        feedback_msg.type = JoyFeedback.TYPE_RUMBLE
        feedback_msg.id = 0

        feedback_msg.intensity = float(intensity)
        self._joy_feedback_pub.publish(feedback_msg)

    def _feedback_timer_callback(self):
        if not self._feedback_generated_flag:
            return

        elapsed = (self.get_clock().now() - self._init_time).nanoseconds / 1e9

        if elapsed < self._duration:
            self._publish_feedback(self._intensivity)
        else:
            self._publish_feedback(0.0)

            self._feedback_generated_flag = False
            self._duration = 0.0
            self._intensivity = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ElkapodJoyFeedbackNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
