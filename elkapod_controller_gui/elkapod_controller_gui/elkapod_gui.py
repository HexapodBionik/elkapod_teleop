import rclpy
from rclpy.executors import MultiThreadedExecutor
from .elkapod_gui_node import ElkapodControllerGui
from .elkapod_controller_gui import ApplicationMainWindow
from .elkapod_controller_ui import QApplication
import sys
import threading
import os

def main(args=None):
    rclpy.init(args=args)

    app = QApplication()

    window = ApplicationMainWindow()
    ros_node = ElkapodControllerGui()
    window.node = ros_node
    window.setup()

    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    thread = threading.Thread(target=executor.spin)
    thread.start()

    try:
        window.show()
        sys.exit(app.exec())
    finally:
        ros_node.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
