import signal
import sys
import threading

import rclpy
from PySide6.QtWidgets import QApplication
from rclpy.executors import MultiThreadedExecutor

from .elkapod_controller_gui import ApplicationMainWindow
from .elkapod_gui_node import ElkapodControllerGui
from .elkapod_navigation_gui import ApplicationNavWindow


def main(args=None):
    rclpy.init(args=args)

    app = QApplication()

    ros_node = ElkapodControllerGui()
    window = ApplicationMainWindow()
    nav_window = ApplicationNavWindow(ros_node)

    window.node = ros_node
    window.setup()

    # nav_window.setup()

    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    thread = threading.Thread(target=executor.spin)
    thread.start()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    try:
        # window.move(100, 100)
        # nav_window.move(800, 100)
        window.show()
        nav_window.show()
        sys.exit(app.exec())

    finally:
        ros_node.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
