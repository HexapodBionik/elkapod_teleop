from .elkapod_gui_node import ElkapodControllerGui

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLayout,
    QFrame, QLabel, QPushButton, QComboBox,
    QVBoxLayout, QHBoxLayout, QSizePolicy, QGridLayout,
)
from PySide6.QtGui import QFont
from PySide6.QtCore import Qt
import sys
from enum import Enum


class SectionFrame(QFrame):
    class Status(Enum):
        UNKNOWN = 0
        RUNNING = 1
        PAUSED = 2

    def __init__(self, title: str, parent=None, data_box: bool = True):
        super().__init__(parent)
        self.setFrameShape(QFrame.Box)
        self.setLineWidth(1)
        self.status = self.Status.UNKNOWN

        self._root_layout = QVBoxLayout(self)


        top_layout = QGridLayout()
        top_layout.setVerticalSpacing(15)
        title_label = QLabel(title)
        title_label.setStyleSheet("""
            font-size: 15pt;
            font-weight: bold;
        """)

        self.status_layout = QHBoxLayout()

        status_label = QLabel("Status: ")
        status_label.setSizePolicy(
            QSizePolicy.Policy.Maximum,
            QSizePolicy.Policy.Preferred
        )
        self.status_label = QLabel(self.status.name)
        self.status_layout.addWidget(status_label, 0)
        self.status_layout.addWidget(self.status_label, 0, Qt.AlignmentFlag.AlignLeft)
        self.status_layout.setSpacing(0)

        if data_box:
            self.data_box = QLabel("Lorem ipsum")
            self.data_box.setAlignment(Qt.AlignCenter)
            self.data_box.setFrameShape(QFrame.Box)
            top_layout.addWidget(self.data_box, 0, 1, 1, 2)

        top_layout.addWidget(title_label, 0, 0)
        top_layout.addLayout(self.status_layout, 1, 0)
        self._root_layout.addLayout(top_layout)
        self._root_layout.addSpacing(10)

        self.setSizePolicy(
            QSizePolicy.Policy.Preferred,
            QSizePolicy.Policy.Maximum
        )
    def update_data_box(self,text:str):
        self.data_box.setText(text)


class OdometryFrame(SectionFrame):
    def __init__(self, parent=None):
        super().__init__("Odometry", parent)

        odom_buttons = QHBoxLayout()
        odom_buttons.setSpacing(7)
        self.start_button = QPushButton("START", parent=self)
        self.pause_button = QPushButton("PAUSE", parent=self)
        self.restart_button = QPushButton("RESTART", parent=self)
        odom_buttons.addWidget(self.start_button)
        odom_buttons.addWidget(self.pause_button)
        odom_buttons.addWidget(self.restart_button)

        self._root_layout.addLayout(odom_buttons)
    

class SLAMFrame(SectionFrame):
    def __init__(self, parent=None):
        super().__init__("SLAM", parent)


        slam_buttons = QHBoxLayout()
        slam_buttons.setSpacing(7)
        self.start_button = QPushButton("START")
        self.pause_button = QPushButton("PAUSE")
        self.restart_button = QPushButton("RESTART")

        slam_buttons.addWidget(self.start_button)
        slam_buttons.addWidget(self.pause_button)
        slam_buttons.addWidget(self.restart_button)

        control_layout = QHBoxLayout()
        control_layout.setSpacing(7)
        node_mode_layout = QHBoxLayout()
        node_mode_layout.addWidget(QLabel("Mode:"))

        self.slam_mode = QComboBox()
        self.slam_mode.addItems(["SLAM", "Localization"])
        node_mode_layout.addWidget(self.slam_mode)

        control_layout.addLayout(node_mode_layout)

        self.republish_map_button = QPushButton("Republish Map")
        self.new_map_button = QPushButton("New Map")

        control_layout.addWidget(self.republish_map_button)
        control_layout.addWidget(self.new_map_button)

        self._root_layout.addLayout(slam_buttons)
        self._root_layout.addSpacing(7)
        self._root_layout.addLayout(control_layout)

class NavigationFrame(SectionFrame):
    def __init__(self, parent=None):
        super().__init__("Navigation", parent)
        nav_buttons = QHBoxLayout()
        nav_buttons.setSpacing(7)
        self.abort_button = QPushButton("ABORT")
        self.surprise_button = QPushButton("SURPRISE")

        nav_buttons.addWidget(self.abort_button)
        nav_buttons.addWidget(self.surprise_button)
        self._root_layout.addLayout(nav_buttons)

class ApplicationNavWindow(QMainWindow):
    def __init__(self, node: ElkapodControllerGui | None  = None):
        super().__init__()
   
        self.node = node

        self.setWindowTitle("Navigation UI")
        self.resize(900, 700)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        root_layout = QHBoxLayout(central_widget)
        root_layout.setSpacing(22)

        left_panel = QVBoxLayout()

        root_layout.addLayout(left_panel, 1)

        self.odom_frame = OdometryFrame(central_widget)
        left_panel.addWidget(self.odom_frame)

        self.slam_frame = SLAMFrame(central_widget)
        left_panel.addWidget(self.slam_frame)

        self.nav_frame = NavigationFrame(central_widget)
        left_panel.addWidget(self.nav_frame)
        right_frame = QFrame()
        right_frame.setFrameShape(QFrame.Box)
        right_layout = QVBoxLayout(right_frame)

        self.right_data = QLabel("")
        self.right_data.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.right_data)

        root_layout.addWidget(right_frame, 1)
        left_panel.addStretch()

        if node is not None:
            self.node.ros2_qt_bridge.odometry_received_signal.connect(self.odom_frame.update_data_box)
            