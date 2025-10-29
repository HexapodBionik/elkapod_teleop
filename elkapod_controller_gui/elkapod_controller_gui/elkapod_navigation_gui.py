from PySide6.QtCore import Signal, Slot
from PySide6.QtWidgets import (
    QWidget,
    QLabel
)

from .elkapod_navigation_ui import Ui_Form
from .elkapod_gui_node import SpeedCommand, GaitType
from .elkapod_gui_node import ElkapodControllerGui
from enum import Enum


class NodeStatus(Enum):
    OFF = 0
    RUNNING = 1
    PAUSED = 2


class ApplicationNavWindow(QWidget):
    MAPPING = "MAPPING"
    LOCALIZATION = "LOCALIZATION"

    odom_info_update = Signal(str)
    def __init__(self, parent=None):
        self.node: ElkapodControllerGui = None
        super().__init__(parent)
        self._ui = Ui_Form()
        self._ui.setupUi(self)

        self.odom_status: NodeStatus = None
        self.slam_status: NodeStatus = None

        self.odom_info_update.connect(self.on_label_update)

    def setup(self):
        self.update_status_label(self._ui.slam_status, NodeStatus.OFF)
        self._ui.slam_mode_combo.addItems([self.MAPPING, self.LOCALIZATION])
        self._ui.slam_mode_combo.currentTextChanged.connect(self._update_slam_mode)

        self._ui.slam_resume.pressed.connect(self._resume_slam)
        self._ui.slam_pause.pressed.connect(self._pause_slam)
        self._ui.slam_restart.pressed.connect(self._restart_slam)

        self.update_status_label(self._ui.odom_status, NodeStatus.OFF)
        self._ui.odom_resume.pressed.connect(self._resume_odom)
        self._ui.odom_pause.pressed.connect(self._pause_odom)
        self._ui.odom_restart.pressed.connect(self._restart_odom)

    @staticmethod
    def update_status_label(label: QLabel, new_status: NodeStatus):
        label.setText(new_status.name)
        match new_status:
            case NodeStatus.OFF:
                label.setStyleSheet("color: #BF0E34")
            case NodeStatus.RUNNING:
                label.setStyleSheet("color: #0EBF67")
            case NodeStatus.PAUSED:
                label.setStyleSheet("color: #EAD765")

    def _update_slam_mode(self, mode: str):
        match mode:
            case self.LOCALIZATION:
                self.node.send_slam_localization_cmd()
            case self.MAPPING:
                self.node.send_slam_mapping_cmd()
            case _:
                print(f"Something failed time to die, got {mode}")

    def _resume_slam(self):
        if self.slam_status == NodeStatus.RUNNING:
            return
        else:
            self.update_status_label(self._ui.slam_status, NodeStatus.RUNNING)
            self.node.send_slam_resume_cmd()

    def _pause_slam(self):
        if self.slam_status == NodeStatus.PAUSED:
            return
        else:
            self.update_status_label(self._ui.slam_status, NodeStatus.PAUSED)
            self.node.send_slam_pause_cmd()

    def _restart_slam(self):
        if self.slam_status == NodeStatus.OFF:
            return
        else:
            self.update_status_label(self._ui.slam_status, NodeStatus.RUNNING)
            self.node.send_slam_restart_cmd()

    def _resume_odom(self):
        if self.odom_status == NodeStatus.RUNNING:
            return
        else:
            self.update_status_label(self._ui.odom_status, NodeStatus.RUNNING)
            self.node.send_odom_resume_cmd()

    def _pause_odom(self):
        if self.odom_status == NodeStatus.PAUSED:
            return
        else:
            self.update_status_label(self._ui.odom_status, NodeStatus.PAUSED)
            self.node.send_odom_pause_cmd()

    def _restart_odom(self):
        if self.odom_status == NodeStatus.OFF:
            return
        else:
            self.update_status_label(self._ui.odom_status, NodeStatus.RUNNING)
            self.node.send_odom_restart_cmd()


    @Slot(str)
    def on_label_update(self, text):
        self._ui.odomInfo.setText(text)

    def set_label_text_threadsafe(self, text):
        """Can be safely called from ROS callbacks"""
        self.odom_info_update.emit(text)