from .elkapod_gui_node import ElkapodControllerGui

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLayout,
    QFrame, QLabel, QPushButton, QComboBox,
    QVBoxLayout, QHBoxLayout, QSizePolicy, QGridLayout,QPlainTextEdit,
)
from PySide6.QtGui import QFont, QFontMetrics,QTextCursor
from PySide6.QtCore import Qt, Signal
import sys
from enum import Enum



class SectionFrame(QFrame):
    class Status(Enum):
        UNKNOWN = 0
        RUNNING = 1
        PAUSED = 2
    _status_color_map = {Status.UNKNOWN:"#7a7a7a", Status.RUNNING:"#4cb568", Status.PAUSED:"#eb3131"}
    dataBoxUpdated = Signal(bool)
    log_signal = Signal(str)
    def __init__(self, title: str, parent=None, node: ElkapodControllerGui | None  = None, data_box: bool = True):
        super().__init__(parent)
        self.setFrameShape(QFrame.Box)
        self.setLineWidth(1)
        self.node = node
        self._status = None

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
        self.status_label = QLabel("")

        self.status_layout.addWidget(status_label, 0)
        self.status_layout.addWidget(self.status_label, 0, Qt.AlignmentFlag.AlignLeft)
        self.status_layout.setSpacing(0)

        if data_box:
            self.data_box = QLabel("No data received")
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
    
        self.status = self.Status.UNKNOWN

    # Is it good? Hell nah. Does it works? Kinda
    @property
    def status(self):
        return self._status
    
    @status.setter
    def status(self, new_status):
        assert isinstance(new_status, self.Status)
        self._status = new_status

        self.status_label.setText(self.status.name)
        new_color = self._status_color_map[self.status]
        self.status_label.setStyleSheet(f"color: {new_color}")

        if self._status != self.Status.RUNNING:
            self._bind_status_databox()
            
    def update_data_box(self,text:str):
        self.data_box.setText(text)
        self.dataBoxUpdated.emit(True)

    def _bind_status_databox(self):
        def one_shot(*args):
            self.status = self.Status.RUNNING
            self.dataBoxUpdated.disconnect(one_shot)
        self.dataBoxUpdated.connect(one_shot)

class OdometryFrame(SectionFrame):
    def __init__(self, parent=None, node: ElkapodControllerGui | None  = None):
        super().__init__("Odometry", parent, node)
        
        odom_buttons = QHBoxLayout()
        odom_buttons.setSpacing(7)

        self.resume_button = QPushButton("Resume", parent=self)
        self.pause_button = QPushButton("PAUSE", parent=self)
        self.reset_button = QPushButton("RESET", parent=self)
        
        self.resume_button.pressed.connect(self._resume_odom)
        self.pause_button.pressed.connect(self._pause_odom)
        self.reset_button.pressed.connect(self._reset_odom)

        odom_buttons.addWidget(self.resume_button)
        odom_buttons.addWidget(self.pause_button)
        odom_buttons.addWidget(self.reset_button)

        self._root_layout.addLayout(odom_buttons)

    
    def _resume_odom(self):
        if self.status == self.Status.PAUSED:
            self.status = self.Status.UNKNOWN
            self.node.send_odom_resume_cmd()
            self.log_signal.emit("Odometry resumed")
            return
        self.log_signal.emit("Cannot resume odometry bacause it isn't paused")

    
    def _pause_odom(self):
        if self.status == self.Status.RUNNING:
            self.status = self.Status.PAUSED
            self.log_signal.emit("Pausing odometry")
            self.node.send_odom_pause_cmd()
            return
        self.log_signal.emit("Cannot pause odometry bacause it isn't runnig")
        
    
    def _reset_odom(self):
        if self.status != self.Status.PAUSED:
            self.status = self.Status.UNKNOWN
        self.node.send_odom_restart_cmd()
        self.log_signal.emit("Resetting odometry")
        
    

class SLAMFrame(SectionFrame):
    SLAM = "SLAM"
    Localization = "Localization"
    def __init__(self, parent=None, node: ElkapodControllerGui | None  = None):
        super().__init__("SLAM", parent, node)
        slam_buttons = QHBoxLayout()
        slam_buttons.setSpacing(7)

        self.resume_button = QPushButton("Resume", parent=self)
        self.pause_button = QPushButton("PAUSE", parent=self)
        self.reset_button = QPushButton("RESET", parent=self)
        
        self.resume_button.pressed.connect(self._resume_slam)
        self.pause_button.pressed.connect(self._pause_slam)
        self.reset_button.pressed.connect(self._reset_slam)

        slam_buttons.addWidget(self.resume_button)
        slam_buttons.addWidget(self.pause_button)
        slam_buttons.addWidget(self.reset_button)

        control_layout = QHBoxLayout()
        control_layout.setSpacing(7)
        node_mode_layout = QHBoxLayout()
        node_mode_layout.addWidget(QLabel("Mode:"))

        self.slam_mode = QComboBox()
        self.slam_mode.addItems([self.SLAM, self.Localization])
        self.slam_mode.currentTextChanged.connect(self._slam_mode_changed)

        node_mode_layout.addWidget(self.slam_mode)

        control_layout.addLayout(node_mode_layout)

        self.republish_map_button = QPushButton("Republish Map")
        self.new_map_button = QPushButton("New Map")
        self.republish_map_button.pressed.connect(self._republish_map)
        self.new_map_button.pressed.connect(self._new_map)
        control_layout.addWidget(self.republish_map_button)
        control_layout.addWidget(self.new_map_button)

        self._root_layout.addLayout(slam_buttons)
        self._root_layout.addSpacing(7)
        self._root_layout.addLayout(control_layout)
    
    def _resume_slam(self):
        if self.status == self.Status.PAUSED:
            self.status = self.Status.UNKNOWN
            self.node.send_slam_resume_cmd()
            self.log_signal.emit("Mapping resumed")
            return
        self.log_signal.emit("Cannot resume mapping bacause it isn't paused")
    
    def _pause_slam(self):
        if self.status == self.Status.RUNNING:
            self.status = self.Status.PAUSED
            self.node.send_slam_pause_cmd()
            self.log_signal.emit("Pausing mapping")
            return
        self.log_signal.emit("Cannot pause mapping bacause it isn't runnig")
    
    def _reset_slam(self):
        if self.status != self.Status.PAUSED:
            self.status = self.Status.UNKNOWN
        self.node.send_slam_restart_cmd()
        self.log_signal.emit("Deleting map")
    
    def _slam_mode_changed(self, mode: str):
        self.status = self.Status.UNKNOWN
        if mode == self.SLAM:
            self.node.send_slam_mapping_mode_cmd()
            self.log_signal.emit("Setting mode to SLAM")
        else:
            self.node.send_slam_localization_mode_cmd()
            self.log_signal.emit("Setting mode to Localization")

    def _republish_map(self):
        self.node.send_publish_map_cmd()
        self.log_signal.emit("Republishing map")
    
    def _new_map(self):
        self.node.send_new_map_cmd()
        self.log_signal.emit("Creating new map")



class NavigationFrame(SectionFrame):
    def __init__(self, parent=None, node: ElkapodControllerGui | None  = None):
        super().__init__("Navigation", parent, node)
        nav_buttons = QHBoxLayout()
        nav_buttons.setSpacing(7)
        self.abort_button = QPushButton("ABORT")
        self.surprise_button = QPushButton("SURPRISE")
        self.abort_button.pressed.connect(self._abort)
        self.surprise_button.pressed.connect(self._surprise)

        nav_buttons.addWidget(self.abort_button)
        nav_buttons.addWidget(self.surprise_button)
        self._root_layout.addLayout(nav_buttons)


    def _abort(self):
        print("Aborting task")

    def _surprise(self):
        print("Looking for a surprise")


class ResponseLoggingFrame(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setFrameShape(QFrame.Box)
        self.setLineWidth(1)

        root_layout = QVBoxLayout(self)

        label = QLabel("Activity", self)
        label.setStyleSheet("""
            font-size: 15pt;
            font-weight: bold;
        """)

        self.text_field = QPlainTextEdit(self)
        self.text_field.setReadOnly(True)

        lines = 15
        fm = QFontMetrics(self.text_field.font())
        line_height = fm.lineSpacing()
        margins = self.text_field.contentsMargins().top() + \
                  self.text_field.contentsMargins().bottom()
        self.text_field.setFixedHeight(lines * line_height + margins + 4)
        self.text_field.setLineWrapMode(QPlainTextEdit.NoWrap)
        font = QFont("Monospace")
        font.setStyleHint(QFont.Monospace)
        self.text_field.setFont(font)

        root_layout.addWidget(label)
        root_layout.addWidget(self.text_field)

        self.setSizePolicy(
            QSizePolicy.Policy.Preferred,
            QSizePolicy.Policy.Maximum
        )

    def append_message(self, text: str):
        doc = self.text_field.document()
        cursor = self.text_field.textCursor()

        cursor.movePosition(QTextCursor.Start)
        cursor.insertText(text + "\n")

        max_blocks = 25
        while doc.blockCount() > max_blocks:
            last_block = doc.lastBlock()
            cursor = QTextCursor(last_block)
            cursor.select(QTextCursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deletePreviousChar()

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

        self.odom_frame = OdometryFrame(central_widget,node)
        left_panel.addWidget(self.odom_frame)

        self.slam_frame = SLAMFrame(central_widget, node)
        left_panel.addWidget(self.slam_frame)

        self.nav_frame = NavigationFrame(central_widget, node)
        left_panel.addWidget(self.nav_frame)
        right_frame = QFrame()
        right_frame.setFrameShape(QFrame.Box)
        right_layout = QVBoxLayout(right_frame)

        self.logging_frame = ResponseLoggingFrame(central_widget)
        right_layout.addWidget(self.logging_frame)

        self.odom_frame.log_signal.connect(self.logging_frame.append_message)

        self.right_data = QLabel("")
        self.right_data.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(self.right_data)

        root_layout.addWidget(right_frame, 1)
        left_panel.addStretch()

        if node is not None:
            self.node.ros2_qt_bridge.odometry_received_signal.connect(self.odom_frame.update_data_box)
            self.node.ros2_qt_bridge.map_info_received_signal.connect(self.slam_frame.update_data_box)