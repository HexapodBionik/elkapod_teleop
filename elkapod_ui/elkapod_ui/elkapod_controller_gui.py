import math

from PySide6.QtWidgets import QMainWindow, QMessageBox

from .elkapod_controller_ui import Ui_HexapodController
from .elkapod_gui_node import ElkapodControllerGui, GaitType, RobotState, SpeedCommand


class ApplicationMainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._ui = Ui_HexapodController()
        self._ui.setupUi(self)
        self.node: ElkapodControllerGui = None

    def setup(self):
        self._ui.vdir_dial.sliderMoved.connect(self._update_vdir_dial)
        self._ui.vdir_spinbox.valueChanged.connect(self._update_vdir_spinbox)

        self._ui.vval_slider.sliderMoved.connect(self._update_vval_slider)
        self._ui.vval_spinbox.valueChanged.connect(self._update_vval_spinbox)
        self._ui.vval_stop_button.pressed.connect(self._update_vval_button)

        self._ui.init_transition_button.pressed.connect(self._send_init_transition)
        self._ui.idle_transition_button.pressed.connect(self._send_idle_transition)
        self._ui.walk_transition_button.pressed.connect(self._send_walk_transition)

        self.node.ros2_qt_bridge.send_async_cmd_signal.connect(self._on_transition_result)

        self.node.ros2_qt_bridge.send_state_signal.connect(self._update_button_availability)

        self.node.ros2_qt_bridge.send_battery_lvl_signal.connect(self._update_battery_level)

        self._ui.angular_vel_slider.sliderMoved.connect(self._update_angular_vel_slider)
        self._ui.angular_vel_spinbox.valueChanged.connect(self._update_angular_vel_spinbox)
        self._ui.angular_vel_stopbutton.pressed.connect(self._update_angular_vel_button)

        self._ui.roll_slider.sliderMoved.connect(self._update_roll_slider)
        self._ui.roll_spinbox.valueChanged.connect(self._update_roll_spinbox)
        self._ui.roll_reset_button.pressed.connect(self._update_roll_button)

        self._ui.pitch_slider.sliderMoved.connect(self._update_pitch_slider)
        self._ui.pitch_spinbox.valueChanged.connect(self._update_pitch_spinbox)
        self._ui.pitch_reset_button.pressed.connect(self._update_pitch_button)

        self._ui.base_height_slider.sliderMoved.connect(self._update_base_height_slider)
        self._ui.base_height_spinbox.valueChanged.connect(self._update_base_height_spinbox)
        self._ui.base_height_default_button.pressed.connect(self._update_base_height_button)

        self._ui.actionAbout.triggered.connect(self._show_about_message)

        self._ui.gait_selection.addItems(
            [
                "TRIPOD",
                "WAVE",
                "RIPPLE",
            ]
        )
        self._ui.gait_selection.currentTextChanged.connect(self._update_gait)

        self._robot_state = RobotState.INIT

        self._speed = SpeedCommand()
        self._current_max_speed = 0.2

    def _send_angular_vel_command(self, omega: float):
        self._ui.vval_slider.setValue(0.0)
        self._ui.vval_spinbox.setValue(0.0)

        self._speed.vx = 0.0
        self._speed.vy = 0.0
        self._speed.omega = omega
        self.node.send_vel_command(self._speed)

    def _update_angular_vel_slider(self, omega: str):
        omega = -0.5 + float(omega) / 100.0
        self._ui.angular_vel_spinbox.setValue(omega)
        self._send_angular_vel_command(omega)

    def _update_angular_vel_spinbox(self, omega: str):
        omega = float(omega)
        slider_value = (omega + 0.5) * 100.0
        self._ui.angular_vel_slider.setValue(slider_value)
        self._send_angular_vel_command(omega)

    def _update_angular_vel_button(self):
        self._ui.angular_vel_spinbox.setValue(0.0)
        self._ui.angular_vel_slider.setValue(50)
        self._send_angular_vel_command(0.0)

    def _update_roll_slider(self, roll: str):
        roll_deg = -3.0 + (float(roll) / 100.0) * 6.0
        roll_rad = roll_deg * math.pi / 180.0
        self._ui.roll_spinbox.setValue(roll_deg)
        self.node.send_roll_command(roll_rad)

    def _update_roll_spinbox(self, roll: str):
        roll_deg = float(roll)
        roll_rad = roll_deg * math.pi / 180.0
        slider_value = (roll_deg + 3.0) / 6.0 * 100.0
        self._ui.roll_slider.setValue(slider_value)
        self.node.send_roll_command(roll_rad)

    def _update_roll_button(self):
        self._ui.roll_spinbox.setValue(0.0)
        self._ui.roll_slider.setValue(50)
        self.node.send_roll_command(0.0)

    def _update_pitch_slider(self, pitch: str):
        pitch_deg = -6.0 + (float(pitch) / 100.0) * 12.0
        pitch_rad = pitch_deg * math.pi / 180.0
        self._ui.pitch_spinbox.setValue(pitch_deg)
        self.node.send_pitch_command(pitch_rad)

    def _update_pitch_spinbox(self, pitch: str):
        pitch_deg = float(pitch)
        pitch_rad = pitch_deg * math.pi / 180.0
        slider_value = (pitch_deg + 6.0) / 12.0 * 100.0
        self._ui.pitch_slider.setValue(slider_value)
        self.node.send_pitch_command(pitch_rad)

    def _update_pitch_button(self):
        self._ui.pitch_spinbox.setValue(0.0)
        self._ui.pitch_slider.setValue(50)
        self.node.send_pitch_command(0.0)

    def _update_base_height_slider(self, height: str):
        base_height = 0.09 + (float(height) / 100.0) * 0.07
        self._ui.base_height_spinbox.setValue(base_height)
        self.node.send_base_height_command(base_height)

    def _update_base_height_spinbox(self, height: str):
        base_height = float(height)
        slider_value = (base_height - 0.09) / 0.07 * 100.0
        self._ui.base_height_slider.setValue(slider_value)
        self.node.send_base_height_command(base_height)

    def _update_base_height_button(self):
        self._ui.base_height_slider.setValue(43)
        self._ui.base_height_spinbox.setValue(0.12)
        self.node.send_base_height_command(0.12)

    def _send_idle_transition(self):
        self._ui.transition_status_label.setText("started")
        self._ui.transition_status_label.setStyleSheet("color: #FBEC5D")

        if self._robot_state == RobotState.IDLE_LOWERED:
            self.node.send_motion_manager_transition("stand_up")
        elif self._robot_state == RobotState.WALKING:
            self.node.send_walk_disable_cmd()

    def _send_walk_transition(self):
        self._ui.transition_status_label.setText("started")
        self._ui.transition_status_label.setStyleSheet("color: #FBEC5D")

        self.node.send_walk_enable_cmd()

    def _send_init_transition(self):
        self._ui.transition_status_label.setText("started")
        self._ui.transition_status_label.setStyleSheet("color: #FBEC5D")

        if self._robot_state == RobotState.INIT:
            self.node.send_motion_manager_transition("init")
        elif self._robot_state == RobotState.IDLE:
            self.node.send_motion_manager_transition("lower")

    def _update_battery_level(self, batery_level: float):
        batery_level = min(max(batery_level * 100.0, 0.0), 100.0)
        self._ui.progressBar.setValue(batery_level)

    def _update_button_availability(self, state: RobotState):
        self._robot_state = state
        match state:
            case RobotState.INIT:
                self._ui.idle_transition_button.setDisabled(True)
                self._ui.init_transition_button.setEnabled(True)
                self._ui.walk_transition_button.setDisabled(True)
                self._ui.Walk.setDisabled(True)
            case RobotState.IDLE_LOWERED:
                self._ui.idle_transition_button.setEnabled(True)
                self._ui.init_transition_button.setDisabled(True)
                self._ui.walk_transition_button.setDisabled(True)
                self._ui.Walk.setDisabled(True)
            case RobotState.IDLE:
                self._ui.walk_transition_button.setEnabled(True)
                self._ui.init_transition_button.setEnabled(True)
                self._ui.idle_transition_button.setDisabled(True)
                self._ui.Walk.setDisabled(True)
            case RobotState.WALKING:
                self._ui.idle_transition_button.setEnabled(True)
                self._ui.walk_transition_button.setDisabled(True)
                self._ui.init_transition_button.setDisabled(True)
                self._ui.Walk.setEnabled(True)

    def _on_transition_result(self, result: bool):
        if result:
            self._ui.transition_status_label.setText("success")
            self._ui.transition_status_label.setStyleSheet("color: #88E788")
        else:
            self._ui.transition_status_label.setText("failed")
            self._ui.transition_status_label.setStyleSheet("color: #DA2C43")

    def _update_vval_slider(self, vval):
        value = float(vval) / 100.0 * self._current_max_speed
        self._ui.vval_spinbox.setValue(value)
        self._update_vval(value)

    def _update_vval_spinbox(self, vval):
        slider_value = min(round(float(vval) / self._current_max_speed * 100.0), 100)
        self._ui.vval_slider.setValue(slider_value)
        self._update_vval(float(vval))

    def _update_vval_button(self):
        self._ui.vval_slider.setValue(0.0)
        self._ui.vval_spinbox.setValue(0.0)
        self._update_vval(0.0)

    def _update_vval(self, vval):
        vval = float(vval)
        vdir = 0.0
        if self._speed.norm() > 0.0:
            vdir = math.atan2(self._speed.vy, self._speed.vx)

        self._speed.vx = math.cos(vdir) * vval
        self._speed.vy = math.sin(vdir) * vval
        self.node.send_vel_command(self._speed)

    def _update_vdir_dial(self, vdir):
        vdir = -float(vdir) + 180.0
        self._ui.vdir_spinbox.setValue(vdir)
        self.update_vdir(vdir)

    def _update_vdir_spinbox(self, vdir):
        vdir = float(vdir)
        self.update_vdir(vdir)
        self._ui.vdir_dial.setValue(-vdir + 180.0)

    def update_vdir(self, vdir):
        vdir = vdir * math.pi / 180.0
        vval = self._speed.norm()

        self._speed.vx = math.cos(vdir) * vval
        self._speed.vy = math.sin(vdir) * vval
        self.node.send_vel_command(self._speed)

    def _update_gait(self, gait: str):
        match gait:
            case "TRIPOD":
                gait_cmd = GaitType.TRIPOD
                self._current_max_speed = 0.2
            case "WAVE":
                gait_cmd = GaitType.WAVE
                self._current_max_speed = 0.05
            case "RIPPLE":
                gait_cmd = GaitType.RIPPLE
                self._current_max_speed = 0.15
            case _:
                gait_cmd = GaitType.TRIPOD
        self._ui.vval_spinbox.setMaximum(self._current_max_speed)
        vval_clamped = min(self._current_max_speed, self._speed.norm())
        self._update_vval(vval_clamped)
        self.node.send_gait_type_command(gait_cmd)

    def _show_about_message(self):
        QMessageBox.information(
            self,
            "Elkapod Control GUI",
            "Version: 1.0.0\nCopyright (c) 2025 Elkapod Bionik, WUT",
        )
