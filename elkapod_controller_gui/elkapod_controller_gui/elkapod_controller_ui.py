# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'elkapod_controller2RTxmwx.ui'
##
## Created by: Qt User Interface Compiler version 6.4.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QDial, QDoubleSpinBox,
    QFrame, QHBoxLayout, QLabel, QMainWindow,
    QMenu, QMenuBar, QProgressBar, QPushButton,
    QSizePolicy, QSlider, QSpacerItem, QSpinBox,
    QTabWidget, QVBoxLayout, QWidget)

class Ui_HexapodController(object):
    def setupUi(self, HexapodController):
        if not HexapodController.objectName():
            HexapodController.setObjectName(u"HexapodController")
        HexapodController.resize(900, 800)
        icon = QIcon()
        iconThemeName = u"battery"
        if QIcon.hasThemeIcon(iconThemeName):
            icon = QIcon.fromTheme(iconThemeName)
        else:
            icon.addFile(u".", QSize(), QIcon.Normal, QIcon.Off)

        HexapodController.setWindowIcon(icon)
        self.actionAbout = QAction(HexapodController)
        self.actionAbout.setObjectName(u"actionAbout")
        self.central_widget = QWidget(HexapodController)
        self.central_widget.setObjectName(u"central_widget")
        self.verticalLayout_2 = QVBoxLayout(self.central_widget)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.tabWidget = QTabWidget(self.central_widget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.Init = QWidget()
        self.Init.setObjectName(u"Init")
        self.verticalLayout_14 = QVBoxLayout(self.Init)
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.verticalLayout_6 = QVBoxLayout()
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.horizontalLayout_8 = QHBoxLayout()
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.horizontalLayout_8.setContentsMargins(-1, 0, -1, 0)
        self.transition_status_title = QLabel(self.Init)
        self.transition_status_title.setObjectName(u"transition_status_title")

        self.horizontalLayout_8.addWidget(self.transition_status_title)

        self.transition_status_label = QLabel(self.Init)
        self.transition_status_label.setObjectName(u"transition_status_label")

        self.horizontalLayout_8.addWidget(self.transition_status_label)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_8.addItem(self.horizontalSpacer)


        self.verticalLayout_6.addLayout(self.horizontalLayout_8)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.horizontalLayout_7.setContentsMargins(-1, 10, -1, 10)
        self.init_transition_button = QPushButton(self.Init)
        self.init_transition_button.setObjectName(u"init_transition_button")
        self.init_transition_button.setMinimumSize(QSize(0, 40))

        self.horizontalLayout_7.addWidget(self.init_transition_button)

        self.idle_transition_button = QPushButton(self.Init)
        self.idle_transition_button.setObjectName(u"idle_transition_button")
        self.idle_transition_button.setEnabled(False)
        self.idle_transition_button.setMinimumSize(QSize(0, 40))

        self.horizontalLayout_7.addWidget(self.idle_transition_button)

        self.walk_transition_button = QPushButton(self.Init)
        self.walk_transition_button.setObjectName(u"walk_transition_button")
        self.walk_transition_button.setEnabled(False)
        self.walk_transition_button.setMinimumSize(QSize(0, 40))

        self.horizontalLayout_7.addWidget(self.walk_transition_button)


        self.verticalLayout_6.addLayout(self.horizontalLayout_7)


        self.verticalLayout_14.addLayout(self.verticalLayout_6)

        self.verticalSpacer_11 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_14.addItem(self.verticalSpacer_11)

        self.tabWidget.addTab(self.Init, "")
        self.Walk = QWidget()
        self.Walk.setObjectName(u"Walk")
        self.Walk.setEnabled(False)
        self.verticalLayout_16 = QVBoxLayout(self.Walk)
        self.verticalLayout_16.setObjectName(u"verticalLayout_16")
        self.verticalLayout_15 = QVBoxLayout()
        self.verticalLayout_15.setObjectName(u"verticalLayout_15")
        self.verticalWidget_2 = QWidget(self.Walk)
        self.verticalWidget_2.setObjectName(u"verticalWidget_2")
        palette = QPalette()
        brush = QBrush(QColor(60, 60, 60, 255))
        brush.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Window, brush)
        brush1 = QBrush(QColor(50, 50, 50, 255))
        brush1.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Inactive, QPalette.Window, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.Window, brush)
        self.verticalWidget_2.setPalette(palette)
        self.velocity_widget = QVBoxLayout(self.verticalWidget_2)
        self.velocity_widget.setObjectName(u"velocity_widget")
        self.velocity_widget.setContentsMargins(-1, 10, -1, 10)
        self.label_8 = QLabel(self.verticalWidget_2)
        self.label_8.setObjectName(u"label_8")
        font = QFont()
        font.setPointSize(18)
        self.label_8.setFont(font)
        self.label_8.setFrameShape(QFrame.NoFrame)
        self.label_8.setTextFormat(Qt.AutoText)

        self.velocity_widget.addWidget(self.label_8)

        self.velocity_control_widget = QWidget(self.verticalWidget_2)
        self.velocity_control_widget.setObjectName(u"velocity_control_widget")
        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.velocity_control_widget.sizePolicy().hasHeightForWidth())
        self.velocity_control_widget.setSizePolicy(sizePolicy)
        self.horizontalLayout_4 = QHBoxLayout(self.velocity_control_widget)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.vdir_widget = QWidget(self.velocity_control_widget)
        self.vdir_widget.setObjectName(u"vdir_widget")
        self.vdir_widget.setAutoFillBackground(True)
        self.velocity_direction = QVBoxLayout(self.vdir_widget)
        self.velocity_direction.setObjectName(u"velocity_direction")
        self.velocity_direction.setContentsMargins(10, -1, 10, -1)
        self.velocity_dir_title = QLabel(self.vdir_widget)
        self.velocity_dir_title.setObjectName(u"velocity_dir_title")
        self.velocity_dir_title.setMinimumSize(QSize(0, 40))
        self.velocity_dir_title.setAlignment(Qt.AlignCenter)

        self.velocity_direction.addWidget(self.velocity_dir_title)

        self.verticalSpacer_6 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.velocity_direction.addItem(self.verticalSpacer_6)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.vdir_dial = QDial(self.vdir_widget)
        self.vdir_dial.setObjectName(u"vdir_dial")
        self.vdir_dial.setMinimumSize(QSize(80, 80))
        self.vdir_dial.setMaximumSize(QSize(80, 16777215))
        self.vdir_dial.setMaximum(360)
        self.vdir_dial.setSingleStep(3)
        self.vdir_dial.setValue(180)
        self.vdir_dial.setWrapping(True)
        self.vdir_dial.setNotchesVisible(True)

        self.horizontalLayout_2.addWidget(self.vdir_dial)

        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.label = QLabel(self.vdir_widget)
        self.label.setObjectName(u"label")
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setAlignment(Qt.AlignCenter)

        self.verticalLayout_5.addWidget(self.label)

        self.vdir_spinbox = QSpinBox(self.vdir_widget)
        self.vdir_spinbox.setObjectName(u"vdir_spinbox")
        self.vdir_spinbox.setMinimum(-180)
        self.vdir_spinbox.setMaximum(180)
        self.vdir_spinbox.setValue(0)

        self.verticalLayout_5.addWidget(self.vdir_spinbox)


        self.horizontalLayout_2.addLayout(self.verticalLayout_5)


        self.velocity_direction.addLayout(self.horizontalLayout_2)

        self.verticalSpacer_5 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.velocity_direction.addItem(self.verticalSpacer_5)


        self.horizontalLayout_4.addWidget(self.vdir_widget)

        self.vval_widget = QWidget(self.velocity_control_widget)
        self.vval_widget.setObjectName(u"vval_widget")
        self.vval_widget.setAutoFillBackground(True)
        self.velocity_value = QVBoxLayout(self.vval_widget)
        self.velocity_value.setObjectName(u"velocity_value")
        self.velocity_value_title = QLabel(self.vval_widget)
        self.velocity_value_title.setObjectName(u"velocity_value_title")
        self.velocity_value_title.setMinimumSize(QSize(0, 40))
        self.velocity_value_title.setAlignment(Qt.AlignCenter)

        self.velocity_value.addWidget(self.velocity_value_title)

        self.verticalSpacer_7 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.velocity_value.addItem(self.verticalSpacer_7)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.vval_slider = QSlider(self.vval_widget)
        self.vval_slider.setObjectName(u"vval_slider")
        sizePolicy1 = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.vval_slider.sizePolicy().hasHeightForWidth())
        self.vval_slider.setSizePolicy(sizePolicy1)
        self.vval_slider.setMinimumSize(QSize(0, 180))
        self.vval_slider.setBaseSize(QSize(0, 0))
        self.vval_slider.setMaximum(100)
        self.vval_slider.setOrientation(Qt.Vertical)
        self.vval_slider.setInvertedAppearance(False)
        self.vval_slider.setTickPosition(QSlider.TicksBelow)

        self.horizontalLayout.addWidget(self.vval_slider)

        self.verticalLayout_13 = QVBoxLayout()
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.vval_spinbox = QDoubleSpinBox(self.vval_widget)
        self.vval_spinbox.setObjectName(u"vval_spinbox")
        self.vval_spinbox.setDecimals(3)
        self.vval_spinbox.setMaximum(0.200000000000000)
        self.vval_spinbox.setSingleStep(0.001000000000000)

        self.verticalLayout_13.addWidget(self.vval_spinbox)

        self.vval_stop_button = QPushButton(self.vval_widget)
        self.vval_stop_button.setObjectName(u"vval_stop_button")

        self.verticalLayout_13.addWidget(self.vval_stop_button)


        self.horizontalLayout.addLayout(self.verticalLayout_13)


        self.velocity_value.addLayout(self.horizontalLayout)

        self.verticalSpacer_8 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.velocity_value.addItem(self.verticalSpacer_8)


        self.horizontalLayout_4.addWidget(self.vval_widget)

        self.angular_vel_widget = QWidget(self.velocity_control_widget)
        self.angular_vel_widget.setObjectName(u"angular_vel_widget")
        self.angular_vel_widget.setAutoFillBackground(True)
        self.angular_velocity = QVBoxLayout(self.angular_vel_widget)
        self.angular_velocity.setObjectName(u"angular_velocity")
        self.angular_vel_title = QLabel(self.angular_vel_widget)
        self.angular_vel_title.setObjectName(u"angular_vel_title")
        self.angular_vel_title.setMinimumSize(QSize(0, 40))
        self.angular_vel_title.setAlignment(Qt.AlignCenter)

        self.angular_velocity.addWidget(self.angular_vel_title)

        self.verticalSpacer_9 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.angular_velocity.addItem(self.verticalSpacer_9)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.angular_vel_slider = QSlider(self.angular_vel_widget)
        self.angular_vel_slider.setObjectName(u"angular_vel_slider")
        self.angular_vel_slider.setMinimumSize(QSize(0, 180))
        self.angular_vel_slider.setMinimum(0)
        self.angular_vel_slider.setMaximum(100)
        self.angular_vel_slider.setValue(50)
        self.angular_vel_slider.setOrientation(Qt.Vertical)
        self.angular_vel_slider.setTickPosition(QSlider.TicksBelow)

        self.horizontalLayout_11.addWidget(self.angular_vel_slider)

        self.verticalLayout_9 = QVBoxLayout()
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.angular_vel_spinbox = QDoubleSpinBox(self.angular_vel_widget)
        self.angular_vel_spinbox.setObjectName(u"angular_vel_spinbox")
        self.angular_vel_spinbox.setDecimals(3)
        self.angular_vel_spinbox.setMinimum(-0.400000000000000)
        self.angular_vel_spinbox.setMaximum(0.400000000000000)
        self.angular_vel_spinbox.setSingleStep(0.001000000000000)

        self.verticalLayout_9.addWidget(self.angular_vel_spinbox)

        self.angular_vel_stopbutton = QPushButton(self.angular_vel_widget)
        self.angular_vel_stopbutton.setObjectName(u"angular_vel_stopbutton")

        self.verticalLayout_9.addWidget(self.angular_vel_stopbutton)


        self.horizontalLayout_11.addLayout(self.verticalLayout_9)


        self.angular_velocity.addLayout(self.horizontalLayout_11)

        self.verticalSpacer_12 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.angular_velocity.addItem(self.verticalSpacer_12)


        self.horizontalLayout_4.addWidget(self.angular_vel_widget)

        self.side_widget = QWidget(self.velocity_control_widget)
        self.side_widget.setObjectName(u"side_widget")
        sizePolicy2 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Preferred)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.side_widget.sizePolicy().hasHeightForWidth())
        self.side_widget.setSizePolicy(sizePolicy2)
        self.side_widget.setAutoFillBackground(False)
        self.verticalLayout_3 = QVBoxLayout(self.side_widget)
        self.verticalLayout_3.setSpacing(4)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.battery_lvl_widget_2 = QWidget(self.side_widget)
        self.battery_lvl_widget_2.setObjectName(u"battery_lvl_widget_2")
        self.battery_lvl_widget_2.setAutoFillBackground(True)
        self.battery_lvl_widget = QVBoxLayout(self.battery_lvl_widget_2)
        self.battery_lvl_widget.setObjectName(u"battery_lvl_widget")
        self.verticalSpacer_13 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.battery_lvl_widget.addItem(self.verticalSpacer_13)

        self.label_2 = QLabel(self.battery_lvl_widget_2)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setMinimumSize(QSize(0, 40))
        self.label_2.setAlignment(Qt.AlignCenter)

        self.battery_lvl_widget.addWidget(self.label_2)

        self.progressBar = QProgressBar(self.battery_lvl_widget_2)
        self.progressBar.setObjectName(u"progressBar")
        self.progressBar.setValue(0)

        self.battery_lvl_widget.addWidget(self.progressBar)

        self.verticalSpacer_4 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.battery_lvl_widget.addItem(self.verticalSpacer_4)


        self.verticalLayout_3.addWidget(self.battery_lvl_widget_2)

        self.gait_type_widget_2 = QWidget(self.side_widget)
        self.gait_type_widget_2.setObjectName(u"gait_type_widget_2")
        self.gait_type_widget_2.setAutoFillBackground(True)
        self.gait_type_widget = QVBoxLayout(self.gait_type_widget_2)
        self.gait_type_widget.setObjectName(u"gait_type_widget")
        self.verticalSpacer_14 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.gait_type_widget.addItem(self.verticalSpacer_14)

        self.label_7 = QLabel(self.gait_type_widget_2)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setMinimumSize(QSize(0, 40))
        self.label_7.setAlignment(Qt.AlignCenter)

        self.gait_type_widget.addWidget(self.label_7)

        self.gait_selection = QComboBox(self.gait_type_widget_2)
        self.gait_selection.setObjectName(u"gait_selection")

        self.gait_type_widget.addWidget(self.gait_selection)

        self.verticalSpacer_15 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.gait_type_widget.addItem(self.verticalSpacer_15)


        self.verticalLayout_3.addWidget(self.gait_type_widget_2)


        self.horizontalLayout_4.addWidget(self.side_widget)

        self.horizontalLayout_4.setStretch(0, 3)
        self.horizontalLayout_4.setStretch(1, 3)
        self.horizontalLayout_4.setStretch(2, 3)
        self.horizontalLayout_4.setStretch(3, 1)

        self.velocity_widget.addWidget(self.velocity_control_widget)

        self.velocity_title = QLabel(self.verticalWidget_2)
        self.velocity_title.setObjectName(u"velocity_title")
        palette1 = QPalette()
        brush2 = QBrush(QColor(229, 165, 10, 255))
        brush2.setStyle(Qt.SolidPattern)
        palette1.setBrush(QPalette.Active, QPalette.Base, brush2)
        brush3 = QBrush(QColor(36, 36, 36, 255))
        brush3.setStyle(Qt.SolidPattern)
        palette1.setBrush(QPalette.Inactive, QPalette.Base, brush3)
        palette1.setBrush(QPalette.Disabled, QPalette.Base, brush1)
        self.velocity_title.setPalette(palette1)
        self.velocity_title.setFont(font)

        self.velocity_widget.addWidget(self.velocity_title)

        self.horizontalWidget_7 = QWidget(self.verticalWidget_2)
        self.horizontalWidget_7.setObjectName(u"horizontalWidget_7")
        palette2 = QPalette()
        palette2.setBrush(QPalette.Active, QPalette.Window, brush)
        palette2.setBrush(QPalette.Inactive, QPalette.Window, brush1)
        palette2.setBrush(QPalette.Disabled, QPalette.Window, brush)
        self.horizontalWidget_7.setPalette(palette2)
        self.horizontalLayout_12 = QHBoxLayout(self.horizontalWidget_7)
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.horizontalLayout_12.setContentsMargins(-1, 10, -1, 10)
        self.verticalWidget_4 = QWidget(self.horizontalWidget_7)
        self.verticalWidget_4.setObjectName(u"verticalWidget_4")
        self.verticalWidget_4.setAutoFillBackground(True)
        self.verticalLayout_10 = QVBoxLayout(self.verticalWidget_4)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.label_3 = QLabel(self.verticalWidget_4)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setMinimumSize(QSize(0, 40))
        self.label_3.setAlignment(Qt.AlignCenter)

        self.verticalLayout_10.addWidget(self.label_3)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_10.addItem(self.verticalSpacer)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.roll_slider = QSlider(self.verticalWidget_4)
        self.roll_slider.setObjectName(u"roll_slider")
        self.roll_slider.setMinimumSize(QSize(0, 180))
        self.roll_slider.setMaximum(100)
        self.roll_slider.setValue(50)
        self.roll_slider.setOrientation(Qt.Vertical)

        self.horizontalLayout_3.addWidget(self.roll_slider)

        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.roll_spinbox = QDoubleSpinBox(self.verticalWidget_4)
        self.roll_spinbox.setObjectName(u"roll_spinbox")
        self.roll_spinbox.setDecimals(3)
        self.roll_spinbox.setMinimum(-3.000000000000000)
        self.roll_spinbox.setMaximum(3.000000000000000)
        self.roll_spinbox.setSingleStep(0.001000000000000)

        self.verticalLayout.addWidget(self.roll_spinbox)

        self.roll_reset_button = QPushButton(self.verticalWidget_4)
        self.roll_reset_button.setObjectName(u"roll_reset_button")

        self.verticalLayout.addWidget(self.roll_reset_button)


        self.horizontalLayout_3.addLayout(self.verticalLayout)


        self.verticalLayout_10.addLayout(self.horizontalLayout_3)


        self.horizontalLayout_12.addWidget(self.verticalWidget_4)

        self.verticalWidget_5 = QWidget(self.horizontalWidget_7)
        self.verticalWidget_5.setObjectName(u"verticalWidget_5")
        self.verticalWidget_5.setAutoFillBackground(True)
        self.verticalLayout_7 = QVBoxLayout(self.verticalWidget_5)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.label_4 = QLabel(self.verticalWidget_5)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setMinimumSize(QSize(0, 40))
        self.label_4.setAlignment(Qt.AlignCenter)

        self.verticalLayout_7.addWidget(self.label_4)

        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_7.addItem(self.verticalSpacer_2)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.pitch_slider = QSlider(self.verticalWidget_5)
        self.pitch_slider.setObjectName(u"pitch_slider")
        self.pitch_slider.setMinimumSize(QSize(0, 180))
        self.pitch_slider.setMaximum(100)
        self.pitch_slider.setValue(50)
        self.pitch_slider.setOrientation(Qt.Vertical)

        self.horizontalLayout_5.addWidget(self.pitch_slider)

        self.verticalLayout_8 = QVBoxLayout()
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.pitch_spinbox = QDoubleSpinBox(self.verticalWidget_5)
        self.pitch_spinbox.setObjectName(u"pitch_spinbox")
        self.pitch_spinbox.setDecimals(3)
        self.pitch_spinbox.setMinimum(-6.000000000000000)
        self.pitch_spinbox.setMaximum(6.000000000000000)
        self.pitch_spinbox.setSingleStep(0.001000000000000)

        self.verticalLayout_8.addWidget(self.pitch_spinbox)

        self.pitch_reset_button = QPushButton(self.verticalWidget_5)
        self.pitch_reset_button.setObjectName(u"pitch_reset_button")

        self.verticalLayout_8.addWidget(self.pitch_reset_button)


        self.horizontalLayout_5.addLayout(self.verticalLayout_8)


        self.verticalLayout_7.addLayout(self.horizontalLayout_5)


        self.horizontalLayout_12.addWidget(self.verticalWidget_5)

        self.verticalWidget_21 = QWidget(self.horizontalWidget_7)
        self.verticalWidget_21.setObjectName(u"verticalWidget_21")
        self.verticalWidget_21.setAutoFillBackground(True)
        self.verticalLayout_4 = QVBoxLayout(self.verticalWidget_21)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.label_5 = QLabel(self.verticalWidget_21)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setMinimumSize(QSize(0, 40))
        self.label_5.setAlignment(Qt.AlignCenter)

        self.verticalLayout_4.addWidget(self.label_5)

        self.verticalSpacer_3 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_4.addItem(self.verticalSpacer_3)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.base_height_slider = QSlider(self.verticalWidget_21)
        self.base_height_slider.setObjectName(u"base_height_slider")
        self.base_height_slider.setMinimumSize(QSize(0, 180))
        self.base_height_slider.setMinimum(0)
        self.base_height_slider.setMaximum(100)
        self.base_height_slider.setValue(43)
        self.base_height_slider.setSliderPosition(43)
        self.base_height_slider.setOrientation(Qt.Vertical)

        self.horizontalLayout_6.addWidget(self.base_height_slider)

        self.verticalLayout_11 = QVBoxLayout()
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.base_height_spinbox = QDoubleSpinBox(self.verticalWidget_21)
        self.base_height_spinbox.setObjectName(u"base_height_spinbox")
        self.base_height_spinbox.setDecimals(3)
        self.base_height_spinbox.setMinimum(0.090000000000000)
        self.base_height_spinbox.setMaximum(0.160000000000000)
        self.base_height_spinbox.setSingleStep(0.001000000000000)
        self.base_height_spinbox.setValue(0.120000000000000)

        self.verticalLayout_11.addWidget(self.base_height_spinbox)

        self.base_height_default_button = QPushButton(self.verticalWidget_21)
        self.base_height_default_button.setObjectName(u"base_height_default_button")

        self.verticalLayout_11.addWidget(self.base_height_default_button)


        self.horizontalLayout_6.addLayout(self.verticalLayout_11)


        self.verticalLayout_4.addLayout(self.horizontalLayout_6)


        self.horizontalLayout_12.addWidget(self.verticalWidget_21)


        self.velocity_widget.addWidget(self.horizontalWidget_7)


        self.verticalLayout_15.addWidget(self.verticalWidget_2)


        self.verticalLayout_16.addLayout(self.verticalLayout_15)

        self.tabWidget.addTab(self.Walk, "")

        self.verticalLayout_2.addWidget(self.tabWidget)

        self.verticalSpacer_10 = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer_10)

        HexapodController.setCentralWidget(self.central_widget)
        self.menuBar = QMenuBar(HexapodController)
        self.menuBar.setObjectName(u"menuBar")
        self.menuBar.setGeometry(QRect(0, 0, 900, 23))
        self.menuAbout = QMenu(self.menuBar)
        self.menuAbout.setObjectName(u"menuAbout")
        HexapodController.setMenuBar(self.menuBar)

        self.menuBar.addAction(self.menuAbout.menuAction())
        self.menuAbout.addAction(self.actionAbout)

        self.retranslateUi(HexapodController)

        self.tabWidget.setCurrentIndex(0)
        self.gait_selection.setCurrentIndex(-1)


        QMetaObject.connectSlotsByName(HexapodController)
    # setupUi

    def retranslateUi(self, HexapodController):
        HexapodController.setWindowTitle(QCoreApplication.translate("HexapodController", u"Elkapod Control ", None))
        self.actionAbout.setText(QCoreApplication.translate("HexapodController", u"About", None))
        self.transition_status_title.setText(QCoreApplication.translate("HexapodController", u"Transition status:", None))
        self.transition_status_label.setText(QCoreApplication.translate("HexapodController", u"none", None))
        self.init_transition_button.setText(QCoreApplication.translate("HexapodController", u"Init", None))
        self.idle_transition_button.setText(QCoreApplication.translate("HexapodController", u"Idle", None))
        self.walk_transition_button.setText(QCoreApplication.translate("HexapodController", u"Walk", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.Init), QCoreApplication.translate("HexapodController", u"Init", None))
        self.label_8.setText(QCoreApplication.translate("HexapodController", u"Walk control", None))
        self.velocity_dir_title.setText(QCoreApplication.translate("HexapodController", u"Linear velocity direction", None))
        self.label.setText(QCoreApplication.translate("HexapodController", u"Angle (rad)", None))
        self.velocity_value_title.setText(QCoreApplication.translate("HexapodController", u"Linear velocity value", None))
        self.vval_stop_button.setText(QCoreApplication.translate("HexapodController", u"Stop", None))
        self.angular_vel_title.setText(QCoreApplication.translate("HexapodController", u"Angular velocity", None))
        self.angular_vel_stopbutton.setText(QCoreApplication.translate("HexapodController", u"Stop", None))
        self.label_2.setText(QCoreApplication.translate("HexapodController", u"Battery level", None))
        self.label_7.setText(QCoreApplication.translate("HexapodController", u"Gait", None))
        self.gait_selection.setCurrentText("")
        self.velocity_title.setText(QCoreApplication.translate("HexapodController", u"Body control", None))
        self.label_3.setText(QCoreApplication.translate("HexapodController", u"Roll control", None))
        self.roll_reset_button.setText(QCoreApplication.translate("HexapodController", u"Reset", None))
        self.label_4.setText(QCoreApplication.translate("HexapodController", u"Pitch control", None))
        self.pitch_reset_button.setText(QCoreApplication.translate("HexapodController", u"Reset", None))
        self.label_5.setText(QCoreApplication.translate("HexapodController", u"Base height", None))
        self.base_height_default_button.setText(QCoreApplication.translate("HexapodController", u"Default", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.Walk), QCoreApplication.translate("HexapodController", u"Walk", None))
        self.menuAbout.setTitle(QCoreApplication.translate("HexapodController", u"Help", None))
    # retranslateUi

