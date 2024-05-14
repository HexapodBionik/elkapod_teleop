# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'elkapod_controller.ui'
##
## Created by: Qt User Interface Compiler version 6.6.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDial,
    QGraphicsView, QHBoxLayout, QLabel, QMainWindow,
    QSizePolicy, QSlider, QSpacerItem, QVBoxLayout,
    QWidget)

class Ui_HexapodController(object):
    def setupUi(self, HexapodController):
        if not HexapodController.objectName():
            HexapodController.setObjectName(u"HexapodController")
        HexapodController.resize(564, 637)
        self.central_widget = QWidget(HexapodController)
        self.central_widget.setObjectName(u"central_widget")
        self.verticalLayout_2 = QVBoxLayout(self.central_widget)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.widget_13 = QWidget(self.central_widget)
        self.widget_13.setObjectName(u"widget_13")
        self.horizontalLayout_8 = QHBoxLayout(self.widget_13)
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.horizontalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.widget_12 = QWidget(self.widget_13)
        self.widget_12.setObjectName(u"widget_12")
        self.verticalLayout_9 = QVBoxLayout(self.widget_12)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.verticalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.leg_spacing_widget = QWidget(self.widget_12)
        self.leg_spacing_widget.setObjectName(u"leg_spacing_widget")
        self.leg_spacing_widget.setEnabled(True)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.leg_spacing_widget.sizePolicy().hasHeightForWidth())
        self.leg_spacing_widget.setSizePolicy(sizePolicy)
        self.horizontalLayout_2 = QHBoxLayout(self.leg_spacing_widget)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.leg_spacing_view = QGraphicsView(self.leg_spacing_widget)
        self.leg_spacing_view.setObjectName(u"leg_spacing_view")
        self.leg_spacing_view.setEnabled(True)
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.leg_spacing_view.sizePolicy().hasHeightForWidth())
        self.leg_spacing_view.setSizePolicy(sizePolicy1)
        self.leg_spacing_view.setMinimumSize(QSize(220, 80))
        self.leg_spacing_view.setMaximumSize(QSize(220, 80))

        self.horizontalLayout_2.addWidget(self.leg_spacing_view)

        self.widget_17 = QWidget(self.leg_spacing_widget)
        self.widget_17.setObjectName(u"widget_17")
        self.verticalLayout_12 = QVBoxLayout(self.widget_17)
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.verticalLayout_12.setContentsMargins(0, 0, 0, 0)
        self.verticalSpacer_6 = QSpacerItem(20, 5, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_12.addItem(self.verticalSpacer_6)

        self.leg_spacing_slider = QSlider(self.widget_17)
        self.leg_spacing_slider.setObjectName(u"leg_spacing_slider")
        self.leg_spacing_slider.setEnabled(True)
        self.leg_spacing_slider.setMouseTracking(True)
        self.leg_spacing_slider.setMinimum(40)
        self.leg_spacing_slider.setMaximum(200)
        self.leg_spacing_slider.setValue(120)
        self.leg_spacing_slider.setOrientation(Qt.Horizontal)

        self.verticalLayout_12.addWidget(self.leg_spacing_slider)

        self.widget_18 = QWidget(self.widget_17)
        self.widget_18.setObjectName(u"widget_18")
        self.horizontalLayout_10 = QHBoxLayout(self.widget_18)
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.horizontalLayout_10.setContentsMargins(0, 0, 0, 0)
        self.label_14 = QLabel(self.widget_18)
        self.label_14.setObjectName(u"label_14")

        self.horizontalLayout_10.addWidget(self.label_14)

        self.horizontalSpacer_4 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_10.addItem(self.horizontalSpacer_4)

        self.label_15 = QLabel(self.widget_18)
        self.label_15.setObjectName(u"label_15")

        self.horizontalLayout_10.addWidget(self.label_15)

        self.horizontalSpacer_5 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_10.addItem(self.horizontalSpacer_5)

        self.label_13 = QLabel(self.widget_18)
        self.label_13.setObjectName(u"label_13")

        self.horizontalLayout_10.addWidget(self.label_13)


        self.verticalLayout_12.addWidget(self.widget_18)

        self.verticalSpacer_7 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_12.addItem(self.verticalSpacer_7)


        self.horizontalLayout_2.addWidget(self.widget_17)


        self.verticalLayout_9.addWidget(self.leg_spacing_widget)

        self.height_widget = QWidget(self.widget_12)
        self.height_widget.setObjectName(u"height_widget")
        sizePolicy.setHeightForWidth(self.height_widget.sizePolicy().hasHeightForWidth())
        self.height_widget.setSizePolicy(sizePolicy)
        self.horizontalLayout = QHBoxLayout(self.height_widget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.height_view = QGraphicsView(self.height_widget)
        self.height_view.setObjectName(u"height_view")
        self.height_view.setMinimumSize(QSize(220, 80))
        self.height_view.setMaximumSize(QSize(220, 80))

        self.horizontalLayout.addWidget(self.height_view)

        self.widget_19 = QWidget(self.height_widget)
        self.widget_19.setObjectName(u"widget_19")
        self.verticalLayout_13 = QVBoxLayout(self.widget_19)
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.verticalLayout_13.setContentsMargins(0, 0, 0, 0)
        self.verticalSpacer_9 = QSpacerItem(20, 5, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_13.addItem(self.verticalSpacer_9)

        self.height_slider = QSlider(self.widget_19)
        self.height_slider.setObjectName(u"height_slider")
        self.height_slider.setMaximum(230)
        self.height_slider.setValue(100)
        self.height_slider.setOrientation(Qt.Horizontal)

        self.verticalLayout_13.addWidget(self.height_slider)

        self.widget_20 = QWidget(self.widget_19)
        self.widget_20.setObjectName(u"widget_20")
        self.horizontalLayout_11 = QHBoxLayout(self.widget_20)
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.horizontalLayout_11.setContentsMargins(0, 0, 0, 0)
        self.label_18 = QLabel(self.widget_20)
        self.label_18.setObjectName(u"label_18")

        self.horizontalLayout_11.addWidget(self.label_18)

        self.horizontalSpacer_6 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_11.addItem(self.horizontalSpacer_6)

        self.label_17 = QLabel(self.widget_20)
        self.label_17.setObjectName(u"label_17")

        self.horizontalLayout_11.addWidget(self.label_17)

        self.horizontalSpacer_7 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_11.addItem(self.horizontalSpacer_7)

        self.label_16 = QLabel(self.widget_20)
        self.label_16.setObjectName(u"label_16")

        self.horizontalLayout_11.addWidget(self.label_16)


        self.verticalLayout_13.addWidget(self.widget_20)

        self.verticalSpacer_8 = QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_13.addItem(self.verticalSpacer_8)


        self.horizontalLayout.addWidget(self.widget_19)


        self.verticalLayout_9.addWidget(self.height_widget)


        self.horizontalLayout_8.addWidget(self.widget_12)

        self.widget_14 = QWidget(self.widget_13)
        self.widget_14.setObjectName(u"widget_14")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.widget_14.sizePolicy().hasHeightForWidth())
        self.widget_14.setSizePolicy(sizePolicy2)
        self.verticalLayout_10 = QVBoxLayout(self.widget_14)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.label_9 = QLabel(self.widget_14)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setAlignment(Qt.AlignCenter)

        self.verticalLayout_10.addWidget(self.label_9)

        self.corpus_position = QGraphicsView(self.widget_14)
        self.corpus_position.setObjectName(u"corpus_position")
        self.corpus_position.setMinimumSize(QSize(120, 180))
        self.corpus_position.setMaximumSize(QSize(120, 180))

        self.verticalLayout_10.addWidget(self.corpus_position)


        self.horizontalLayout_8.addWidget(self.widget_14)


        self.verticalLayout_2.addWidget(self.widget_13)

        self.velocity_widget = QWidget(self.central_widget)
        self.velocity_widget.setObjectName(u"velocity_widget")
        sizePolicy.setHeightForWidth(self.velocity_widget.sizePolicy().hasHeightForWidth())
        self.velocity_widget.setSizePolicy(sizePolicy)
        self.horizontalLayout_4 = QHBoxLayout(self.velocity_widget)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.widget_3 = QWidget(self.velocity_widget)
        self.widget_3.setObjectName(u"widget_3")
        self.verticalLayout_3 = QVBoxLayout(self.widget_3)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalSpacer_4 = QSpacerItem(20, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_3.addItem(self.verticalSpacer_4)

        self.vdir_dial = QDial(self.widget_3)
        self.vdir_dial.setObjectName(u"vdir_dial")
        self.vdir_dial.setMinimumSize(QSize(80, 80))
        self.vdir_dial.setMaximumSize(QSize(80, 16777215))
        self.vdir_dial.setMaximum(360)
        self.vdir_dial.setSingleStep(3)
        self.vdir_dial.setValue(180)
        self.vdir_dial.setWrapping(True)
        self.vdir_dial.setNotchesVisible(True)

        self.verticalLayout_3.addWidget(self.vdir_dial)

        self.verticalSpacer_5 = QSpacerItem(20, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_3.addItem(self.verticalSpacer_5)

        self.label = QLabel(self.widget_3)
        self.label.setObjectName(u"label")
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setAlignment(Qt.AlignCenter)

        self.verticalLayout_3.addWidget(self.label)


        self.horizontalLayout_4.addWidget(self.widget_3)

        self.widget_2 = QWidget(self.velocity_widget)
        self.widget_2.setObjectName(u"widget_2")
        self.verticalLayout = QVBoxLayout(self.widget_2)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalSpacer_2 = QSpacerItem(20, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer_2)

        self.widget_15 = QWidget(self.widget_2)
        self.widget_15.setObjectName(u"widget_15")
        self.verticalLayout_11 = QVBoxLayout(self.widget_15)
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.verticalLayout_11.setContentsMargins(0, 0, 0, 0)
        self.vval_slider = QSlider(self.widget_15)
        self.vval_slider.setObjectName(u"vval_slider")
        self.vval_slider.setMaximum(100)
        self.vval_slider.setOrientation(Qt.Horizontal)
        self.vval_slider.setTickPosition(QSlider.NoTicks)
        self.vval_slider.setTickInterval(0)

        self.verticalLayout_11.addWidget(self.vval_slider)

        self.widget_16 = QWidget(self.widget_15)
        self.widget_16.setObjectName(u"widget_16")
        self.horizontalLayout_9 = QHBoxLayout(self.widget_16)
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.horizontalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.label_12 = QLabel(self.widget_16)
        self.label_12.setObjectName(u"label_12")

        self.horizontalLayout_9.addWidget(self.label_12)

        self.horizontalSpacer_3 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_9.addItem(self.horizontalSpacer_3)

        self.label_11 = QLabel(self.widget_16)
        self.label_11.setObjectName(u"label_11")

        self.horizontalLayout_9.addWidget(self.label_11)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_9.addItem(self.horizontalSpacer_2)

        self.label_10 = QLabel(self.widget_16)
        self.label_10.setObjectName(u"label_10")

        self.horizontalLayout_9.addWidget(self.label_10)


        self.verticalLayout_11.addWidget(self.widget_16)


        self.verticalLayout.addWidget(self.widget_15)

        self.verticalSpacer_3 = QSpacerItem(20, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer_3)

        self.label_2 = QLabel(self.widget_2)
        self.label_2.setObjectName(u"label_2")
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setAlignment(Qt.AlignCenter)

        self.verticalLayout.addWidget(self.label_2)


        self.horizontalLayout_4.addWidget(self.widget_2)


        self.verticalLayout_2.addWidget(self.velocity_widget)

        self.widget = QWidget(self.central_widget)
        self.widget.setObjectName(u"widget")
        self.horizontalLayout_3 = QHBoxLayout(self.widget)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.widget_4 = QWidget(self.widget)
        self.widget_4.setObjectName(u"widget_4")
        self.verticalLayout_4 = QVBoxLayout(self.widget_4)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.omega_dial = QDial(self.widget_4)
        self.omega_dial.setObjectName(u"omega_dial")
        self.omega_dial.setMinimumSize(QSize(80, 80))
        self.omega_dial.setMaximumSize(QSize(80, 80))
        self.omega_dial.setMinimum(-17)
        self.omega_dial.setMaximum(17)
        self.omega_dial.setNotchesVisible(True)

        self.verticalLayout_4.addWidget(self.omega_dial)

        self.label_3 = QLabel(self.widget_4)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setAlignment(Qt.AlignCenter)

        self.verticalLayout_4.addWidget(self.label_3)


        self.horizontalLayout_3.addWidget(self.widget_4)

        self.widget_5 = QWidget(self.widget)
        self.widget_5.setObjectName(u"widget_5")
        self.verticalLayout_5 = QVBoxLayout(self.widget_5)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.yaw_dial = QDial(self.widget_5)
        self.yaw_dial.setObjectName(u"yaw_dial")
        self.yaw_dial.setMinimumSize(QSize(80, 80))
        self.yaw_dial.setMaximumSize(QSize(80, 80))
        self.yaw_dial.setMinimum(-120)
        self.yaw_dial.setMaximum(120)
        self.yaw_dial.setSingleStep(2)
        self.yaw_dial.setNotchesVisible(True)

        self.verticalLayout_5.addWidget(self.yaw_dial)

        self.yaw_label = QLabel(self.widget_5)
        self.yaw_label.setObjectName(u"yaw_label")
        self.yaw_label.setAlignment(Qt.AlignCenter)

        self.verticalLayout_5.addWidget(self.yaw_label)


        self.horizontalLayout_3.addWidget(self.widget_5)

        self.widget_6 = QWidget(self.widget)
        self.widget_6.setObjectName(u"widget_6")
        self.verticalLayout_6 = QVBoxLayout(self.widget_6)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.pitch_dial = QDial(self.widget_6)
        self.pitch_dial.setObjectName(u"pitch_dial")
        self.pitch_dial.setMinimumSize(QSize(80, 80))
        self.pitch_dial.setMaximumSize(QSize(80, 80))
        self.pitch_dial.setMinimum(-250)
        self.pitch_dial.setMaximum(250)
        self.pitch_dial.setSingleStep(4)
        self.pitch_dial.setNotchesVisible(True)

        self.verticalLayout_6.addWidget(self.pitch_dial)

        self.label_4 = QLabel(self.widget_6)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setAlignment(Qt.AlignCenter)

        self.verticalLayout_6.addWidget(self.label_4)


        self.horizontalLayout_3.addWidget(self.widget_6)

        self.widget_7 = QWidget(self.widget)
        self.widget_7.setObjectName(u"widget_7")
        self.verticalLayout_7 = QVBoxLayout(self.widget_7)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.roll_dial = QDial(self.widget_7)
        self.roll_dial.setObjectName(u"roll_dial")
        self.roll_dial.setMinimumSize(QSize(80, 80))
        self.roll_dial.setMaximumSize(QSize(80, 80))
        self.roll_dial.setMinimum(-120)
        self.roll_dial.setMaximum(120)
        self.roll_dial.setSingleStep(2)
        self.roll_dial.setPageStep(10)
        self.roll_dial.setNotchesVisible(True)

        self.verticalLayout_7.addWidget(self.roll_dial)

        self.label_5 = QLabel(self.widget_7)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setAlignment(Qt.AlignCenter)

        self.verticalLayout_7.addWidget(self.label_5)


        self.horizontalLayout_3.addWidget(self.widget_7)

        self.widget_8 = QWidget(self.widget)
        self.widget_8.setObjectName(u"widget_8")
        self.verticalLayout_8 = QVBoxLayout(self.widget_8)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.verticalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.widget_21 = QWidget(self.widget_8)
        self.widget_21.setObjectName(u"widget_21")
        self.verticalLayout_14 = QVBoxLayout(self.widget_21)
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.verticalLayout_14.setContentsMargins(0, 0, 0, 0)
        self.verticalSpacer_11 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_14.addItem(self.verticalSpacer_11)

        self.step_height_slider = QSlider(self.widget_21)
        self.step_height_slider.setObjectName(u"step_height_slider")
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.step_height_slider.sizePolicy().hasHeightForWidth())
        self.step_height_slider.setSizePolicy(sizePolicy3)
        self.step_height_slider.setMinimumSize(QSize(0, 15))
        self.step_height_slider.setMaximumSize(QSize(16777215, 15))
        self.step_height_slider.setMaximum(150)
        self.step_height_slider.setOrientation(Qt.Horizontal)

        self.verticalLayout_14.addWidget(self.step_height_slider)

        self.widget_22 = QWidget(self.widget_21)
        self.widget_22.setObjectName(u"widget_22")
        self.horizontalLayout_12 = QHBoxLayout(self.widget_22)
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.horizontalLayout_12.setContentsMargins(0, 0, 0, 0)
        self.label_20 = QLabel(self.widget_22)
        self.label_20.setObjectName(u"label_20")

        self.horizontalLayout_12.addWidget(self.label_20)

        self.horizontalSpacer_8 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_12.addItem(self.horizontalSpacer_8)

        self.label_19 = QLabel(self.widget_22)
        self.label_19.setObjectName(u"label_19")

        self.horizontalLayout_12.addWidget(self.label_19)


        self.verticalLayout_14.addWidget(self.widget_22)

        self.verticalSpacer_10 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_14.addItem(self.verticalSpacer_10)


        self.verticalLayout_8.addWidget(self.widget_21)

        self.label_6 = QLabel(self.widget_8)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setAlignment(Qt.AlignCenter)

        self.verticalLayout_8.addWidget(self.label_6)


        self.horizontalLayout_3.addWidget(self.widget_8)


        self.verticalLayout_2.addWidget(self.widget)

        self.widget_9 = QWidget(self.central_widget)
        self.widget_9.setObjectName(u"widget_9")
        self.horizontalLayout_5 = QHBoxLayout(self.widget_9)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.widget_10 = QWidget(self.widget_9)
        self.widget_10.setObjectName(u"widget_10")
        self.horizontalLayout_6 = QHBoxLayout(self.widget_10)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.label_7 = QLabel(self.widget_10)
        self.label_7.setObjectName(u"label_7")

        self.horizontalLayout_6.addWidget(self.label_7)

        self.gait_selection = QComboBox(self.widget_10)
        self.gait_selection.setObjectName(u"gait_selection")

        self.horizontalLayout_6.addWidget(self.gait_selection)


        self.horizontalLayout_5.addWidget(self.widget_10)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_5.addItem(self.horizontalSpacer)

        self.widget_11 = QWidget(self.widget_9)
        self.widget_11.setObjectName(u"widget_11")
        self.horizontalLayout_7 = QHBoxLayout(self.widget_11)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.label_8 = QLabel(self.widget_11)
        self.label_8.setObjectName(u"label_8")

        self.horizontalLayout_7.addWidget(self.label_8)

        self.checkBox_1 = QCheckBox(self.widget_11)
        self.checkBox_1.setObjectName(u"checkBox_1")

        self.horizontalLayout_7.addWidget(self.checkBox_1)

        self.checkBox_2 = QCheckBox(self.widget_11)
        self.checkBox_2.setObjectName(u"checkBox_2")

        self.horizontalLayout_7.addWidget(self.checkBox_2)

        self.checkBox_3 = QCheckBox(self.widget_11)
        self.checkBox_3.setObjectName(u"checkBox_3")

        self.horizontalLayout_7.addWidget(self.checkBox_3)

        self.checkBox_4 = QCheckBox(self.widget_11)
        self.checkBox_4.setObjectName(u"checkBox_4")

        self.horizontalLayout_7.addWidget(self.checkBox_4)

        self.checkBox_5 = QCheckBox(self.widget_11)
        self.checkBox_5.setObjectName(u"checkBox_5")

        self.horizontalLayout_7.addWidget(self.checkBox_5)

        self.checkBox_6 = QCheckBox(self.widget_11)
        self.checkBox_6.setObjectName(u"checkBox_6")

        self.horizontalLayout_7.addWidget(self.checkBox_6)


        self.horizontalLayout_5.addWidget(self.widget_11)


        self.verticalLayout_2.addWidget(self.widget_9)

        self.verticalSpacer = QSpacerItem(20, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer)

        HexapodController.setCentralWidget(self.central_widget)

        self.retranslateUi(HexapodController)

        self.gait_selection.setCurrentIndex(-1)


        QMetaObject.connectSlotsByName(HexapodController)
    # setupUi

    def retranslateUi(self, HexapodController):
        HexapodController.setWindowTitle(QCoreApplication.translate("HexapodController", u"Elkapod Controller", None))
        self.label_14.setText(QCoreApplication.translate("HexapodController", u"0.000", None))
        self.label_15.setText(QCoreApplication.translate("HexapodController", u"0.023", None))
        self.label_13.setText(QCoreApplication.translate("HexapodController", u"0.046", None))
        self.label_18.setText(QCoreApplication.translate("HexapodController", u"0.0", None))
        self.label_17.setText(QCoreApplication.translate("HexapodController", u"0.2", None))
        self.label_16.setText(QCoreApplication.translate("HexapodController", u"0.4", None))
        self.label_9.setText(QCoreApplication.translate("HexapodController", u"Corpus Position", None))
        self.label.setText(QCoreApplication.translate("HexapodController", u"v direction", None))
        self.label_12.setText(QCoreApplication.translate("HexapodController", u"0.000m/s", None))
        self.label_11.setText(QCoreApplication.translate("HexapodController", u"0.015m/s", None))
        self.label_10.setText(QCoreApplication.translate("HexapodController", u"0.030m/s", None))
        self.label_2.setText(QCoreApplication.translate("HexapodController", u"v value", None))
        self.label_3.setText(QCoreApplication.translate("HexapodController", u"omega", None))
        self.yaw_label.setText(QCoreApplication.translate("HexapodController", u"yaw", None))
        self.label_4.setText(QCoreApplication.translate("HexapodController", u"pitch", None))
        self.label_5.setText(QCoreApplication.translate("HexapodController", u"roll", None))
        self.label_20.setText(QCoreApplication.translate("HexapodController", u"0.000", None))
        self.label_19.setText(QCoreApplication.translate("HexapodController", u"0.015", None))
        self.label_6.setText(QCoreApplication.translate("HexapodController", u"step height", None))
        self.label_7.setText(QCoreApplication.translate("HexapodController", u"Gait", None))
        self.gait_selection.setCurrentText("")
        self.label_8.setText(QCoreApplication.translate("HexapodController", u"Supportive legs", None))
        self.checkBox_1.setText(QCoreApplication.translate("HexapodController", u"1", None))
        self.checkBox_2.setText(QCoreApplication.translate("HexapodController", u"2", None))
        self.checkBox_3.setText(QCoreApplication.translate("HexapodController", u"3", None))
        self.checkBox_4.setText(QCoreApplication.translate("HexapodController", u"4", None))
        self.checkBox_5.setText(QCoreApplication.translate("HexapodController", u"5", None))
        self.checkBox_6.setText(QCoreApplication.translate("HexapodController", u"6", None))
    # retranslateUi

