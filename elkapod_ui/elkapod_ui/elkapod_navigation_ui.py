# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'elkapod_navigation.ui'
##
## Created by: Qt User Interface Compiler version 6.10.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (
    QCoreApplication,
    QDate,
    QDateTime,
    QLocale,
    QMetaObject,
    QObject,
    QPoint,
    QRect,
    QSize,
    QTime,
    QUrl,
    Qt,
)
from PySide6.QtGui import (
    QBrush,
    QColor,
    QConicalGradient,
    QCursor,
    QFont,
    QFontDatabase,
    QGradient,
    QIcon,
    QImage,
    QKeySequence,
    QLinearGradient,
    QPainter,
    QPalette,
    QPixmap,
    QRadialGradient,
    QTransform,
)
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLayout,
    QPushButton,
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName("Form")
        Form.resize(800, 570)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Form.sizePolicy().hasHeightForWidth())
        Form.setSizePolicy(sizePolicy)
        font = QFont()
        font.setKerning(False)
        Form.setFont(font)
        Form.setStyleSheet("background-color: rgb(42, 42, 42);")
        self.horizontalLayout = QHBoxLayout(Form)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setSpacing(20)
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout.setSizeConstraint(QLayout.SizeConstraint.SetMinAndMaxSize)
        self.verticalLayout.setContentsMargins(5, 5, 5, -1)
        self.Navigation = QLabel(Form)
        self.Navigation.setObjectName("Navigation")
        sizePolicy1 = QSizePolicy(
            QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Maximum
        )
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.Navigation.sizePolicy().hasHeightForWidth())
        self.Navigation.setSizePolicy(sizePolicy1)
        font1 = QFont()
        font1.setPointSize(20)
        font1.setKerning(True)
        self.Navigation.setFont(font1)
        self.Navigation.setLineWidth(1)
        self.Navigation.setTextFormat(Qt.TextFormat.AutoText)
        self.Navigation.setAlignment(
            Qt.AlignmentFlag.AlignLeading
            | Qt.AlignmentFlag.AlignLeft
            | Qt.AlignmentFlag.AlignTop
        )

        self.verticalLayout.addWidget(self.Navigation)

        self.slamWidget_2 = QWidget(Form)
        self.slamWidget_2.setObjectName("slamWidget_2")
        sizePolicy1.setHeightForWidth(
            self.slamWidget_2.sizePolicy().hasHeightForWidth()
        )
        self.slamWidget_2.setSizePolicy(sizePolicy1)
        self.slamWidget_2.setMinimumSize(QSize(450, 150))
        self.slamWidget_2.setStyleSheet("background-color: rgb(82, 81, 81);")
        self.slamWidget = QHBoxLayout(self.slamWidget_2)
        self.slamWidget.setSpacing(0)
        self.slamWidget.setObjectName("slamWidget")
        self.slamWidget.setSizeConstraint(QLayout.SizeConstraint.SetFixedSize)
        self.slamWidget.setContentsMargins(3, 3, 3, 0)
        self.slamInfo = QVBoxLayout()
        self.slamInfo.setSpacing(10)
        self.slamInfo.setObjectName("slamInfo")
        self.slam = QLabel(self.slamWidget_2)
        self.slam.setObjectName("slam")
        sizePolicy2 = QSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum
        )
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.slam.sizePolicy().hasHeightForWidth())
        self.slam.setSizePolicy(sizePolicy2)
        font2 = QFont()
        font2.setPointSize(13)
        self.slam.setFont(font2)

        self.slamInfo.addWidget(self.slam)

        self.slamStatusBox = QWidget(self.slamWidget_2)
        self.slamStatusBox.setObjectName("slamStatusBox")
        sizePolicy2.setHeightForWidth(
            self.slamStatusBox.sizePolicy().hasHeightForWidth()
        )
        self.slamStatusBox.setSizePolicy(sizePolicy2)
        self.horizontalLayout_3 = QHBoxLayout(self.slamStatusBox)
        self.horizontalLayout_3.setSpacing(10)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.status = QLabel(self.slamStatusBox)
        self.status.setObjectName("status")
        sizePolicy1.setHeightForWidth(self.status.sizePolicy().hasHeightForWidth())
        self.status.setSizePolicy(sizePolicy1)
        self.status.setScaledContents(False)
        self.status.setAlignment(
            Qt.AlignmentFlag.AlignLeading
            | Qt.AlignmentFlag.AlignLeft
            | Qt.AlignmentFlag.AlignTop
        )

        self.horizontalLayout_3.addWidget(self.status)

        self.slam_status = QLabel(self.slamStatusBox)
        self.slam_status.setObjectName("slamStatus")
        sizePolicy1.setHeightForWidth(self.slam_status.sizePolicy().hasHeightForWidth())
        self.slam_status.setSizePolicy(sizePolicy1)
        self.slam_status.setAlignment(
            Qt.AlignmentFlag.AlignLeading
            | Qt.AlignmentFlag.AlignLeft
            | Qt.AlignmentFlag.AlignTop
        )

        self.horizontalLayout_3.addWidget(self.slam_status)

        self.horizontalSpacer_2 = QSpacerItem(
            40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum
        )

        self.horizontalLayout_3.addItem(self.horizontalSpacer_2)

        self.slamInfo.addWidget(self.slamStatusBox)

        self.verticalSpacer_4 = QSpacerItem(
            20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding
        )

        self.slamInfo.addItem(self.verticalSpacer_4)

        self.slamWidget.addLayout(self.slamInfo)

        self.slamControls = QVBoxLayout()
        self.slamControls.setSpacing(0)
        self.slamControls.setObjectName("slamControls")
        self.slamControls.setContentsMargins(-1, 10, -1, 10)
        self.mode = QLabel(self.slamWidget_2)
        self.mode.setObjectName("mode")
        sizePolicy2.setHeightForWidth(self.mode.sizePolicy().hasHeightForWidth())
        self.mode.setSizePolicy(sizePolicy2)

        self.slamControls.addWidget(self.mode)

        self.slam_mode_combo = QComboBox(self.slamWidget_2)
        self.slam_mode_combo.setObjectName("slamModeCombo")
        sizePolicy3 = QSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed
        )
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(
            self.slam_mode_combo.sizePolicy().hasHeightForWidth()
        )
        self.slam_mode_combo.setSizePolicy(sizePolicy3)

        self.slamControls.addWidget(self.slam_mode_combo)

        self.verticalSpacer_2 = QSpacerItem(
            20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred
        )

        self.slamControls.addItem(self.verticalSpacer_2)

        self.horizontalWidget = QWidget(self.slamWidget_2)
        self.horizontalWidget.setObjectName("horizontalWidget")
        sizePolicy2.setHeightForWidth(
            self.horizontalWidget.sizePolicy().hasHeightForWidth()
        )
        self.horizontalWidget.setSizePolicy(sizePolicy2)
        self.horizontalLayout_2 = QHBoxLayout(self.horizontalWidget)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.slam_resume = QPushButton(self.horizontalWidget)
        self.slam_resume.setObjectName("slamResume")

        self.horizontalLayout_2.addWidget(self.slam_resume)

        self.slam_pause = QPushButton(self.horizontalWidget)
        self.slam_pause.setObjectName("slamPause")

        self.horizontalLayout_2.addWidget(self.slam_pause)

        self.slam_restart = QPushButton(self.horizontalWidget)
        self.slam_restart.setObjectName("slamRestart")

        self.horizontalLayout_2.addWidget(self.slam_restart)

        self.slamControls.addWidget(self.horizontalWidget)

        self.slamWidget.addLayout(self.slamControls)

        self.verticalLayout.addWidget(self.slamWidget_2)

        self.odomWidget_2 = QWidget(Form)
        self.odomWidget_2.setObjectName("odomWidget_2")
        sizePolicy1.setHeightForWidth(
            self.odomWidget_2.sizePolicy().hasHeightForWidth()
        )
        self.odomWidget_2.setSizePolicy(sizePolicy1)
        self.odomWidget_2.setMinimumSize(QSize(450, 150))
        self.odomWidget_2.setStyleSheet("background-color: rgb(82, 81, 81);")
        self.odomWidget = QHBoxLayout(self.odomWidget_2)
        self.odomWidget.setSpacing(0)
        self.odomWidget.setObjectName("odomWidget")
        self.odomWidget.setSizeConstraint(QLayout.SizeConstraint.SetFixedSize)
        self.odomWidget.setContentsMargins(3, 3, 3, 0)
        self.odomInfo = QVBoxLayout()
        self.odomInfo.setSpacing(10)
        self.odomInfo.setObjectName("odomInfo")
        self.odometry = QLabel(self.odomWidget_2)
        self.odometry.setObjectName("odometry")
        sizePolicy2.setHeightForWidth(self.odometry.sizePolicy().hasHeightForWidth())
        self.odometry.setSizePolicy(sizePolicy2)
        self.odometry.setFont(font2)

        self.odomInfo.addWidget(self.odometry)

        self.horizontalWidget_4 = QWidget(self.odomWidget_2)
        self.horizontalWidget_4.setObjectName("horizontalWidget_4")
        sizePolicy2.setHeightForWidth(
            self.horizontalWidget_4.sizePolicy().hasHeightForWidth()
        )
        self.horizontalWidget_4.setSizePolicy(sizePolicy2)
        self.horizontalLayout_4 = QHBoxLayout(self.horizontalWidget_4)
        self.horizontalLayout_4.setSpacing(10)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.status_2 = QLabel(self.horizontalWidget_4)
        self.status_2.setObjectName("status_2")
        sizePolicy1.setHeightForWidth(self.status_2.sizePolicy().hasHeightForWidth())
        self.status_2.setSizePolicy(sizePolicy1)
        self.status_2.setAlignment(
            Qt.AlignmentFlag.AlignLeading
            | Qt.AlignmentFlag.AlignLeft
            | Qt.AlignmentFlag.AlignTop
        )

        self.horizontalLayout_4.addWidget(self.status_2)

        self.odom_status = QLabel(self.horizontalWidget_4)
        self.odom_status.setObjectName("odomStatus")
        sizePolicy1.setHeightForWidth(self.odom_status.sizePolicy().hasHeightForWidth())
        self.odom_status.setSizePolicy(sizePolicy1)
        self.odom_status.setAlignment(
            Qt.AlignmentFlag.AlignLeading
            | Qt.AlignmentFlag.AlignLeft
            | Qt.AlignmentFlag.AlignTop
        )

        self.horizontalLayout_4.addWidget(self.odom_status)

        self.horizontalSpacer_3 = QSpacerItem(
            40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum
        )

        self.horizontalLayout_4.addItem(self.horizontalSpacer_3)

        self.odomInfo.addWidget(self.horizontalWidget_4)

        self.verticalSpacer_3 = QSpacerItem(
            20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding
        )

        self.odomInfo.addItem(self.verticalSpacer_3)

        self.odomWidget.addLayout(self.odomInfo)

        self.odomControls = QVBoxLayout()
        self.odomControls.setSpacing(0)
        self.odomControls.setObjectName("odomControls")
        self.odomControls.setContentsMargins(-1, 10, -1, 10)
        self.odomStats = QLabel(self.odomWidget_2)
        self.odomStats.setObjectName("odomStats")
        sizePolicy2.setHeightForWidth(self.odomStats.sizePolicy().hasHeightForWidth())
        self.odomStats.setSizePolicy(sizePolicy2)

        self.odomControls.addWidget(self.odomStats)

        self.verticalSpacer = QSpacerItem(
            20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum
        )

        self.odomControls.addItem(self.verticalSpacer)

        self.horizontalWidget_2 = QWidget(self.odomWidget_2)
        self.horizontalWidget_2.setObjectName("horizontalWidget_2")
        sizePolicy2.setHeightForWidth(
            self.horizontalWidget_2.sizePolicy().hasHeightForWidth()
        )
        self.horizontalWidget_2.setSizePolicy(sizePolicy2)
        self.horizontalLayout_5 = QHBoxLayout(self.horizontalWidget_2)
        self.horizontalLayout_5.setSpacing(0)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.horizontalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.odom_resume = QPushButton(self.horizontalWidget_2)
        self.odom_resume.setObjectName("odomResume")

        self.horizontalLayout_5.addWidget(self.odom_resume)

        self.odom_pause = QPushButton(self.horizontalWidget_2)
        self.odom_pause.setObjectName("odomPause")

        self.horizontalLayout_5.addWidget(self.odom_pause)

        self.odom_restart = QPushButton(self.horizontalWidget_2)
        self.odom_restart.setObjectName("odomRestart")

        self.horizontalLayout_5.addWidget(self.odom_restart)

        self.odomControls.addWidget(self.horizontalWidget_2)

        self.odomWidget.addLayout(self.odomControls)

        self.verticalLayout.addWidget(self.odomWidget_2)

        self.verticalSpacer_5 = QSpacerItem(
            20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding
        )

        self.verticalLayout.addItem(self.verticalSpacer_5)

        self.horizontalLayout.addLayout(self.verticalLayout)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalSpacer = QSpacerItem(
            40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum
        )

        self.verticalLayout_2.addItem(self.horizontalSpacer)

        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)

    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", "Form", None))
        self.Navigation.setText(QCoreApplication.translate("Form", "Navigation", None))
        self.slam.setText(QCoreApplication.translate("Form", "SLAM", None))
        self.status.setText(QCoreApplication.translate("Form", "Status", None))
        self.slam_status.setText(QCoreApplication.translate("Form", "TextLabel", None))
        self.mode.setText(QCoreApplication.translate("Form", "Mode:", None))
        self.slam_resume.setText(QCoreApplication.translate("Form", "Resume", None))
        self.slam_pause.setText(QCoreApplication.translate("Form", "Pause", None))
        self.slam_restart.setText(QCoreApplication.translate("Form", "Restart", None))
        self.odometry.setText(QCoreApplication.translate("Form", "Odometry", None))
        self.status_2.setText(QCoreApplication.translate("Form", "Status", None))
        self.odom_status.setText(QCoreApplication.translate("Form", "TextLabel", None))
        self.odomStats.setText(QCoreApplication.translate("Form", "TextLabel", None))
        self.odom_resume.setText(QCoreApplication.translate("Form", "Resume", None))
        self.odom_pause.setText(QCoreApplication.translate("Form", "Pause", None))
        self.odom_restart.setText(QCoreApplication.translate("Form", "Restart", None))

    # retranslateUi
