import sys
import cv2
import rclpy
import subprocess
import numpy as np

from PyQt5 import QtGui
from PyQt5.QtGui import QPixmap, QImage
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
from PyQt5.QtCore import Qt, QTimer
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QSlider, QMainWindow, QAction, QLabel

from qt_material import apply_stylesheet

QT_DEBUG_PLUGINS = 1
cmd_str = 'ros2 node list'
MAIN_WINDOW_WIDTH = 2560
MAIN_WINDOW_HEIGHT = 1440


class MainWindow(QMainWindow):
    MAIN_COLOR = '#efeff2'
    SECONDARY_TEXT_COLOR = '#8f9394'
    FRAME_COLOR = '#e4e9eb'
    BORDER_COLOR = '#dfe3e4'
    PRESSED_BUTTON_COLOR = '#426787'

    def __init__(self, node: Node):
        super().__init__()
        # Подготовка параметров ROS2
        self.guiNode = node
        self.mode = 'debug'
        self.availableNodes = []

        # Подготовка интерфейса
        self.setWindowTitle("Crawler Bot Control Panel")
        self.showFullScreen()
        self.close()

        self.MENU_WIDTH = self.size().width()
        self.MENU_HEIGHT = 25

        self.FRAME_COLS = 3
        self.FRAME_ROWS = 1
        self.FRAME_SPACE = 10
        self.FRAME_WIDTH = int(MAIN_WINDOW_WIDTH / self.FRAME_COLS -
                               (self.FRAME_SPACE * self.FRAME_COLS + self.FRAME_SPACE))
        self.FRAME_HEIGHT = int(MAIN_WINDOW_HEIGHT / self.FRAME_ROWS -
                                (self.FRAME_SPACE * self.FRAME_ROWS + self.FRAME_SPACE)) - self.MENU_HEIGHT

        self.mainTimer = QTimer(self)
        self.mainTimer.setInterval(100)
        self.mainTimer.timeout.connect(self.timer_callback)
        self.mainTimer.start()

        self.connectionTimer = QTimer(self)
        self.connectionTimer.setInterval(2000)
        self.connectionTimer.timeout.connect(self.connection_timer_callback)
        self.connectionTimer.start()

        self.mainMenu = self.menuBar()
        self.viewMenu = self.mainMenu.addMenu("Настройки")

        self.fullscreenAction = QAction("Полноэкранный режим", self)
        self.fullscreenAction.setShortcut("shift+alt+enter")
        self.fullscreenAction.triggered.connect(self.toggle_fullscreen)
        self.exitAction = QAction("Выход", self)
        self.exitAction.setShortcut("esc")
        self.exitAction.triggered.connect(self.toggle_exit)
        self.viewMenu.addAction(self.fullscreenAction)
        self.viewMenu.addAction(self.exitAction)

        self.operatingFrame = QWidget(self)
        self.operatingFrame.setGeometry(self.FRAME_SPACE, self.MENU_HEIGHT + self.FRAME_SPACE,
                                        self.FRAME_WIDTH, int(self.FRAME_HEIGHT / self.FRAME_ROWS))
        self.operatingFrame.setStyleSheet(f'background-color: {self.FRAME_COLOR};'
                                          f'border: 1px solid {self.BORDER_COLOR};')

        self.operatingLabel = QLabel(self.operatingFrame)
        self.operatingLabel.setStyleSheet(f'border: 0px;'
                                          f'color: {self.SECONDARY_TEXT_COLOR}; '
                                          f'font: Arial; font-size: 20px;')
        self.operatingLabel.setGeometry(self.FRAME_SPACE,
                                        self.FRAME_SPACE,
                                        200, 22)
        self.operatingLabel.setText('OPERATING PANEL')

        self.debugModeButton = QPushButton('DEBUG', self.operatingFrame)
        self.debugModeButton.setGeometry(self.FRAME_SPACE, 42,
                                         int((self.operatingFrame.size().width() - self.FRAME_SPACE * 2) / 3),
                                         60)
        self.debugModeButton.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                           f'color: black;'
                                           f'font: Normal; font-size: 16px;')
        self.debugModeButton.clicked.connect(self.debug_mode)

        self.manualModeButton = QPushButton('MANUAL', self.operatingFrame)
        self.manualModeButton.setGeometry(self.debugModeButton.pos().x() + self.debugModeButton.size().width(), 42,
                                          int((self.operatingFrame.size().width() - self.FRAME_SPACE * 2) / 3),
                                          60)
        self.manualModeButton.setStyleSheet(f'background-color: {self.MAIN_COLOR}; color: black;'
                                            f'font: Arial; font-size: 16px;')
        self.manualModeButton.clicked.connect(self.manual_mode)

        self.autoModeButton = QPushButton('AUTO', self.operatingFrame)
        self.autoModeButton.setGeometry(self.manualModeButton.pos().x() + self.manualModeButton.size().width(), 42,
                                        int((self.operatingFrame.size().width() - self.FRAME_SPACE * 2) / 3),
                                        60)
        self.autoModeButton.setStyleSheet(f'background-color: {self.MAIN_COLOR}; color: black;'
                                          f'font: Arial; font-size: 16px;')
        self.autoModeButton.clicked.connect(self.auto_mode)

        self.currentLinearSpeedMonitor = QLabel(self.operatingFrame)
        self.currentLinearSpeedMonitor.setGeometry(self.FRAME_SPACE,
                                                   self.FRAME_SPACE + self.debugModeButton.pos().y() + self.debugModeButton.size().height(),
                                                   int((self.operatingFrame.size().width() - self.FRAME_SPACE * 2) / 3),
                                                   30)
        self.currentLinearSpeedMonitor.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                                     f'color: {self.SECONDARY_TEXT_COLOR};'
                                                     f'font-size: 18px;'
                                                     f'font: Arial;'
                                                     f'border: 1px solid {self.BORDER_COLOR};')
        self.currentLinearSpeedMonitor.setText(f'V_X: /n')

        self.currentAngularSpeedMonitor = QLabel(self.operatingFrame)
        self.currentAngularSpeedMonitor.setGeometry(
            self.currentLinearSpeedMonitor.pos().x() + self.currentLinearSpeedMonitor.size().width(),
            self.FRAME_SPACE + self.debugModeButton.pos().y() + self.debugModeButton.size().height(),
            int((
                        self.operatingFrame.size().width() - self.FRAME_SPACE * 2) / 3),
            30)
        self.currentAngularSpeedMonitor.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                                      f'color: {self.SECONDARY_TEXT_COLOR};'
                                                      f'font-size: 18px;'
                                                      f'font: Arial;'
                                                      f'border: 1px solid {self.BORDER_COLOR};')
        self.currentAngularSpeedMonitor.setText(f'V_YAW:   /n')

        self.currentAccelerationMonitor = QLabel(self.operatingFrame)
        self.currentAccelerationMonitor.setGeometry(
            self.currentAngularSpeedMonitor.pos().x() + self.currentAngularSpeedMonitor.size().width(),
            self.FRAME_SPACE + self.debugModeButton.pos().y() + self.debugModeButton.size().height(),
            int((
                        self.operatingFrame.size().width() - self.FRAME_SPACE * 2) / 3),
            30)
        self.currentAccelerationMonitor.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                                      f'color: {self.SECONDARY_TEXT_COLOR};'
                                                      f'font-size: 18px;'
                                                      f'font: Arial;'
                                                      f'border: 1px solid {self.BORDER_COLOR};')
        self.currentAccelerationMonitor.setText(f'ACCEL: /n')

        self.debugLabel = QLabel(self.operatingFrame)
        self.debugLabel.setStyleSheet(f'border: 0px;'
                                      f'color: {self.SECONDARY_TEXT_COLOR}; '
                                      f'font: Arial; font-size: 20px;')
        self.debugLabel.setGeometry(self.FRAME_SPACE,
                                    self.FRAME_SPACE + self.currentLinearSpeedMonitor.pos().y() + self.currentLinearSpeedMonitor.size().height(),
                                    200, 22)
        self.debugLabel.setText('DEBUG MODE PANEL')

        self.debugFrame = QWidget(self.operatingFrame)
        self.debugFrame.setGeometry(self.FRAME_SPACE,
                                    self.FRAME_SPACE + self.debugLabel.pos().y() + self.debugLabel.size().height(),
                                    self.FRAME_WIDTH - self.FRAME_SPACE * 2, 194)
        self.debugFrame.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                      f'border: 1px solid {self.BORDER_COLOR};')
        self.debugFrame.setEnabled(False)

        self.linearSpeedLabel = QLabel(self.debugFrame)
        self.linearSpeedLabel.setStyleSheet(f'border: 0px;'
                                            f'color: gray; '
                                            f'font: Arial; font-size: 16px;')
        self.linearSpeedLabel.setGeometry(self.FRAME_SPACE,
                                          self.FRAME_SPACE,
                                          self.FRAME_WIDTH, 18)
        self.linearSpeedLabel.setText('ROBOT LINEAR SPEED')

        self.linearSpeedSlider = QSlider(Qt.Horizontal, self.debugFrame)
        self.linearSpeedSlider.setGeometry(self.FRAME_SPACE,
                                           self.linearSpeedLabel.y() + self.linearSpeedLabel.height()
                                           + self.FRAME_SPACE,
                                           int(self.debugFrame.size().width() * 0.5 - self.FRAME_SPACE * 2),
                                           30)
        self.linearSpeedSlider.setMinimum(0)
        self.linearSpeedSlider.setMaximum(100)
        self.linearSpeedSlider.setTickPosition(QSlider.TicksBelow)
        self.linearSpeedSlider.setTickInterval(2)
        self.linearSpeedSlider.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                             f'border: 0px solid {self.BORDER_COLOR};')

        self.linearSpeedMonitor = QLabel(self.debugFrame)
        self.linearSpeedMonitor.setGeometry(self.linearSpeedSlider.x() + self.linearSpeedSlider.size().width()
                                            + self.FRAME_SPACE,
                                            self.linearSpeedSlider.y(),
                                            int(self.debugFrame.width() * 0.24 - self.FRAME_SPACE * 2),
                                            30)
        self.linearSpeedMonitor.setText(f"{round(float(self.linearSpeedSlider.value()) / 100, 2)}")
        self.linearSpeedMonitor.setStyleSheet(f'border: 1px solid {self.BORDER_COLOR};'
                                              f'color: gray;'
                                              f'font: Arial; font-size: 20px;')

        self.angularSpeedLabel = QLabel(self.debugFrame)
        self.angularSpeedLabel.setStyleSheet(f'border: 0px;'
                                             f'color: gray; '
                                             f'font: Arial; font-size: 16px;')
        self.angularSpeedLabel.setGeometry(self.FRAME_SPACE,
                                           self.linearSpeedSlider.y() + self.linearSpeedSlider.height() +
                                           self.FRAME_SPACE,
                                           self.FRAME_WIDTH, 18)
        self.angularSpeedLabel.setText('ROBOT ANGULAR SPEED')

        self.angularSpeedSlider = QSlider(Qt.Horizontal, self.debugFrame)
        self.angularSpeedSlider.setGeometry(self.FRAME_SPACE,
                                            self.angularSpeedLabel.y() + self.angularSpeedLabel.height(),
                                            int(self.debugFrame.width() * 0.5 - self.FRAME_SPACE * 2),
                                            30)
        self.angularSpeedSlider.setMinimum(0)
        self.angularSpeedSlider.setMaximum(100)
        self.angularSpeedSlider.setTickPosition(QSlider.TicksBelow)
        self.angularSpeedSlider.setTickInterval(2)
        self.angularSpeedSlider.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                              f'border: 0px solid {self.BORDER_COLOR};')

        self.angularSpeedMonitor = QLabel(self.debugFrame)
        self.angularSpeedMonitor.setGeometry(self.angularSpeedSlider.x() + self.angularSpeedSlider.size().width()
                                             + self.FRAME_SPACE,
                                             self.angularSpeedSlider.y(),
                                             int(self.debugFrame.width() * 0.24 - self.FRAME_SPACE * 2),
                                             30)
        self.angularSpeedMonitor.setText(f"{round(float(self.angularSpeedSlider.value()) / 100, 2)}")
        self.angularSpeedMonitor.setStyleSheet(f'border: 1px solid {self.BORDER_COLOR};'
                                               f'color: gray; '
                                               f'font: Arial; font-size: 20px;')

        self.averageAccelerationLabel = QLabel(self.debugFrame)
        self.averageAccelerationLabel.setStyleSheet(f'border: 0px;'
                                                    f'color: gray; '
                                                    f'font: Arial; font-size: 16px;')
        self.averageAccelerationLabel.setGeometry(self.FRAME_SPACE,
                                                  self.angularSpeedSlider.y() + self.angularSpeedSlider.height()
                                                  + self.FRAME_SPACE,
                                                  self.FRAME_WIDTH, 18)
        self.averageAccelerationLabel.setText('ROBOT AVERAGE ACCELERATION')

        self.averageAccelerationSlider = QSlider(Qt.Horizontal, self.debugFrame)
        self.averageAccelerationSlider.setGeometry(self.FRAME_SPACE,
                                                   self.averageAccelerationLabel.y()
                                                   + self.averageAccelerationLabel.height(),
                                                   int(self.debugFrame.width() * 0.5 - self.FRAME_SPACE * 2),
                                                   30)
        self.averageAccelerationSlider.setMinimum(0)
        self.averageAccelerationSlider.setMaximum(100)
        self.averageAccelerationSlider.setTickPosition(QSlider.TicksBelow)
        self.averageAccelerationSlider.setTickInterval(2)
        self.averageAccelerationSlider.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                                     f'border: 0px solid {self.BORDER_COLOR};')

        self.averageAccelerationMonitor = QLabel(self.debugFrame)
        self.averageAccelerationMonitor.setGeometry(
            self.averageAccelerationSlider.x() + self.averageAccelerationSlider.width()
            + self.FRAME_SPACE,
            self.averageAccelerationSlider.y(),
            int(self.debugFrame.width() * 0.24 - self.FRAME_SPACE * 2),
            30)
        self.averageAccelerationMonitor.setText(f"{round(float(self.averageAccelerationSlider.value()) / 100, 2)}")
        self.averageAccelerationMonitor.setStyleSheet(f'border: 1px solid {self.BORDER_COLOR};'
                                                      f'color: gray;'
                                                      f'font: Arial; font-size: 20px;')

        self.debugApplyButton = QPushButton('APPLY', self.debugFrame)
        self.debugApplyButton.setGeometry(int(self.debugLabel.x() + self.debugFrame.width() * 0.74 -
                                              self.FRAME_SPACE * 2),
                                          int(self.debugFrame.height() / 2 - 60),
                                          int(self.debugFrame.width() * 0.27 - self.FRAME_SPACE),
                                          146)
        self.debugApplyButton.setStyleSheet(f'background-color: {self.MAIN_COLOR}; '
                                            f'color: black; font: Arial; font-size: 16px;')
        self.debugApplyButton.clicked.connect(self.debug_apply)

        self.manualLabel = QLabel(self.operatingFrame)
        self.manualLabel.setStyleSheet(f'border: 0px;'
                                       f'color: {self.SECONDARY_TEXT_COLOR}; '
                                       f'font: Arial; font-size: 20px;')
        self.manualLabel.setGeometry(self.FRAME_SPACE,
                                     self.FRAME_SPACE + self.debugFrame.y() + self.debugFrame.height(),
                                     220,
                                     22)
        self.manualLabel.setText('MANUAL MODE PANEL')

        self.manualFrame = QWidget(self.operatingFrame)
        self.manualFrame.setGeometry(self.FRAME_SPACE,
                                     self.manualLabel.y() + self.manualLabel.height() + self.FRAME_SPACE,
                                     self.FRAME_WIDTH - self.FRAME_SPACE * 2, 50)
        self.manualFrame.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                       f'border: 1px solid {self.BORDER_COLOR};')
        self.manualFrame.setEnabled(False)

        self.controllerMonitor = QLabel(self.manualFrame)
        self.controllerMonitor.setGeometry(self.FRAME_SPACE,
                                           self.FRAME_SPACE,
                                           self.manualFrame.width() - 2 * self.FRAME_SPACE,
                                           30)
        self.controllerMonitor.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                             f'color: {self.SECONDARY_TEXT_COLOR};'
                                             f'font-size: 18px;'
                                             f'font: Arial;'
                                             f'border: 1px solid {self.BORDER_COLOR};')
        self.controllerMonitor.setText('Connecting Controller...')

        self.autoLabel = QLabel(self.operatingFrame)
        self.autoLabel.setStyleSheet(f'border: 0px;'
                                     f'color: {self.SECONDARY_TEXT_COLOR}; '
                                     f'font: Arial; font-size: 20px;')
        self.autoLabel.setGeometry(self.FRAME_SPACE,
                                   self.FRAME_SPACE + self.manualFrame.y() + self.manualFrame.height(),
                                   220,
                                   22)
        self.autoLabel.setText('AUTO MODE PANEL')

        self.autoFrame = QWidget(self.operatingFrame)

        self.autoFrame.setGeometry(self.FRAME_SPACE,
                                   self.FRAME_SPACE + self.autoLabel.y() + self.autoLabel.height(),
                                   self.FRAME_WIDTH - self.FRAME_SPACE * 2,
                                   int(self.FRAME_HEIGHT / self.FRAME_ROWS -
                                       (
                                               self.autoLabel.y() + self.autoLabel.height() + self.FRAME_SPACE * 2)))
        self.autoFrame.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                     f'border: 1px solid {self.BORDER_COLOR};')
        self.autoFrame.setEnabled(False)

        self.targetDistanceLabel = QLabel(self.autoFrame)
        self.targetDistanceLabel.setStyleSheet(f'border: 0px;'
                                               f'color: gray; '
                                               f'font: Arial; font-size: 16px;')
        self.targetDistanceLabel.setGeometry(self.FRAME_SPACE,
                                             self.FRAME_SPACE,
                                             self.FRAME_WIDTH, 18)
        self.targetDistanceLabel.setText('ROBOT TARGET DISTANCE')

        self.targetDistanceSlider = QSlider(Qt.Horizontal, self.autoFrame)
        self.targetDistanceSlider.setGeometry(self.FRAME_SPACE,
                                              self.targetDistanceLabel.y()
                                              + self.targetDistanceLabel.height() + self.FRAME_SPACE,
                                              int(self.debugFrame.width() * 0.5 - self.FRAME_SPACE * 2),
                                              30)
        self.targetDistanceSlider.setMinimum(0)
        self.targetDistanceSlider.setMaximum(600)
        self.targetDistanceSlider.setTickPosition(QSlider.TicksBelow)
        self.targetDistanceSlider.setTickInterval(12)
        self.targetDistanceSlider.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                                f'border: 0px solid {self.BORDER_COLOR};')

        self.targetDistanceMonitor = QLabel(self.autoFrame)
        self.targetDistanceMonitor.setGeometry(self.targetDistanceSlider.x() + self.targetDistanceSlider.size().width()
                                               + self.FRAME_SPACE,
                                               self.targetDistanceSlider.y(),
                                               int(self.debugFrame.width() * 0.24 - self.FRAME_SPACE * 2),
                                               30)
        self.targetDistanceMonitor.setText(f"{round(float(self.targetDistanceSlider.value()) / 100, 2)}")
        self.targetDistanceMonitor.setStyleSheet(f'border: 1px solid {self.BORDER_COLOR};'
                                                 f'color: gray;'
                                                 f'font: Arial; font-size: 20px;')

        self.autoApplyButton = QPushButton('APPLY', self.autoFrame)
        self.autoApplyButton.setGeometry(int(self.autoLabel.x() + self.autoFrame.width() * 0.74 -
                                             self.FRAME_SPACE * 2),
                                         int(self.FRAME_SPACE + 27),
                                         int(self.autoFrame.width() * 0.27 - self.FRAME_SPACE),
                                         30)
        self.autoApplyButton.setStyleSheet(f'background-color: {self.MAIN_COLOR}; '
                                           f'color: black; font: Arial; font-size: 16px;')
        self.autoApplyButton.clicked.connect(self.auto_apply)

        self.vizualLabel = QLabel(self)
        self.vizualLabel.setGeometry(self.FRAME_SPACE + self.FRAME_SPACE + self.FRAME_WIDTH,
                                     self.MENU_HEIGHT + self.FRAME_SPACE,
                                     self.FRAME_WIDTH + self.FRAME_WIDTH + self.FRAME_SPACE + self.FRAME_SPACE,
                                     int(self.FRAME_HEIGHT / self.FRAME_ROWS))
        self.vizualLabel.setStyleSheet(f'background-color: {self.FRAME_COLOR};'
                                       f'border: 1px solid {self.BORDER_COLOR};')

        self.videoFrame = QLabel(self.vizualLabel)
        self.videoFrame.setGeometry(self.FRAME_SPACE, self.FRAME_SPACE,
                                    self.FRAME_WIDTH + self.FRAME_WIDTH,
                                    int(self.FRAME_WIDTH * 2 / 16 * 9))
        self.videoFrame.setStyleSheet(f'background-color: {self.FRAME_COLOR};'
                                      f'border: 1px solid {self.BORDER_COLOR};')

        self.buttonAnimationTimer = QTimer()
        self.buttonAnimationTimer.timeout.connect(self.animate_button)
        self.buttonAnimationTimer.setInterval(100)
        self.buttonAnimationTimerCounter = 0
        self.buttonAnimationTimerColor = QtGui.QColor(self.PRESSED_BUTTON_COLOR)
        self.animatingButton = None

        self.showFullScreen()

    def animate_button(self):
        # Анимация нажатия кнопки помещенной в контейнер self.animatingButton
        if self.buttonAnimationTimerCounter == 2:
            self.buttonAnimationTimer.stop()
            self.buttonAnimationTimerCounter = 0
            self.animatingButton.setStyleSheet(f'background-color: {self.MAIN_COLOR}; '
                                               f'color: black;'
                                               f'font: Arial; font-size: 16px; ')
        else:
            self.animatingButton.setStyleSheet(f'background-color: {self.PRESSED_BUTTON_COLOR}; '
                                               f'color: {self.MAIN_COLOR};'
                                               f'font: Arial; font-size: 16px; font-weight: bold')
            self.buttonAnimationTimerCounter += 1

    def connection_timer_callback(self):
        # Каждые 3 секунды проверка существующих нод в системе
        self.availableNodes = subprocess.run(cmd_str, capture_output=True, shell=True, text=True).stdout.split('\n')

    def timer_callback(self):
        # Обновление задаваемых слайдерами скоростей
        self.linearSpeedMonitor.setText(f'{round(self.linearSpeedSlider.value() / 100, 2)}')
        self.angularSpeedMonitor.setText(f'{round(self.angularSpeedSlider.value() / 100, 2)}')
        self.averageAccelerationMonitor.setText(f'{round(self.averageAccelerationSlider.value() / 100, 2)}')
        self.targetDistanceMonitor.setText(f'{round(float(self.targetDistanceSlider.value()) / 100, 2)}')

        color_frame = self.guiNode.realsenseImage
        color_frame = cv2.resize(color_frame, (int(self.FRAME_WIDTH * 2), int(self.FRAME_WIDTH * 2 / 16 * 9)))
        # color_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = color_frame.shape
        img = QImage(color_frame.data, w, h, ch * w, QImage.Format_RGB888)
        scaled_img = img.scaled(int(self.FRAME_WIDTH * 2), int(self.FRAME_WIDTH * 2 / 16 * 9), Qt.KeepAspectRatio)

        self.videoFrame.setPixmap(QPixmap.fromImage(scaled_img))

        # Постоянная проверка параметров, если существуют соответствующие ноды
        for n in self.availableNodes:
            if n == '/robot':

                self.currentLinearSpeedMonitor.setText(
                    f'V_X: {round(self.guiNode.robotTwist.linear.x, 2)}')
                self.currentAngularSpeedMonitor.setText(
                    f'V_YAW: {round(self.guiNode.robotTwist.angular.z, 2)}')
                self.currentAccelerationMonitor.setText(f'ACCEL: /n')

                mode, linear, angular = self.guiNode.get_robot_parameters()
                if mode == 'debug' and self.mode != 'debug':
                    self.debug_mode()
                    self.mode = mode
                if mode == 'manual' and self.mode != 'manual':
                    self.manual_mode()
                    self.mode = mode
                if mode == 'auto' and self.mode != 'auto':
                    self.auto_mode()
                    self.mode = mode
            if n == '/controller':
                if self.guiNode.get_controller_parameter():
                    self.controllerMonitor.setText("Controller PS4 DualShock connected")
                else:
                    self.controllerMonitor.setText("Controller PS4 DualShock disconnected")

    def auto_apply(self):
        self.animatingButton = self.autoApplyButton
        self.buttonAnimationTimer.start()
        for n in self.availableNodes:
            if n == '/detector':
                self.guiNode.set_detector_parameter('target_z', self.targetDistanceSlider.value() / 100)

    def debug_apply(self):
        self.animatingButton = self.debugApplyButton
        self.buttonAnimationTimer.start()
        for n in self.availableNodes:
            if n == '/robot':
                self.guiNode.set_robot_parameter('debug_linear_velocity', self.linearSpeedSlider.value() / 100)
                self.guiNode.set_robot_parameter('debug_angular_velocity', self.angularSpeedSlider.value() / 100)

    def debug_mode(self):
        self.debugModeButton.setEnabled(False)
        self.manualModeButton.setEnabled(True)
        self.autoModeButton.setEnabled(True)

        self.debugFrame.setEnabled(True)
        self.manualFrame.setEnabled(False)
        self.autoFrame.setEnabled(False)

        self.debugModeButton.setStyleSheet(f'background-color: {self.PRESSED_BUTTON_COLOR};'
                                           f'color: {self.MAIN_COLOR};'
                                           f'font: Normal; font-size: 16px; font-weight: bold;')
        self.manualModeButton.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                            f'color: black;'
                                            f'font: Normal; font-size: 16px;')
        self.autoModeButton.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                          f'color: black;'
                                          f'font: Normal; font-size: 16px;')
        for n in self.availableNodes:
            if n == '/robot':
                self.guiNode.set_robot_parameter('operating_mode', 'debug')

    def manual_mode(self):
        self.debugModeButton.setEnabled(True)
        self.manualModeButton.setEnabled(False)
        self.autoModeButton.setEnabled(True)

        self.debugFrame.setEnabled(False)
        self.manualFrame.setEnabled(True)
        self.autoFrame.setEnabled(False)

        self.debugModeButton.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                           f'color: black;'
                                           f'font: Normal; font-size: 16px;')
        self.manualModeButton.setStyleSheet(f'background-color: {self.PRESSED_BUTTON_COLOR};'
                                            f'color: {self.MAIN_COLOR};'
                                            f'font: Normal; font-size: 16px; font-weight: bold;')
        self.autoModeButton.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                          f'color: black;'
                                          f'font: Normal; font-size: 16px;')
        for n in self.availableNodes:
            if n == '/robot':
                self.guiNode.set_robot_parameter('operating_mode', 'manual')

    def auto_mode(self):
        self.debugModeButton.setEnabled(True)
        self.manualModeButton.setEnabled(True)
        self.autoModeButton.setEnabled(False)

        self.debugFrame.setEnabled(False)
        self.manualFrame.setEnabled(False)
        self.autoFrame.setEnabled(True)

        self.debugModeButton.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                           f'color: black;'
                                           f'font: Normal; font-size: 16px;')
        self.manualModeButton.setStyleSheet(f'background-color: {self.MAIN_COLOR};'
                                            f'color: black;'
                                            f'font: Normal; font-size: 16px;')
        self.autoModeButton.setStyleSheet(f'background-color: {self.PRESSED_BUTTON_COLOR};'
                                          f'color: {self.MAIN_COLOR};'
                                          f'font: Normal; font-size: 16px; font-weight: bold;')
        for n in self.availableNodes:
            if n == '/robot':
                self.guiNode.set_robot_parameter('operating_mode', 'auto')

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def toggle_exit(self):
        self.close()


class GuiNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.getRobotParameterClient = self.create_client(GetParameters, 'robot/get_parameters')
        self.setRobotParameterClient = self.create_client(SetParameters, 'robot/set_parameters')

        self.getControllerStatusClient = self.create_client(SetBool, 'controller_connection_service')

        self.setDetectorParameterClient = self.create_client(SetParameters, 'detector/set_parameters')

        self.getParameterRequest = GetParameters.Request()
        self.setParameterRequest = SetParameters.Request()

        self.controllerStatusRequest = SetBool.Request()

        self.robotTwist = Twist()
        self.detectorTarget = [0., 0., 0., 0]
        self.realsenseImage = np.zeros((100, 100, 3))
        self.create_subscription(Twist,
                                 '/CrawlerBot/twist',
                                 self.robot_twist_callback,
                                 10)

        self.create_subscription(Twist,
                                 '/detector/target_box',
                                 self.detector_target_callback,
                                 10)

        self.create_subscription(CompressedImage,
                                 '/realsense/color_image',
                                 self.realsense_image_callback,
                                 10)

        self.cvBridge = CvBridge()

    def robot_twist_callback(self, msg):
        self.robotTwist = msg

    def detector_target_callback(self, msg):
        self.detectorTarget = [int(msg.linear.x), int(msg.linear.y), int(msg.angular.x), int(msg.angular.y)]

    def realsense_image_callback(self, msg):
        self.realsenseImage = self.cvBridge.compressed_imgmsg_to_cv2(msg)
        self.realsenseImage = cv2.rectangle(self.realsenseImage,
                                            self.detectorTarget[:2], self.detectorTarget[2:4],
                                            color=(0, 0, 255), thickness=3, lineType=0)

    def set_robot_parameter(self, name, value):
        param = Parameter()
        param.name = name
        if type(value) is str:
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = value
        elif type(value) is int:
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = value
        elif type(value) is float:
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = value
        elif type(value) is bool:
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = value
        else:
            pass
        req = SetParameters.Request()
        req.parameters.append(param)
        future = self.setRobotParameterClient.call_async(req)
        while not future.done():
            rclpy.spin_once(self)

    def set_detector_parameter(self, name, value):
        param = Parameter()
        param.name = name
        if type(value) is str:
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = value
        elif type(value) is int:
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = value
        elif type(value) is float:
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = value
        elif type(value) is bool:
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = value
        else:
            pass
        req = SetParameters.Request()
        req.parameters.append(param)
        future = self.setDetectorParameterClient.call_async(req)
        print(param.value)
        while not future.done():
            print("spinning...")
            rclpy.spin_once(self)

    def get_robot_parameters(self):
        self.getParameterRequest.names = ['operating_mode', 'debug_linear_velocity', 'debug_angular_velocity']
        future = self.getRobotParameterClient.call_async(self.getParameterRequest)
        while not future.done():
            rclpy.spin_once(self)
        res = [future.result().values[0].string_value, future.result().values[1].double_value,
               future.result().values[2].double_value]
        return res

    def get_controller_parameter(self):
        future = self.getControllerStatusClient.call_async(self.controllerStatusRequest)
        while not future.done():
            rclpy.spin_once(self)
        res = future.result().success
        return res


rclpy.init()
guiNode = GuiNode('gui')
application = QApplication(sys.argv)
window = MainWindow(guiNode)
application.exec_()
rclpy.shutdown()
exit()
