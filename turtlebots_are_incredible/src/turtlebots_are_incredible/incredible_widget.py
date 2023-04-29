#!/usr/bin/env python3
# Copyright 2019 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.resources import get_resource
from geometry_msgs.msg import Twist
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut
from python_qt_binding.QtWidgets import QWidget
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import BatteryState
from incredible_interface.action import MoveXWithTime
from turtlebot3_msgs.srv import Sound

class IncredibleWidget(QWidget):
    def __init__(self, node):
        super(IncredibleWidget, self).__init__()
        self.setObjectName('IncredibleWidget')

        self.node = node

        self.REDRAW_INTERVAL = 30
        self.PUBLISH_INTERVAL = 100
        self.CMD_VEL_X_FACTOR = 1000.0
        self.CMD_VEL_YAW_FACTOR = -10.0

        self.burger_selected = False
        self.waffle_selected = False

        pkg_name = 'turtlebots_are_incredible'
        ui_filename = 'turtlebots_are_incredible.ui'
        burger_cmd_topic = 'burger/cmd_vel'
        waffle_cmd_topic = 'waffle_pi/cmd_vel'
        burger_battery_topic = 'burger/battery_state'
        waffle_battery_topic = 'waffle_pi/battery_state'

        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        self.pub_velocity = Twist()
        self.pub_velocity.linear.x = 0.0
        self.pub_velocity.angular.z = 0.0
        self.sub_burger_battery = BatteryState()
        self.sub_burger_velocity = Twist()
        self.sub_burger_velocity.linear.x = 0.0
        self.sub_burger_velocity.angular.z = 0.0
        self.sub_waffle_battery = BatteryState()
        self.sub_waffle_velocity = Twist()
        self.sub_waffle_velocity.linear.x = 0.0
        self.sub_waffle_velocity.angular.z = 0.0

        self.burger_slider_x.setValue(0)
        self.burger_lcd_number_x.display(0.0)
        self.burger_lcd_number_yaw.display(0.0)
        self.waffle_slider_x.setValue(0)
        self.waffle_lcd_number_x.display(0.0)
        self.waffle_lcd_number_yaw.display(0.0)

        qos = QoSProfile(depth=10)
        self.burger_cmd_publisher = self.node.create_publisher(Twist, waffle_cmd_topic, qos)
        self.waffle_cmd_publisher = self.node.create_publisher(Twist, burger_cmd_topic, qos)
        self.burger_cmd_subscriber = self.node.create_subscription(Twist, burger_cmd_topic, self.get_burger_velocity, qos)
        self.waffle_cmd_subscriber = self.node.create_subscription(Twist, waffle_cmd_topic, self.get_waffle_velocity, qos)
        self.burger_battery_subscriber = self.node.create_subscription(BatteryState, waffle_battery_topic, self.get_burger_battery, qos)
        self.waffle_battery_subscriber = self.node.create_subscription(BatteryState, burger_battery_topic, self.get_waffle_battery, qos)

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.send_velocity)
        self.publish_timer.start(self.PUBLISH_INTERVAL)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_indicators)
        self.update_timer.start(self.REDRAW_INTERVAL)

        self.push_button_w.pressed.connect(self.increase_linear_x)
        self.push_button_x.pressed.connect(self.decrease_linear_x)
        self.push_button_a.pressed.connect(self.increase_angular_z)
        self.push_button_d.pressed.connect(self.decrease_angular_z)
        self.push_button_s.pressed.connect(self.set_stop)

        self.push_button_w.setShortcut('w')
        self.push_button_x.setShortcut('x')
        self.push_button_a.setShortcut('a')
        self.push_button_d.setShortcut('d')
        self.push_button_s.setShortcut('s')

        self.shortcut_space = QShortcut(QKeySequence(Qt.Key_Space), self)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)
        self.shortcut_space.activated.connect(self.push_button_s.pressed)

        self.check_box_burger.stateChanged.connect(self.burger_choose)
        self.check_box_waffle.stateChanged.connect(self.waffle_choose)
    def burger_choose(self, state):
        self.burger_selected = not self.burger_selected
    def waffle_choose(self, state):
        self.waffle_selected = not self.waffle_selected
    def get_burger_velocity(self, msg):
        self.sub_burger_velocity = msg
    def get_waffle_velocity(self, msg):
        self.sub_waffle_velocity = msg
    def get_burger_battery(self, msg):
        self.sub_burger_battery = msg
    def get_waffle_battery(self, msg):
        self.sub_waffle_battery = msg

    def increase_linear_x(self):
        self.pub_velocity.linear.x += 0.05
    def decrease_linear_x(self):
        self.pub_velocity.linear.x -= 0.05
    def increase_angular_z(self):
        self.pub_velocity.angular.z += 0.05
    def decrease_angular_z(self):
        self.pub_velocity.angular.z -= 0.05
    def set_stop(self):
        self.pub_velocity.linear.x = 0.0
        self.pub_velocity.angular.z = 0.0

    def send_velocity(self):
        twist = Twist()
        twist.linear.x = self.pub_velocity.linear.x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.pub_velocity.angular.z

        if self.burger_selected == True:
            self.burger_cmd_publisher.publish(twist)
        if self.waffle_selected == True:
            self.waffle_cmd_publisher.publish(twist)

    def update_indicators(self):
        self.burger_battery.setValue(int(self.sub_burger_battery.percentage))
        self.burger_slider_x.setValue(self.sub_burger_velocity.linear.x * self.CMD_VEL_X_FACTOR)
        self.burger_dial_yaw.setValue(self.sub_burger_velocity.angular.z * self.CMD_VEL_YAW_FACTOR)
        self.burger_lcd_number_x.display(self.sub_burger_velocity.linear.x)
        self.burger_lcd_number_yaw.display(self.sub_burger_velocity.angular.z)
        self.waffle_battery.setValue(int(self.sub_waffle_battery.percentage))
        self.waffle_slider_x.setValue(self.sub_waffle_velocity.linear.x * self.CMD_VEL_X_FACTOR)
        self.waffle_dial_yaw.setValue(self.sub_waffle_velocity.angular.z * self.CMD_VEL_YAW_FACTOR)
        self.waffle_lcd_number_x.display(self.sub_waffle_velocity.linear.x)
        self.waffle_lcd_number_yaw.display(self.sub_waffle_velocity.angular.z)

    def shutdown_widget(self):
        self.update_timer.stop()
        self.publish_timer.stop()
        self.node.destroy_subscription(self.burger_cmd_subscriber)
        self.node.destroy_subscription(self.waffle_cmd_subscriber)
        self.node.destroy_subscription(self.burger_battery_subscriber)
        self.node.destroy_subscription(self.waffle_battery_subscriber)
        self.node.destroy_publisher(self.burger_cmd_publisher)
        self.node.destroy_publisher(self.waffle_cmd_publisher)