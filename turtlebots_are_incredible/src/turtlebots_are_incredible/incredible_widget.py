#!/usr/bin/env python3
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
import rclpy
import time

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut
from python_qt_binding.QtWidgets import QWidget

from ament_index_python.resources import get_resource
from rclpy.qos import QoSProfile
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse

from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from turtlebot3_msgs.srv import Sound
from incredible_interface.msg import HandControl
from incredible_interface.action import MoveXWithTime

BURGER_MAX_LIN_VEL = 0.22
WAFFLE_MAX_LIN_VEL = 0.26

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

class IncredibleWidget(QWidget):
    def __init__(self, node):
        super(IncredibleWidget, self).__init__()
        self.setObjectName('IncredibleWidget')

        self.node = node

        self.REDRAW_INTERVAL = 30
        self.PUBLISH_INTERVAL = 100
        self.CMD_VEL_X_FACTOR = 1000.0
        self.CMD_VEL_YAW_FACTOR = -10.0

        self.e_stop_click_status = False
        self.burger_selected = False
        self.waffle_selected = False
        self.hands_control_mode = False

        pkg_name = 'turtlebots_are_incredible'
        ui_filename = 'turtlebots_are_incredible.ui'
        burger_cmd_topic = 'burger/cmd_vel'
        waffle_cmd_topic = 'waffle_pi/cmd_vel'
        burger_battery_topic = 'burger/battery_state'
        waffle_battery_topic = 'waffle_pi/battery_state'
        hand_control_topic = 'hand_control'
        self.action_name = 'move_x_with_time'

        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)

        self.pub_burger_velocity = Twist()
        self.pub_burger_velocity.linear.x = 0.0
        self.pub_burger_velocity.angular.z = 0.0
        self.sub_burger_battery = BatteryState()
        self.sub_burger_velocity = Twist()
        self.sub_burger_velocity.linear.x = 0.0
        self.sub_burger_velocity.angular.z = 0.0
        self.left_hand_control = HandControl()

        self.pub_waffle_velocity = Twist()
        self.pub_waffle_velocity.linear.x = 0.0
        self.pub_waffle_velocity.angular.z = 0.0
        self.sub_waffle_battery = BatteryState()
        self.sub_waffle_velocity = Twist()
        self.sub_waffle_velocity.linear.x = 0.0
        self.sub_waffle_velocity.angular.z = 0.0
        self.right_hand_control = HandControl()

        qos = QoSProfile(depth=10)
        self.burger_cmd_publisher = self.node.create_publisher(Twist, burger_cmd_topic, qos)
        self.waffle_cmd_publisher = self.node.create_publisher(Twist, waffle_cmd_topic, qos)
        self.burger_cmd_subscriber = self.node.create_subscription(Twist, burger_cmd_topic, self.get_burger_velocity, qos)
        self.waffle_cmd_subscriber = self.node.create_subscription(Twist, waffle_cmd_topic, self.get_waffle_velocity, qos)
        self.burger_battery_subscriber = self.node.create_subscription(BatteryState, burger_battery_topic, self.get_burger_battery, qos)
        self.waffle_battery_subscriber = self.node.create_subscription(BatteryState, waffle_battery_topic, self.get_waffle_battery, qos)
        self.hand_control_subscriber = self.node.create_subscription(HandControl, hand_control_topic, self.get_hand_control, qos)

        self.burger_alarm_client = self.node.create_client(Sound, 'burger/sound')
        self.waffle_alarm_client = self.node.create_client(Sound, 'waffle_pi/sound')
        self.burger_action_client = rclpy.action.ActionClient(self.node, MoveXWithTime, self.action_name)
        self.waffle_action_client = rclpy.action.ActionClient(self.node, MoveXWithTime, self.action_name)
        self.move_with_time_action_server = rclpy.action.ActionServer(
            node=self.node,
            action_type=MoveXWithTime,
            action_name=self.action_name,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback)

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self.send_velocity)
        self.publish_timer.start(self.PUBLISH_INTERVAL)

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_indicators)
        self.update_timer.start(self.REDRAW_INTERVAL)

        self.push_button_alarm.clicked.connect(self.make_alarm)
        self.push_button_e_stop.clicked.connect(self.e_stop_clicked)
        self.push_button_action_start.clicked.connect(self.make_action)
        self.push_button_action_cancel.clicked.connect(self.cancel_action)
        self.push_button_hands_control.clicked.connect(self.activate_hands_control)

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


    # robot choosing calls
    def burger_choose(self):
        self.burger_selected = not self.burger_selected
    def waffle_choose(self):
        self.waffle_selected = not self.waffle_selected

    # w, a, s, d, x key/button calls
    def increase_linear_x(self):
        self.pub_burger_velocity.linear.x += LIN_VEL_STEP_SIZE
        self.pub_waffle_velocity.linear.x += LIN_VEL_STEP_SIZE
    def decrease_linear_x(self):
        self.pub_burger_velocity.linear.x -= LIN_VEL_STEP_SIZE
        self.pub_waffle_velocity.linear.x -= LIN_VEL_STEP_SIZE
    def increase_angular_z(self):
        self.pub_burger_velocity.angular.z += ANG_VEL_STEP_SIZE
        self.pub_waffle_velocity.angular.z += ANG_VEL_STEP_SIZE
    def decrease_angular_z(self):
        self.pub_burger_velocity.angular.z -= ANG_VEL_STEP_SIZE
        self.pub_waffle_velocity.angular.z -= ANG_VEL_STEP_SIZE
    def set_stop(self):
        self.pub_burger_velocity.linear.x = 0.0
        self.pub_burger_velocity.angular.z = 0.0
        self.pub_waffle_velocity.linear.x = 0.0
        self.pub_waffle_velocity.angular.z = 0.0
    def e_stop_clicked(self):
        self.e_stop_click_status = True
        self.cancel_action()
        self.set_stop()
        self.make_alarm()
    def activate_hands_control(self):
        self.hands_control_mode = not self.hands_control_mode
        if self.hands_control_mode:
            self.push_button_hands_control.setText("Deactivate Hands Control")
        else:
            self.push_button_hands_control.setText("Activate Hands Control")


    # cmd subscriber calls
    def get_burger_velocity(self, msg):
        self.sub_burger_velocity = msg
    def get_waffle_velocity(self, msg):
        self.sub_waffle_velocity = msg
    def get_burger_battery(self, msg):
        self.sub_burger_battery = msg
    def get_waffle_battery(self, msg):
        self.sub_waffle_battery = msg
    def get_hand_control(self, msg):
        if msg.handedness == "Left":
            self.left_hand_control = msg
        else:
            self.right_hand_control = msg



    # service client call
    def make_alarm(self):
        request_burger = Sound.Request()
        request_waffle = Sound.Request()

        if self.burger_selected == True:
            while not self.burger_alarm_client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    self.node.get_logger().error('interruped while waiting for the server.')
                    return
                else:
                    self.node.get_logger().info('server not available, waiting again...')
            request_burger.value = 0
            future_burger = self.burger_alarm_client.call_async(request_burger)
            if future_burger.done():
                try:
                    response = future_burger.result()
                except Exception as e:
                    raise RuntimeError(f'exception while calling service: {future_burger.exception()}')
                else:
                    self.node.get_logger().info(f'Result success:{response.success} message:{response.message}')
                    self.node.get_logger().info('==== burger Call Done ====')

        if self.waffle_selected == True:
            while not self.waffle_alarm_client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    self.node.get_logger().error('interruped while waiting for the server.')
                    return
                else:
                    self.node.get_logger().info('server not available, waiting again...')
            request_waffle.value = 1
            future_waffle = self.waffle_alarm_client.call_async(request_waffle)
            if future_waffle.done():
                try:
                    response = future_waffle.result()
                except Exception as e:
                    raise RuntimeError(f'exception while calling service: {future_waffle.exception()}')
                else:
                    self.node.get_logger().info(f'Result success:{response.success} message:{response.message}')
                    self.node.get_logger().info('==== waffle Call Done ====')

    # action start call
    def make_action(self):
        if self.burger_action_client.wait_for_server(1.0) is False:
            self.node.get_logger().error('Server not available')

        if self.burger_selected == True:
            burger_goal = MoveXWithTime.Goal()
            burger_goal.vel_x = float(self.line_edit_vel_x.text())
            if burger_goal.vel_x < -BURGER_MAX_LIN_VEL:
                burger_goal.vel_x = -BURGER_MAX_LIN_VEL
            elif burger_goal.vel_x > BURGER_MAX_LIN_VEL:
                burger_goal.vel_x = BURGER_MAX_LIN_VEL
            self.line_edit_vel_x.setText(str(burger_goal.vel_x))
            burger_goal.time = float(self.line_edit_time.text())

            self.burger_remain_lcd_number.display(burger_goal.vel_x * burger_goal.time)
            self.burger_send_goal_future = self.burger_action_client.send_goal_async(burger_goal, feedback_callback=self.feedback_callback)
            self.burger_send_goal_future.add_done_callback(self.goal_response_callback)

        if self.waffle_selected == True:
            waffle_goal = MoveXWithTime.Goal()
            waffle_goal.vel_x = float(self.line_edit_vel_x.text())
            if waffle_goal.vel_x < -WAFFLE_MAX_LIN_VEL:
                waffle_goal.vel_x = -WAFFLE_MAX_LIN_VEL
            elif waffle_goal.vel_x > WAFFLE_MAX_LIN_VEL:
                waffle_goal.vel_x = WAFFLE_MAX_LIN_VEL
            self.line_edit_vel_x.setText(str(waffle_goal.vel_x))
            waffle_goal.time = float(self.line_edit_time.text())
            self.waffle_remain_lcd_number.display(waffle_goal.vel_x * waffle_goal.time)
            self.waffle_send_goal_future = self.waffle_action_client.send_goal_async(waffle_goal, feedback_callback=self.feedback_callback)
            self.waffle_send_goal_future.add_done_callback(self.goal_response_callback)


    # action client calls for execute
    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().info('Goal rejected by server')
            return
        else:
            self.node.get_logger().info('Goal accepted by server, waiting for result')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance_remain = feedback.distance_remain
        if self.burger_selected == True:
            self.burger_remain_lcd_number.display(distance_remain)
        if self.waffle_selected == True:
            self.waffle_remain_lcd_number.display(distance_remain)
    def result_callback(self, future):
        result = future.result().result
        if self.burger_selected == True:
            self.burger_movement_lcd_number.display(result.total_movement)
        if self.waffle_selected == True:
            self.waffle_movement_lcd_number.display(result.total_movement)
        self.node.get_logger().info(f'Server successfully executed goal. total movement is : {result.total_movement}')


    # action client calls for cancel
    def cancel_action(self):
        self.node.get_logger().info('Canceling goal')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.client_cancel_callback)
    def client_cancel_callback(self, future):
        stop_response = future.result()
        #self.burger_movement_lcd_number.display(stop_response.total_movement)
        if len(stop_response.goals.canceling) > 0:
            self.node.get_logger().info("Goal successfully canceled")
        else:
            self.node.get_logger().info("Goal failed to cancel")


    # action server calls
    def goal_callback(self, goal_request):
        self.node.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    def cancel_callback(self, goal_handle):
        self.node.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    def execute_callback(self, goal_handle):
        self.node.get_logger().info('Executing goal')
        goal = goal_handle.request
        feedback = MoveXWithTime.Feedback()
        result = MoveXWithTime.Result()

        past = self.node.get_clock().now()
        now = self.node.get_clock().now()
        total_distance = goal.vel_x * goal.time
        if self.burger_selected == True:
            self.pub_burger_velocity.linear.x = goal.vel_x
            self.pub_burger_velocity.angular.z = 0.0
        if self.waffle_selected == True:
            self.pub_waffle_velocity.linear.x = goal.vel_x
            self.pub_waffle_velocity.angular.z = 0.0

        while ((now - past).nanoseconds * 1e-9 <= goal.time):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                if self.burger_selected == True:
                    self.pub_burger_velocity.linear.x = 0.0
                if self.waffle_selected == True:
                    self.pub_waffle_velocity.linear.x = 0.0
                self.node.get_logger().error('Goal Canceled')
                return result
            total_movement = goal.vel_x * ((now - past).nanoseconds * 1e-9)
            feedback.distance_remain = total_distance - total_movement
            goal_handle.publish_feedback(feedback)
            result.total_movement = total_movement
            now = self.node.get_clock().now()

        self.pub_burger_velocity.linear.x = 0.0
        self.pub_waffle_velocity.linear.x = 0.0

        goal_handle.succeed()
        self.node.get_logger().info('Successfully executed goal')
        return result


    # timer callbacks which is called periodically
    def send_velocity(self):
        burger_twist = Twist()
        burger_twist.linear.x = self.pub_burger_velocity.linear.x
        burger_twist.angular.z = self.pub_burger_velocity.angular.z

        waffle_twist = Twist()
        waffle_twist.linear.x = self.pub_waffle_velocity.linear.x
        waffle_twist.angular.z = self.pub_waffle_velocity.angular.z

        if self.burger_selected == True and self.hands_control_mode == False:
            self.burger_cmd_publisher.publish(burger_twist)
        if self.waffle_selected == True and self.hands_control_mode == False:
            self.waffle_cmd_publisher.publish(waffle_twist)

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

        self.left_hand_command.setText(self.left_hand_control.command)
        self.left_hand_handedness.setText(self.left_hand_control.handedness)
        self.left_hand_num_of_fingers_up.display(self.left_hand_control.num_of_finger_up)
        self.left_hand_x.display(self.left_hand_control.x)
        self.left_hand_y.display(self.left_hand_control.y)
        self.right_hand_command.setText(self.right_hand_control.command)
        self.right_hand_handedness.setText(self.right_hand_control.handedness)
        self.right_hand_num_of_fingers_up.display(self.right_hand_control.num_of_finger_up)
        self.right_hand_x.display(self.right_hand_control.x)
        self.right_hand_y.display(self.right_hand_control.y)


    # calls when program ends
    def shutdown_widget(self):
        self.update_timer.stop()
        self.publish_timer.stop()
        self.node.destroy_subscription(self.burger_cmd_subscriber)
        self.node.destroy_subscription(self.waffle_cmd_subscriber)
        self.node.destroy_subscription(self.burger_battery_subscriber)
        self.node.destroy_subscription(self.waffle_battery_subscriber)
        self.node.destroy_subscription(self.hand_control_subscriber)
        self.node.destroy_publisher(self.burger_cmd_publisher)
        self.node.destroy_publisher(self.waffle_cmd_publisher)
        self.node.destroy_client(self.burger_alarm_client)
        self.node.destroy_client(self.waffle_alarm_client)
