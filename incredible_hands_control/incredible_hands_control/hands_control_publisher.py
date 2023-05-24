import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from .hand_solution import HandProcessing
#from hand_solution import HandProcessing
from incredible_interface.msg import HandControl
from mediapipe.python.solutions.hands import HandLandmark
from geometry_msgs.msg import Twist

from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition

bridge = CvBridge()
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

class HandsPublisher(Node):
    def __init__(self):
        super().__init__('hands_publisher')
        qos = QoSProfile(depth=10)
        self.time_period = 0.01

        self.image_publisher = self.create_publisher(Image, 'image', qos)
        self.image_publisher2 = self.create_publisher(Image, 'image2', qos)
        self.hand_control_publisher = self.create_publisher(HandControl, 'hand_control', qos)
        self.burger_cmd_publisher = self.create_publisher(Twist, 'burger/cmd_vel', qos)
        self.waffle_cmd_publisher = self.create_publisher(Twist, 'waffle_pi/cmd_vel', qos)

        self.timer = self.create_timer(self.time_period, self.time_callback)
        self.missing_timer = 0
        self.cap = cv2.VideoCapture(0)
        self.cap2 = cv2.VideoCapture(2)
        self.hand_processing = HandProcessing()
        self.left_cmd_vel = Twist()
        self.right_cmd_vel = Twist()
        self.hand_control = HandControl()
        self.left_missing = False
        self.right_missing = False
        self.left_missing_count = 0
        self.right_missing_count = 0


    def time_callback(self):
        ret, image = self.cap.read()
        ret2, image2 = self.cap2.read()
        image = cv2.flip(image, 1)
        image_h, image_w, _ = image.shape
        my_hands = self.hand_processing.process_hands(image)

        self.missing_check(my_hands)

        for hand in my_hands:
            hand.draw_landmarks(image)
            hand.draw_bounding_box(image)
            if hand.handedness == "Left":
                left_hand_control, self.left_cmd_vel = self.make_hand_control_msg(hand, image_w, image_h)
                self.hand_control_publisher.publish(left_hand_control)
                self.burger_cmd_publisher.publish(self.left_cmd_vel)
            else:
                right_hand_control, self.right_cmd_vel = self.make_hand_control_msg(hand, image_w, image_h)
                self.hand_control_publisher.publish(right_hand_control)
                self.waffle_cmd_publisher.publish(self.right_cmd_vel)

        if self.left_missing:
            self.left_cmd_vel.linear.x = 0.0
            self.left_cmd_vel.angular.z = 0.0
        if self.right_missing:
            self.right_cmd_vel.linear.x = 0.0
            self.right_cmd_vel.angular.z = 0.0
        self.burger_cmd_publisher.publish(self.left_cmd_vel)
        self.waffle_cmd_publisher.publish(self.right_cmd_vel)

        if ret == True:
            image_encoded = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        if ret2 == True:
            image2_encoded = bridge.cv2_to_imgmsg(image2, encoding='bgr8')
        self.image_publisher.publish(image_encoded)
        self.image_publisher2.publish(image2_encoded)
        #cv2.imshow('image', image)
        cv2.waitKey(1)

    def missing_check(self, my_hands):
        missing_limit = 0.1

        if len(my_hands) == 0:
            self.left_missing_count += 1
            self.right_missing_count += 1
        elif len(my_hands) == 1:
            if my_hands[0].handedness == "Left":
                self.left_missing_count = 0
                self.right_missing_count += 1
            else:
                self.left_missing_count += 1
                self.right_missing_count = 0
        else:
            self.left_missing_count = 0
            self.right_missing_count = 0

        if self.left_missing_count * self.time_period >= missing_limit:
            self.left_missing = True
        else:
            self.left_missing = False
        if self.right_missing_count * self.time_period >= missing_limit:
            self.right_missing = True
        else:
            self.right_missing = False

    def make_hand_control_msg(self, hand, image_w, image_h):
        handedness = hand.get_handedness()
        finger_up_list = hand.get_finger_up_list()
        num_of_fingers_up = hand.get_num_of_fingers_up()

        ### HandControl
        hand_control = HandControl()
        hand_control.handedness = handedness
        hand_control.num_of_finger_up = num_of_fingers_up

        if finger_up_list[1] != True:
            command = "Stop"
        else:
            command = "Move"
        selected_landmark = hand.get_selected_landmark(HandLandmark.INDEX_FINGER_TIP)

        hand_control.command = command
        hand_control.x = selected_landmark[1]
        hand_control.y = selected_landmark[2]

        ### Hand_CMD_VEL
        if handedness == "Left":
            max_linear_vel = (num_of_fingers_up / 5) * BURGER_MAX_LIN_VEL
            max_angular_vel = (num_of_fingers_up / 5) * BURGER_MAX_ANG_VEL
        else:
            max_linear_vel = (num_of_fingers_up / 5) * WAFFLE_MAX_LIN_VEL
            max_angular_vel = (num_of_fingers_up / 5) * WAFFLE_MAX_ANG_VEL

        hand_cmd_vel = Twist()
        cx, cy = image_w / 2, image_h / 2
        hand_cmd_vel.linear.x = -1 * (hand_control.y - cy)/cy * max_linear_vel
        hand_cmd_vel.angular.z = -1 * (hand_control.x - cx)/cx * max_angular_vel

        return hand_control, hand_cmd_vel


def main(args=None):
    rclpy.init(args=args)
    node = HandsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Publish Stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

