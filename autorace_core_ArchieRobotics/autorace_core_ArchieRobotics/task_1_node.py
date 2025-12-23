import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import os
from cv_bridge import CvBridge
import numpy as np
from ament_index_python.packages import get_package_share_directory
from collections import deque


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('task_1_node')
        self.publisher = self.create_publisher(String, '/comp/task_1', 10)
        self.subscription = self.create_subscription(Image, '/color/image', self.image_callback, 10)
        self.bridge = CvBridge()

    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_image(cv_image)

    
    def process_image(self, image):
        # Преобразуем изображение в формат HSV для работы с цветом
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Диапазон зеленого цвета
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])

        # Маска зеленого цвета
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        
        # Если есть зеленый цвет, то публикуем в /robot_start и завершаем работу узла
        if cv2.countNonZero(mask_green) > 0:
            start_message = String()
            start_message.data = "Start!"
            self.publisher.publish(start_message)
            rclpy.logging.get_logger("Task 1 Node").info('Message was sent. Quit working...')
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    task_1 = TrafficLightNode()
    
    rclpy.logging.get_logger("Task 1 Node").info('Working...')

    try:
        rclpy.spin(task_1)
    except SystemExit:
        rclpy.logging.get_logger("Task 1 Node").info('Done!')
    finally:
        task_1.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()