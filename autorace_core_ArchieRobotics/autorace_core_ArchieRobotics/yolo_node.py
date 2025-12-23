import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion
import cv2
import numpy as np
import math


def detect_turn_direction(
    img_bgr: np.ndarray,
    threshold_value=60,
    top_percent=0.4,
) -> str:
    # 1. Сглаживание
    img_blur = cv2.GaussianBlur(img_bgr, (5, 5), 0)

    # 2. HSV
    hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)

    # 3. Маска синего
    lower_blue = np.array([90, 60, 60])
    upper_blue = np.array([140, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 4. Контур знака
    contours, _ = cv2.findContours(
        blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        roi = img_bgr.copy()
        mask_roi = np.ones(img_bgr.shape[:2], dtype=np.uint8) * 255
    else:
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        roi = img_bgr[y:y+h, x:x+w]

        mask_roi = np.zeros((h, w), dtype=np.uint8)
        shifted = c - [x, y]
        cv2.drawContours(mask_roi, [shifted], -1, 255, -1)

    # 5. Серый + маска
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    gray = cv2.bitwise_and(gray, gray, mask=mask_roi)

    # 6. Бинаризация
    _, bw = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)

    # 7. Анализ верхней части
    h, w = bw.shape
    top = bw[:int(h * top_percent), :]

    left_white = np.sum(top[:, :w//2] == 255)
    right_white = np.sum(top[:, w//2:] == 255)

    return "left" if left_white > right_white else "right"


class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # Подписки
        self.start_sub = self.create_subscription(
            String, '/comp/task_1', self.start_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, '/color/image', self.image_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Публикаторы
        self.label_pub = self.create_publisher(String, '/yolo/label', 10)
        self.task3_publisher = self.create_publisher(String, '/comp/task_3', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # Новый публикатор для остановки

        self.bridge = CvBridge()

        # Текущая позиция робота
        self.current_x = 0.0
        self.current_y = 0.0
        
        self.sign_check_point = [0.77049, 1.12266]  
        self.work_check_point = [-0.36198823, 2.37605868]  
        self.tolerance = 0.08

        # Состояния
        self.active = False
        self.direction_sent = False
        self.task3_sent = False
        self.sign_check_done = False
        self.work_check_done = False
        self.stopped_at_sign = False  # Флаг остановки на точке знака

        # Для хранения изображения для анализа
        self.last_image = None

    def start_callback(self, msg):
        self.get_logger().info("Green light detected. YOLO node activated.")
        self.active = True

    def odom_callback(self, msg):
        """
        Получение текущей позиции робота из одометрии
        """
        if not self.active:
            return
            
        # Получаем позицию
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Проверяем точки
        self.check_points()

    def check_points(self):
        """
        Проверка достижения контрольных точек
        """
        # Проверяем точку знака
        if not self.sign_check_done:
            distance_to_sign = math.sqrt(
                (self.current_x - self.sign_check_point[0])**2 + 
                (self.current_y - self.sign_check_point[1])**2
            )
            
            if distance_to_sign <= self.tolerance:
                self.get_logger().info(f"Reached sign check point at ({self.current_x:.3f}, {self.current_y:.3f})")
                self.sign_check_done = True
                
                # ОСТАНАВЛИВАЕМ РОБОТА
                self.stop_robot()
                self.stopped_at_sign = True
                
                # Если есть изображение, анализируем его для определения направления
                if self.last_image is not None and not self.direction_sent:
                    self.analyze_sign()

        # Проверяем точку работ (только если уже прошли точку знака)
        if self.sign_check_done and not self.work_check_done:
            distance_to_work = math.sqrt(
                (self.current_x - self.work_check_point[0])**2 + 
                (self.current_y - self.work_check_point[1])**2
            )
            
            if distance_to_work <= self.tolerance:
                self.get_logger().info(f"Reached work check point at ({self.current_x:.3f}, {self.current_y:.3f})")
                self.work_check_done = True
                
                # Отправляем сигнал для Task 3
                if not self.task3_sent:
                    self.send_task3_signal()

    def image_callback(self, msg):
        """
        Получение изображения с камеры
        """
        if not self.active:
            return
            
        # Сохраняем последнее изображение для анализа
        self.last_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Если достигли точки знака и еще не отправили направление, анализируем сразу
        if self.sign_check_done and not self.direction_sent:
            self.analyze_sign()

    def analyze_sign(self):
        """
        Анализ изображения для определения направления знака
        """
        if self.last_image is None:
            self.get_logger().warn("No image available for sign analysis")
            return
            
        try:
            direction = detect_turn_direction(self.last_image)
            
            out = String()
            out.data = direction
            self.label_pub.publish(out)
            
            self.direction_sent = True
            self.get_logger().info(f"Direction detected and published: {direction}")
            
            # Очищаем изображение, чтобы не анализировать повторно
            self.last_image = None
            
        except Exception as e:
            self.get_logger().error(f"Error analyzing sign: {e}")

    def send_task3_signal(self):
        """
        Отправка сигнала для активации Task 3
        """
        task3_msg = String()
        task3_msg.data = "start_navigation"
        self.task3_publisher.publish(task3_msg)
        
        self.task3_sent = True
        self.get_logger().info("Task 3 activation signal sent")
        
        self.get_logger().info("YOLO node completed its work")

    def stop_robot(self):
        """
        Остановка робота
        """
        try:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.get_logger().info("Robot stopped at sign check point")
        except Exception as e:
            self.get_logger().error(f"Error stopping robot: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()