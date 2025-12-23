import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


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

        # Публикаторы
        self.label_pub = self.create_publisher(String, '/yolo/label', 10)
        # self.finish_pub = self.create_publisher(String, '/robot_finish', 10)
        self.task3_publisher = self.create_publisher(String, '/comp/task_3', 10)


        self.bridge = CvBridge()

        # Тайминги
        self.start_time = None
        self.direction_time = None

        # Состояния
        self.active = False
        self.direction_sent = False
        self.task3_sent = False

    def start_callback(self, msg):
        self.get_logger().info("Green light detected. Timer started.")
        self.start_time = time.time()

    def image_callback(self, msg):
        if self.start_time is None:
            return

        now = time.time()

        # === ФАЗА 1: определение направления ===
        if not self.direction_sent:
            if now - self.start_time >= 19.5:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                direction = detect_turn_direction(cv_image)

                out = String()
                out.data = direction
                self.label_pub.publish(out)

                self.direction_sent = True
                self.direction_time = now

                self.get_logger().info(f"Direction detected and published: {direction}")

        # === ФАЗА 2: завершение робота ===
        else:
            # if now - self.direction_time >= 40.0:
            #     finish_msg = String()
            #     finish_msg.data = "ArchieRobotics"
            #     self.finish_pub.publish(finish_msg)

            #     self.get_logger().info("Finish message published. Shutting down node.")
            #     raise SystemExit
            if now - self.direction_time >= 40.0:
                task3_msg = String()
                task3_msg.data = "start_navigation"
                self.task3_publisher.publish(task3_msg)  # Новый publisher
                self.task3_sent = True
                self.get_logger().info("Task 3 activation signal sent")
                raise SystemExit



def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
