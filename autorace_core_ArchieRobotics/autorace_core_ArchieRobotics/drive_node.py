import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Int32
import cv2
import os
import time
from cv_bridge import CvBridge
import numpy as np
from ament_index_python.packages import get_package_share_directory
from collections import deque


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        # Публикатор
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)   # Управление роботом
        
        # Подписки
        self.mode_subscriber = self.create_subscription(String, '/comp/drive', self.mode_callback, 10)     # Режим отслеживания полос
        self.task_subscriber = self.create_subscription(Int32, '/comp/task', self.task_callback, 10)       # Текущий этап
        self.image_subscriber = self.create_subscription(Image, '/color/image', self.image_callback, 10)   # Изображения с камер
        
        # Режим отслеживания полос, 0 - обе, 1 - желтая, 2 - белая, 3 - не отслеживать, -1 - остановиться, -2 - отключиться
        self.mode = 3
        self.task = 0

        self.bridge = CvBridge()

        # Коэффициенты pid-регулятора
        self.proportional = 0.0025
        self.integral = 0.0001
        self.differential = 0.0002
        self.target = 0.3

        # Ошибки для pid-регулятора
        self.error = 0                      # Накопленная интегральная ошибка
        self.errors = [0] * 30              # Стек предыдущих ошибок
        self.previous_error = 0             # Предыдущая ошибка
        self.previous_time = time.time()    # Время предыдущего измерения

        # Ограничения для линейной и угловой скоростей
        self.angular_max = 0.6
        self.linear_max = 0.08
        
        # Коэффициент пропорциональности линейной скорости к угловой
        self.velocity_coeff = 1.75
        
    
    def task_callback(self, msg):
        self.task = msg.data
        # Временная остановка drive_node на этапе 3
        if msg.data == 3:
            return 

    
    def mode_callback(self, msg):
        # Устанавливаем режим отслеживаения в зависимости от этапа
        if msg.data == 'yellow':
            self.mode = 1
        elif msg.data == 'white':
            self.mode = 2
        elif msg.data == 'both':
            self.mode = 0
        elif msg.data == 'nothing':
            self.mode = -1
        # Остановка робота в случае финиша на знаке парковки
        elif msg.data == 'finish':
            message = Twist()
            message.linear.x = 0.0
            message.angular.z = 0.0
            self.publisher.publish(message)
            
            rclpy.logging.get_logger("Drive Node").info('Message was sent. Quit working...')
            raise SystemExit
        
        self.get_logger().info(f"Mode \"{self.mode}\" enabled!")

    
    def image_callback(self, msg):
        if self.mode == -1:
            return 

        elif self.mode < 3:
            # Получение изображения
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Диапазоны цветов
            lower_white = np.array([0, 0, 240])
            upper_white = np.array([180, 30, 255])
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])

            # Определяем размер изображения и вырезаем нижнюю треть
            # 480, 848
            height, width, _ = cv_image.shape
            bottom_third = height - height // 4

            # Определяем точки для перспективной трансформации
            src_points = np.float32([[0, bottom_third], [width, bottom_third], [0, height], [width, height]])
            dst_points = np.float32([[0, 0], [width, 0], [0, height], [width, height]])

            # Вычисляем матрицу трансформации и преобразуем изображение
            M = cv2.getPerspectiveTransform(src_points, dst_points)
            warped_image = cv2.warpPerspective(cv_image, M, (width, height))

            # Преобразуем трансформированное изображение в HSV и находим маски по цветам
            warped_hsv_image = cv2.cvtColor(warped_image, cv2.COLOR_BGR2HSV)
            warped_mask_white = cv2.inRange(warped_hsv_image, lower_white, upper_white)
            warped_mask_yellow = cv2.inRange(warped_hsv_image, lower_yellow, upper_yellow)

            # Считаем количество белых и желтых пикселей
            cnt_white = np.sum(warped_mask_white == 255)
            cnt_yellow = np.sum(warped_mask_yellow == 255)

            # Обрабатываем желтые пиксели
            right_yellow_points = []    # Линия из самых правых точек желтой полосы
            cx_yellow = -1              # X центральной точки линии
            cy_yellow = -1              # Y центральной точки линии
            cnt_yellow_left = 0         # Количество желтых пикселей в левой половине изображения
            cnt_yellow_right = 0        # Количество желтых пикселей в правой половине изображения
            first_cx_yellow = -1        # X первого желтого пикселя
            last_cx_yellow = -1         # X последнего желтого пикселя
            first_cy_yellow = -1        # Y первого желтого пикселя
            last_cy_yellow = -1         # Y последнего желтого пикселя

            # Если есть желтые пиксели
            if cnt_yellow != 0:
                # Обрабатываем построчно пиксели изображения
                for y, row in enumerate(warped_mask_yellow):
                    # Получаем индексы желтых пикселей для каждой строчки изображения
                    non_zero_indices = np.nonzero(row)[0]

                    # Находим индексы и количество пикселей по обеим сторонам изображения
                    valid_indices = non_zero_indices[non_zero_indices < width//2]
                    no_valid_indices = non_zero_indices[non_zero_indices >= width//2]
                    cnt_yellow_left += len(valid_indices)
                    cnt_yellow_right += len(no_valid_indices)

                    # Если на левой половине изображения были желтые пиксели, находим самый правый
                    if len(valid_indices) > 0:
                        rightmost_index = valid_indices[-1]
                        right_yellow_points.append([rightmost_index, y])

                # Если список не пуст
                if right_yellow_points:
                    # Находим среднюю точку массива
                    right_yellow_points = np.array(right_yellow_points)
                    centroid_yellow = np.mean(right_yellow_points, axis=0)

                    # Находим координаты существующего пикселя, близкого к средней точки желтой линии
                    cy_yellow = int(centroid_yellow[1])
                    cx_yellow, cy_yellow = right_yellow_points[np.argmin(np.abs(right_yellow_points[:, 1] - cy_yellow))]

                    # Находим координаты первого и последнего пикселя крайней линии
                    first_cx_yellow, first_cy_yellow = right_yellow_points[0]
                    last_cx_yellow, last_cy_yellow = right_yellow_points[-1]

            
            # Обрабатываем белые пиксели
            left_white_points = []      # Линия из самых левых точек белой полосы
            cx_white = -1               # Y центральной точки линии
            cy_white = -1               # X центральной точки линии
            cnt_white_left = 0          # Количество белых пикселей в левой половине изображения
            cnt_white_right = 0         # Количество белых пикселей в правой половине изображения
            first_cx_white = -1         # X первого белого пикселя
            last_cx_white = -1          # X последнего белого пикселя
            first_cy_white = -1         # Y первого белого пикселя
            last_cy_white = -1          # Y последнего белого пикселя

            # Если есть желтые пиксели
            if cnt_white != 0:
                # Обрабатываем построчно пиксели изображения
                for y, row in enumerate(warped_mask_white):
                    # Получаем индексы белых пикселей для каждой строчки изображения
                    non_zero_indices = np.nonzero(row)[0]

                    # Находим индексы и количество пикселей по обеим сторонам изображения
                    valid_indices = non_zero_indices[non_zero_indices >= width//2]
                    no_valid_indices = non_zero_indices[non_zero_indices < width//2]
                    cnt_white_left += len(no_valid_indices)
                    cnt_white_right += len(valid_indices)

                    # Если на правой половине изображения были белые пиксели, находим самый левый
                    if len(valid_indices) > 0:
                        leftmost_index = valid_indices[0]
                        left_white_points.append([leftmost_index, y])

                # Если список не пуст
                if left_white_points:
                    # Находим среднюю точку массива
                    left_white_points = np.array(left_white_points)
                    centroid_white = np.mean(left_white_points, axis=0)
                    
                    # Находим координаты существующего пикселя, близкого к средней точки белой линии
                    cy_white = int(centroid_white[1])
                    cx_white, cy_white = left_white_points[np.argmin(np.abs(left_white_points[:, 1] - cy_white))]
                    
                    # Находим координаты первого и последнего пикселя крайней линии
                    first_cx_white, first_cy_white = left_white_points[0]
                    last_cx_white, last_cy_white = left_white_points[-1]

            # Определяем режим движения на основе обнаруженных пикселей
            real_mode = 0

            # Если центры обеих полос найдены, то отслеживаем обе
            if cy_white > -1 and cy_yellow > -1:
                # ava_mode = 0
                real_mode = self.mode

                # Для этапа 2
                if self.task == 2:
                    # Если более 10% белых пикселей находятся в левой части изображения
                    if cnt_white_left / (cnt_white_right + cnt_white_left + 1) > 0.1:
                        real_mode = 2
                    # Если более 10% желтых пикселей находятся в правой части изображения
                    elif cnt_yellow_right / (cnt_yellow_left + cnt_yellow_right + 1) > 0.1:
                        real_mode = 1

            # Если найден центр только для желтой линии, то отслеживаем только ее
            elif cy_white == -1 and cy_yellow > -1:
                real_mode = 1
            
            # Если найден центр только для белой линии, то отслеживаем только ее
            elif cy_white > -1 and cy_yellow == -1:
                real_mode = 2
            
            # Если центры не найдены, то ничего не отслеживаем
            else:
                real_mode = 3

            # Угол и ошибка для управления движением
            angle = 0
            error = 0
            
            # Если отслеживаем обе линии:
            if real_mode == 0:
                # Находим прямую между центрами двух полос
                cv2.circle(warped_image, (cx_white, cy_white), 5, (255, 0, 0), -1)
                cv2.circle(warped_image, (cx_yellow, cy_yellow), 5, (0, 0, 255), -1)
                cv2.line(warped_image, (cx_white, cy_white), (cx_yellow, cy_yellow), (0, 255, 0), 2)

                # Находим центр прямой
                cx_center = (cx_white + cx_yellow) // 2
                cy_center = (cy_white + cy_yellow) // 2

                # Находим среднюю прямую для двух полос
                cx_first = int((first_cx_white + first_cx_yellow) // 2)
                cy_first = int((first_cy_white + first_cy_yellow) // 2)
                cx_last = int((last_cx_white + last_cx_yellow) // 2)
                cy_last = int((last_cy_white + last_cy_yellow) // 2)

                # Определяем наклон средней прямой относительно прямой между центрами
                angle = abs((cx_first - cx_last) / (cy_first - cy_last + 1))
                
                # Определяем отклонение центра прямой и центра изображения
                error = (width // 2 - cx_center)

                # Выводим изображение с прямыми
                cv2.line(warped_image, (cx_first, cy_first), (cx_last, cy_last), (225, 0, 225), 2)
                cv2.circle(warped_image, (cx_first, cy_first), 5, (255, 0, 100), -1)
                cv2.circle(warped_image, (cx_last, cy_last), 5, (100, 0, 255), -1)
                cv2.circle(warped_image, (cx_center, cy_center), 5, (0, 255, 0), -1)
                cv2.imshow("Center", warped_image)
                cv2.waitKey(1)

            # Если отслеживаем только желтую:
            elif real_mode == 1:
                # Создаем фиктивную точку на расстоянии от центра желтой полосы для замены центра белой полосы
                cv2.circle(warped_image, (cx_yellow + 320, height//2), 5, (255, 0, 0), -1)
                cv2.circle(warped_image, (cx_yellow, cy_yellow), 5, (0, 0, 255), -1)
                cv2.line(warped_image, (cx_yellow + 320, height//2), (cx_yellow, cy_yellow), (0, 255, 0), 2)

                # Находим центр прямой
                cx_center = cx_yellow + 160
                cy_center = (cy_yellow + height//2) // 2

                # Находим среднюю прямую для двух полос
                cx_first = int(first_cx_yellow + 160)
                cy_first = int(first_cy_yellow // 2)
                cx_last = int(last_cx_yellow + 160)
                cy_last = int((last_cy_yellow + height) // 2)

                # Определяем наклон средней прямой относительно прямой между центрами
                angle = abs((cx_first - cx_last) / (cy_first - cy_last + 1))

                # Определяем отклонение центра прямой и центра изображения
                error = (width // 2 - cx_center)

                # Выводим изображение с прямыми
                cv2.line(warped_image, (cx_first, cy_first), (cx_last, cy_last), (225, 0, 225), 2)
                cv2.circle(warped_image, (cx_first, cy_first), 5, (255, 0, 100), -1)
                cv2.circle(warped_image, (cx_last, cy_last), 5, (100, 0, 255), -1)
                cv2.circle(warped_image, (cx_center, cy_center), 5, (0, 255, 0), -1)
                cv2.imshow("Center", warped_image)
                cv2.waitKey(1)

            # Если отслеживаем только белую:
            elif real_mode == 2:
                # Создаем фиктивную точку на расстоянии от центра белой полосы для замены центра желтой полосы
                cv2.circle(warped_image, (cx_white, cy_white), 5, (255, 0, 0), -1)
                cv2.circle(warped_image, (cx_white - 320, height//2), 5, (0, 0, 255), -1)
                cv2.line(warped_image, (cx_white - 320, height//2), (cx_white, cy_white), (0, 255, 0), 2)

                # Находим центр прямой
                cx_center = cx_white - 160
                cy_center = (240 + cy_white) // 2

                # Находим среднюю прямую для двух полос
                cx_first = int(first_cx_white - 160)
                cy_first = int(first_cy_white // 2) 
                cx_last = int(last_cx_white - 160)
                cy_last = int((last_cy_white + height) // 2)

                # Определяем наклон средней прямой относительно прямой между центрами
                angle = abs((cx_first - cx_last) / (cy_first - cy_last + 1))
                
                # Определяем отклонение центра прямой и центра изображения
                error = (width // 2 - cx_center)

                # Выводим изображение с прямыми
                cv2.line(warped_image, (cx_first, cy_first), (cx_last, cy_last), (225, 0, 225), 2)
                cv2.circle(warped_image, (cx_first, cy_first), 5, (255, 0, 100), -1)
                cv2.circle(warped_image, (cx_last, cy_last), 5, (100, 0, 255), -1)
                cv2.circle(warped_image, (cx_center, cy_center), 5, (0, 255, 0), -1)
                cv2.imshow("Center", warped_image)
                cv2.waitKey(1)
            
            # Если не движемся
            else:
                error = 0

            self.move(error, angle)


    def move(self, error, angle):
        # Вычисление ошибки
        current_error = error
        error_integral = sum(self.errors) + error
        error_differential = error - self.previous_error

        # Вычисление угловой скорости
        angular_velocity = current_error * self.proportional + error_integral * self.integral + error_differential * self.differential
        angular_velocity = max(min(angular_velocity, self.angular_max), -self.angular_max)
        
        # Обновляем стек ошибок
        self.errors.pop(0)
        self.errors.append(error)
        self.previous_error = error
        
        angle_loss = 4      # Коэффициент влияния угла наклона прямой
        angular_loss = 2    # Коэффициент влияния угловой скорости

        # Уменьшаем линейную скорость пропорционально угловой скорости
        linear_velocity = max(self.linear_max, self.target / (np.exp(angular_loss * abs(angular_velocity) / self.angular_max) * np.exp(angle_loss * angle)))

        message = Twist()
        message.linear.x = linear_velocity
        message.angular.z = angular_velocity

        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    drive = DriveNode()
    rclpy.logging.get_logger("Drive Node").info('Working...')

    try:
        rclpy.spin(drive)
    except SystemExit:
        rclpy.logging.get_logger("Drive Node").info('Done!')
    finally:
        drive.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()