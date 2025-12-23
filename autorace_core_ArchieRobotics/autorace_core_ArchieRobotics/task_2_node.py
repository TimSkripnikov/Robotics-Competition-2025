import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math


class CrossroadNode(Node):
    def __init__(self):
        super().__init__('task_2_node')

        # Подписки
        self.label_subscriber = self.create_subscription(
            String, '/yolo/label', self.label_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Публикаторы
        self.publisher = self.create_publisher(String, '/comp/task_2', 10)
        self.task_publisher = self.create_publisher(Int32, '/comp/task', 10)
        self.mode_publisher = self.create_publisher(String, '/comp/drive', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)


        # Две точки, по которым нужно проехать перед включением желтой линии
        self.left_turn_waypoints = [
            [0.411486, 1.0341104, 0.0],  
            [0.23827, 0.643149, 0.0]  
        ]
        self.current_waypoint_index = 0
        self.position_tolerance = 0.1  # м
        
        # Текущая позиция робота
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Состояния
        self.left_turn_active = False  # Активен ли поворот налево
        self.navigation_timer = None   # Таймер для навигации по точкам
        self.yellow_timer = None       # Таймер для переключения в режим обеих линий
        
        # Флаги для этапов поворота
        self.navigating_to_points = False
        self.yellow_mode_active = False
        
        # PID коэффициенты для навигации
        self.kp_linear = 0.5
        self.kp_angular = 0.8
        self.max_linear_speed = 0.1
        self.max_angular_speed = 0.4

    def odom_callback(self, msg):
        """
        Получение текущей позиции робота из одометрии
        """
        # Получаем позицию
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Получаем ориентацию (yaw) из кватерниона
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)

    def label_callback(self, msg):
        task = Int32()

        # ===== ПОВОРОТ НАПРАВО =====
        if msg.data == 'right':
            task.data = 2
            self.task_publisher.publish(task)

            mode = String()
            mode.data = 'both'
            self.mode_publisher.publish(mode)

            self.get_logger().info('Turn right - both lines mode')

        # ===== ПОВОРОТ НАЛЕВО =====
        elif msg.data == 'left' and not self.left_turn_active:
            task.data = 2
            self.task_publisher.publish(task)

            # Отключаем слежение за линиями
            mode_off = String()
            mode_off.data = 'nothing'
            self.mode_publisher.publish(mode_off)
            self.get_logger().info('Left turn initiated, navigating by coordinates')
            
            # Начинаем навигацию по координатам
            self.left_turn_active = True
            self.navigating_to_points = True
            self.current_waypoint_index = 0
            
            # Запускаем таймер для навигации по точкам
            self.navigation_timer = self.create_timer(0.1, self.navigate_left_turn)

        elif msg.data == 'work':
            task.data = 3
            self.task_publisher.publish(task)

            mode = String()
            mode.data = 'white'
            self.mode_publisher.publish(mode)

            self.get_logger().info('Preparing for task 3')

            result = String()
            result.data = 'Done'
            self.publisher.publish(result)

            rclpy.logging.get_logger(
                "Task 2 Node"
            ).info('Task completed. Quit working...')
            raise SystemExit

    def navigate_left_turn(self):
        """
        Навигация по координатам для поворота налево
        """
        if not self.navigating_to_points:
            return
        
        # Если все точки пройдены
        if self.current_waypoint_index >= len(self.left_turn_waypoints):
            # Завершаем навигацию по точкам
            self.navigating_to_points = False
            
            # Останавливаем робота
            self.stop_robot()
            
            # Отменяем таймер навигации
            if self.navigation_timer:
                self.navigation_timer.cancel()
                self.navigation_timer = None
            
            # Включаем режим следования по желтой линии
            self.enable_yellow_mode()
            return
        
        # Текущая целевая точка
        target = self.left_turn_waypoints[self.current_waypoint_index]
        target_x, target_y, _ = target
        
        # Вычисляем расстояние до цели
        distance = math.sqrt((target_x - self.current_x)**2 + 
                           (target_y - self.current_y)**2)
        
        # Если достигли текущей точки, переходим к следующей
        if distance < self.position_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}')
            self.current_waypoint_index += 1
            
            # Если все точки пройдены, выходим из функции
            if self.current_waypoint_index >= len(self.left_turn_waypoints):
                return
            
            # Обновляем цель для следующей итерации
            target = self.left_turn_waypoints[self.current_waypoint_index]
            target_x, target_y, _ = target
            distance = math.sqrt((target_x - self.current_x)**2 + 
                               (target_y - self.current_y)**2)
        
        # Вычисляем желаемый угол к цели
        target_angle = math.atan2(target_y - self.current_y, 
                                 target_x - self.current_x)
        
        # Разница между текущим углом и желаемым
        angle_error = self.normalize_angle(target_angle - self.current_yaw)
        
        # Вычисляем скорости
        linear_speed = min(self.kp_linear * distance, self.max_linear_speed)
        angular_speed = self.kp_angular * angle_error
        
        # Ограничиваем скорости
        angular_speed = max(min(angular_speed, self.max_angular_speed), 
                           -self.max_angular_speed)
        
        # Если угол сильно отклонен, сначала поворачиваем на месте
        if abs(angle_error) > 0.3:  # ~17 градусов
            linear_speed = 0.0
            angular_speed = 0.3 if angle_error > 0 else -0.3
        
        # Публикуем команду движения
        self.publish_velocity(linear_speed, angular_speed)

    def enable_yellow_mode(self):
        """
        Включение режима следования по желтой линии
        """
        mode = String()
        mode.data = 'yellow'
        self.mode_publisher.publish(mode)
        
        self.yellow_mode_active = True
        self.get_logger().info('Yellow line mode ON')
        
        # Через 5 секунд переключаемся на режим обеих линий
        self.yellow_timer = self.create_timer(5.0, self.switch_to_both)

    def switch_to_both(self):
        """
        Переключение в режим следования по обеим линиям
        """
        mode = String()
        mode.data = 'both'
        self.mode_publisher.publish(mode)
        
        self.get_logger().info('Switched to BOTH mode')
        
        # Отменяем таймер и сбрасываем флаги
        if self.yellow_timer:
            self.yellow_timer.cancel()
            self.yellow_timer = None
        
        self.yellow_mode_active = False
        self.left_turn_active = False

    def normalize_angle(self, angle):
        """
        Нормализация угла в диапазон [-π, π]
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_velocity(self, linear, angular):
        """
        Публикация команды скорости
        """
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """
        Остановка робота
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def destroy_node(self):
        """
        Корректное завершение работы ноды
        """
        if self.navigation_timer:
            self.navigation_timer.cancel()
        if self.yellow_timer:
            self.yellow_timer.cancel()
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    task_2 = CrossroadNode()

    rclpy.logging.get_logger("Task 2 Node").info('Working...')

    try:
        rclpy.spin(task_2)
    except SystemExit:
        rclpy.logging.get_logger("Task 2 Node").info('Done!')
    finally:
        task_2.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()