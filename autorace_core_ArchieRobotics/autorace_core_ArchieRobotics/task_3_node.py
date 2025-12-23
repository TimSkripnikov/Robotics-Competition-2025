#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
from tf_transformations import euler_from_quaternion
import math
import numpy as np


class Task3NavigationNode(Node):
    def __init__(self):
        super().__init__('task_3_node')
        
        # Подписки
        self.task_sub = self.create_subscription(
            String, '/comp/task_3', self.task_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Публикаторы
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.drive_mode_pub = self.create_publisher(String, '/comp/drive', 10)
        self.task_pub = self.create_publisher(Int32, '/comp/task', 10)
        self.finish_pub = self.create_publisher(String, '/robot_finish', 10)  # Новый публикатор
        
        # Параметры навигации
        self.active = False
        self.waypoints = [] 
        self.current_waypoint_index = 0
        self.position_tolerance = 0.1  # м
        self.angle_tolerance = 0.1     # рад
        
        self.waypoints = [
            [0.0101, 2.3741, 0.0],   
            [0.660460, 2.390891, 0.0],  
            [0.905015, 2.58005, 0.0],  
            [0.66094297, 2.93248646, 0.0],  
            [0.5784844, 3.0040847, 0.0],  
            [0.8852056, 3.5302577, 0.0],  
            [0.6100953, 3.79361879, 0.0],  
            [0.6114873, 4.4169717, 0.0]     
        ]
        
        # PID коэффициенты
        self.kp_linear = 0.5
        self.kp_angular = 0.8
        self.max_linear_speed = 0.1  
        self.max_angular_speed = 0.4
        
        # Текущая позиция и ориентация
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Флаги
        self.initial_position_received = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.finish_sent = False  # Флаг, что финишное сообщение отправлено
        
        # Таймер
        self.navigation_timer = None
        
        self.get_logger().info('Task 3 Node initialized (waiting for activation)')
    
    def task_callback(self, msg):
        if msg.data == "start_navigation" and not self.active:
            self.active = True
            
            # Отправляем команду отключить слежение за полосами
            mode_msg = String()
            mode_msg.data = "nothing"
            self.drive_mode_pub.publish(mode_msg)
            
            # Устанавливаем этап 3
            task_msg = Int32()
            task_msg.data = 3
            self.task_pub.publish(task_msg)
            
            # Ждем получения начальной позиции
            self.get_logger().info('Task 3 activated! Starting coordinate navigation...')
            
            # Запускаем навигацию сразу
            self.navigation_timer = self.create_timer(0.1, self.navigate)  # 10 Гц
    
    def odom_callback(self, msg):
        # Получаем позицию
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Получаем ориентацию 
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)
        
        # Запоминаем начальную позицию при первом получении
        if not self.initial_position_received:
            self.initial_x = self.current_x
            self.initial_y = self.current_y
            self.initial_position_received = True
    
    def navigate(self):
        if not self.active:
            return
        
        # Если все точки пройдены
        if self.current_waypoint_index >= len(self.waypoints):
            # Останавливаемся только один раз
            if not self.finish_sent:
                self.stop_robot()
                
                # Отправляем финишное сообщение
                finish_msg = String()
                finish_msg.data = "ArchieRobotics"
                self.finish_pub.publish(finish_msg)
                
                self.get_logger().info('All waypoints completed! Navigation finished.')
                self.finish_sent = True
            
            if self.navigation_timer:
                self.navigation_timer.cancel()
            return
        
        # Текущая целевая точка (АБСОЛЮТНЫЕ координаты)
        target = self.waypoints[self.current_waypoint_index]
        target_x, target_y, _ = target
        
        # Вычисляем расстояние до цели
        distance = math.sqrt((target_x - self.current_x)**2 + 
                           (target_y - self.current_y)**2)
        
        # Если достигли текущей точки, переходим к следующей
        if distance < self.position_tolerance:
            # БЕЗ ЛОГА для промежуточных точек
            
            self.current_waypoint_index += 1
            
            # Если все точки пройдены - не логируем здесь, логируем в следующей итерации
            return
        
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
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def publish_velocity(self, linear, angular):
        try:
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            self.cmd_vel_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing velocity: {e}')
    
    def stop_robot(self):
        try:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error stopping robot: {e}')
    
    def destroy_node(self):
        if self.navigation_timer:
            self.navigation_timer.cancel()
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = Task3NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()