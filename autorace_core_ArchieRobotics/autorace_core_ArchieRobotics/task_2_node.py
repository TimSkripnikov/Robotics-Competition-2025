import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist


class CrossroadNode(Node):
    def __init__(self):
        super().__init__('task_2_node')

        # Подписки
        self.label_subscriber = self.create_subscription(
            String, '/yolo/label', self.label_callback, 10
        )

        # Публикаторы
        self.publisher = self.create_publisher(String, '/comp/task_2', 10)
        self.task_publisher = self.create_publisher(Int32, '/comp/task', 10)
        self.mode_publisher = self.create_publisher(String, '/comp/drive', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Состояния
        self.turning = False
        self.left_turn_timer = None
        self.yellow_timer = None

    def label_callback(self, msg):
        task = Int32()

        # ===== ПОВОРОТ НАПРАВО =====
        if msg.data == 'right':
            task.data = 2
            self.task_publisher.publish(task)

            mode = String()
            mode.data = 'both'
            self.mode_publisher.publish(mode)

            self.get_logger().info('Turn right')

        # ===== ПОВОРОТ НАЛЕВО =====
        elif msg.data == 'left':
            task.data = 2
            self.task_publisher.publish(task)

            # Отключаем слежение за линиями
            mode_off = String()
            mode_off.data = 'nothing'
            self.mode_publisher.publish(mode_off)
            self.get_logger().info('Left turn initiated, drive disabled')

            # Поворот на месте
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.4
            self.cmd_pub.publish(twist)

            self.turning = True

            # Таймер окончания поворота
            if self.left_turn_timer is None:
                self.left_turn_timer = self.create_timer(
                    1.6, self.finish_left_turn
                )

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

    def finish_left_turn(self):
        if not self.turning:
            return

        # Останавливаем поворот
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        # Включаем ЖЁЛТУЮ линию
        mode = String()
        mode.data = 'yellow'
        self.mode_publisher.publish(mode)

        self.get_logger().info('Left turn completed, yellow mode ON')

        self.turning = False

        # Удаляем таймер поворота
        self.left_turn_timer.cancel()
        self.left_turn_timer = None

        # Таймер на 1.5 секунды жёлтого режима
        if self.yellow_timer is None:
            self.yellow_timer = self.create_timer(
                1.5, self.switch_to_both
            )

    def switch_to_both(self):
        mode = String()
        mode.data = 'both'
        self.mode_publisher.publish(mode)

        self.get_logger().info('Switched to BOTH mode')

        # Удаляем таймер
        self.yellow_timer.cancel()
        self.yellow_timer = None


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
