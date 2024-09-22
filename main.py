import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.subscription = self.create_subscription(
            String,
            'dog_control',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        command = msg.data
        if command == 'forward':
            self.get_logger().info('Dog is moving forward.')
            # 模擬狗狗前進的行為
        elif command == 'backward':
            self.get_logger().info('Dog is moving backward.')
            # 模擬狗狗後退的行為
        elif command == 'stop':
            self.get_logger().info('Dog is stopping.')
            # 模擬狗狗停止的行為
        elif command == 'left':
            self.get_logger().info('Dog is turn left.')
            # 模擬狗狗停止的行為
        elif command == 'right':
            self.get_logger().info('Dog is turn right.')
            # 模擬狗狗停止的行為
        else:
            self.get_logger().info('Unknown command received.')

def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode()

    try:
        rclpy.spin(main_node)
    except KeyboardInterrupt:
        pass

    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
