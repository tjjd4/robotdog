import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNewNode(Node):
    def __init__(self):
        super().__init__('my_new_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0  # 1秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('MyNewNode start！')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS 2! {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Plublish: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
