import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RobotDogController(Node):

    def __init__(self):
        super().__init__('robot_dog_controller')
        self.subscription = self.create_subscription(
            String,
            '/dog_control',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.image_subscription  # prevent unused variable warning

        self.bridge = CvBridge()

    def listener_callback(self, msg):
        if msg.data == 'Move forward':
            self.move_forward()
        elif msg.data == 'Move backward':
            self.move_backward()
        elif msg.data == 'Turn right':
            self.move_turnright()
        elif msg.data == 'Turn left':
            self.move_turnleft()


    def image_callback(self, msg):
        self.get_logger().info('Received an image!')
        # # Convert ROS Image message to OpenCV image
        # cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # # Optionally, save or display the image
        # cv2.imshow("Robot Camera", cv_image)
        # cv2.waitKey(1)

    def move_forward(self):
        self.get_logger().info("Moving forward")

    def move_backward(self):
        self.get_logger().info("Moving backward")
    
    def move_turnleft(self):
        self.get_logger().info("Moving left")
    
    def move_turnright(self):
        self.get_logger().info("Moving right")

def main(args=None):
    rclpy.init(args=args)
    robot_dog_controller = RobotDogController()
 
    try:
        rclpy.spin(robot_dog_controller)
        robot_dog_controller.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        robot_dog_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
