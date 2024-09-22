import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template, request
from threading import Thread
from werkzeug.serving import make_server
import os

class FlaskServer(Thread):
    def __init__(self, app, host='0.0.0.0', port=5000):
        Thread.__init__(self)
        self.server = make_server(host, port, app)
        self.ctx = app.app_context()
        self.ctx.push()
        self.daemon = True
        self.is_running = False

    def run(self):
        self.is_running = True
        self.server.serve_forever()

    def shutdown(self):
        if self.is_running:
            self.server.shutdown()
            self.is_running = False

class DogController(Node):
    def __init__(self):
        super().__init__('dog_controller')
        self.publisher_ = self.create_publisher(String, 'dog_control', 10)
        self.get_logger().info('DogController node has been started.')

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def create_flask_app(node):
    template_dir = os.path.abspath(os.path.join(os.getcwd(), 'src/machine_dog_control/templates'))
    app = Flask(__name__, template_folder=template_dir)

    @app.route('/', methods=['GET'])
    def index():
        return render_template('index.html')

    @app.route('/move', methods=['POST'])
    def move():
        direction = request.json.get('direction')
        if direction:
            node.publish_command(direction)
        return '', 200

    return app

def ros2_thread(node):
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass  

def main(args=None):
    rclpy.init(args=args)
    node = DogController()

    try:
        flask_app = create_flask_app(node)
        flask_server = FlaskServer(flask_app)
        flask_server.start()
        node.get_logger().info('Flask server is running.')

        # start ROS 2 thread
        ros_thread = Thread(target=ros2_thread, args=(node,), daemon=True)
        ros_thread.start()

        # run Flask 
        while ros_thread.is_alive():
            ros_thread.join(timeout=1)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down Flask server...')
        flask_server.shutdown()
        node.get_logger().info('Flask server shut down.')

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
