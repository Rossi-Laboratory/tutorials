import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Go1DriverNode(Node):
    def __init__(self):
        super().__init__('go1_driver_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'go1/command',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'go1/state', 10)
        self.get_logger().info('Go1 Driver Node initialized.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        # TODO: Replace this with real motor control logic
        response = Float32MultiArray()
        response.data = [0.0] * 12  # Fake joint state
        self.publisher_.publish(response)

if __name__ == '__main__':
    rclpy.init()
    node = Go1DriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
