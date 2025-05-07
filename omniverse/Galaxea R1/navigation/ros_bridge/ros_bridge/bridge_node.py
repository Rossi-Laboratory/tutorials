import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import torch
import numpy as np

# Define PPO model architecture (2-layer MLP)
class PPOPolicy(torch.nn.Module):
    def __init__(self, input_dim, output_dim):
        super().__init__()
        self.fc = torch.nn.Sequential(
            torch.nn.Linear(input_dim, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, output_dim)
        )

    def forward(self, x):
        return self.fc(x)

# ROS2 Node to bridge PPO policy output to /cmd_vel
class PolicyBridge(Node):
    def __init__(self):
        super().__init__('policy_bridge')

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LIDAR scan
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Load trained PPO model from file
        self.model = PPOPolicy(input_dim=360, output_dim=2)
        self.model.load_state_dict(torch.load('model.pt', map_location=torch.device('cpu')))
        self.model.eval()

    def scan_callback(self, msg):
        # Skip invalid scan
        if len(msg.ranges) < 360:
            return

        # Convert scan data to tensor
        scan_array = np.array(msg.ranges[:360], dtype=np.float32)
        scan_tensor = torch.tensor(scan_array).unsqueeze(0)

        # Run inference with PPO model
        with torch.no_grad():
            action = self.model(scan_tensor).squeeze(0).numpy()

        # Publish the action as Twist message
        twist = Twist()
        twist.linear.x = float(action[0])
        twist.angular.z = float(action[1])
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PolicyBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()