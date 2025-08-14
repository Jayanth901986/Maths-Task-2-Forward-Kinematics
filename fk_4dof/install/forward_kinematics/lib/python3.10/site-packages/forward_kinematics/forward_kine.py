import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import numpy as np

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')
        
        # Subscriber to joint states (expecting j1, j2, j3, j4 angles in radians)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)
        
        # Publisher for end-effector position
        self.publisher_ = self.create_publisher(PointStamped, '/end_effector_position', 10)
        
        # Link length parameter
        self.L = 1.0  # default length of each link (meters)
        
    def rot_z(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [c, -s,  0, 0],
            [s,  c,  0, 0],
            [0,  0,  1, 0],
            [0,  0,  0, 1]
        ])

    def rot_y(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [ c, 0, s, 0],
            [ 0, 1, 0, 0],
            [-s, 0, c, 0],
            [ 0, 0, 0, 1]
        ])

    def trans_x(self, L):
        return np.array([
            [1, 0, 0, L],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, j1, j2, j3, j4):
        T1 = self.rot_z(j1) @ self.trans_x(self.L)
        T2 = self.rot_y(j2) @ self.trans_x(self.L)
        T3 = self.rot_z(j3) @ self.trans_x(self.L)
        T4 = self.rot_y(j4) @ self.trans_x(self.L)
        T = T1 @ T2 @ T3 @ T4
        pos = T[0:3, 3]
        return pos

    def joint_callback(self, msg):
        # Assuming msg.position has at least 4 positions for j1, j2, j3, j4
        if len(msg.position) < 4:
            self.get_logger().error('Received joint_states with insufficient positions')
            return
        
        j1, j2, j3, j4 = msg.position[:4]
        end_effector_pos = self.forward_kinematics(j1, j2, j3, j4)
        
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'base_link'  # Adjust as per your TF frame setup
        point_msg.point.x = float(end_effector_pos[0])
        point_msg.point.y = float(end_effector_pos[1])
        point_msg.point.z = float(end_effector_pos[2])
        
        self.publisher_.publish(point_msg)
        self.get_logger().info(f'Published end-effector position: {point_msg.point.x:.3f}, {point_msg.point.y:.3f}, {point_msg.point.z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

