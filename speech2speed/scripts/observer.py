#!/home/absolutezeno/physical_ai/venv/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
# from speech2speed_interface import 
from geometry_msgs.msg import Twist
from speech2speed_interface.msg import TwistSimpleStamped
from speech2speed_interface.srv import TwistTraj
import numpy as np
import argparse

from speech2speed.utils import constant_func, linear_func, trapezoidal_func, sine_func

parser = argparse.ArgumentParser()
parser.add_argument('--traj_type', type=str, default="linear", help="Trajectory Type: constant, linear, trapezoidal, sine")
args_without_ros, ros_args = parser.parse_known_args()

class ObserverNode(Node):
    def __init__(self):
        super().__init__('observer_node')

        self.ref_traj_pub = self.create_publisher(Twist, 'ref_cmd_vel', 10)

        self.create_subscription(Twist,'upsampled_llm_cmd_vel', self.subscriber_callback, 10)

        self.ref_frequency = 25 # Hz      Frequency of reference trajectory
        self.duration = 100.0    # s       Duration of reference trajectory

        self.t = np.linspace(0, self.duration, int(self.duration * self.ref_frequency), endpoint=False)

        # Reference linear trajectory functions
        self.ref_vx = lambda t: 0.0  # m/s
        self.ref_vy = lambda t: 0.0  # m/s
        self.ref_vz = lambda t: 0.0  # m/s

        # Reference angular trajectory functions
        self.ref_wx = lambda t: 0.0  # rad/s
        self.ref_wy = lambda t: 0.0  # rad/s


        self.ref_wz = linear_func(start=0.0, slope=6.0, duration=2.0)  # rad/s
        
        self.ref_traj = []
        for ti in self.t:
            vx = self.ref_vx(ti)
            vy = self.ref_vy(ti)
            vz = self.ref_vz(ti)
            wx = self.ref_wx(ti)
            wy = self.ref_wy(ti)
            wz = self.ref_wz(ti)
            self.ref_traj.append((ti, vx, vy, vz, wx, wy, wz))

        self.count = 0
        self.error = 0.0

    def subscriber_callback(self, msg):
        msg_out = Twist()
        msg_out.linear.x = self.ref_traj[self.count][1]
        msg_out.linear.y = self.ref_traj[self.count][2]
        msg_out.linear.z = self.ref_traj[self.count][3]
        msg_out.angular.x = self.ref_traj[self.count][4]
        msg_out.angular.y = self.ref_traj[self.count][5]
        msg_out.angular.z = self.ref_traj[self.count][6]

        self.ref_traj_pub.publish(msg_out)

        error_vx = msg_out.linear.x - msg.linear.x
        error_vy = msg_out.linear.y - msg.linear.y
        error_vz = msg_out.linear.z - msg.linear.z
        error_wx = msg_out.angular.x - msg.angular.x
        error_wy = msg_out.angular.y - msg.angular.y
        error_wz = msg_out.angular.z - msg.angular.z

        error_norm = np.sqrt(error_vx**2 + error_vy**2 + error_vz**2 + error_wx**2 + error_wy**2 + error_wz**2)
        
        # Average error over time
        self.error = (self.error * self.count + error_norm) / (self.count + 1)
        self.get_logger().info(f"Tracking Error Norm: {error_norm:.4f}, Average Error: {self.error:.4f}")

        if self.count < len(self.ref_traj) - 1:
            self.count += 1
        else:
            self.count = 0
            self.error = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = ObserverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
