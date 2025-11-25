#!/usr/bin/env python3

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
parser.add_argument('--traj_ind', type=int, default=0, help="Trajectory Index")
parser.add_argument('--duration', type=float, default=1.0, help="Duration for slope calculation used in linear trajectory")
args_without_ros, ros_args = parser.parse_known_args()

class ObserverNode(Node):
    def __init__(self, args_cli):
        super().__init__('observer_node')
        self.args_cli = args_cli

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

        self.traj_list = [  constant_func(2.5),                                                                 # 0

                            linear_func(start=0, end=12, duration=self.args_cli.duration) ,                     # 1
                            linear_func(start=5, end=2, duration=self.args_cli.duration),                       # 2
                            linear_func(start=3.14, end=-3.14, duration=self.args_cli.duration),                # 3
                            linear_func(start=1, slope=0.5),                                                    # 4
                            linear_func(start=10.0, slope=-0.5),                                                # 5

                            trapezoidal_func(start=0, end=0, max_v=6.28, acc_time=4, duration=20.0),            # 6
                            trapezoidal_func(start=0, end=0, max_v=6.28, acc_max=2, duration=20.0),             # 7
                            trapezoidal_func(start=0, end=0, max_v=62.8, acc_max=0.5, duration=20.0),           # 8

                            sine_func(amplitude=3.14, frequency=0.2, offset=0.0, phase=0.0) ,                   # 9
                            sine_func(amplitude=1, frequency=2, offset=-3, phase=0.0),                          # 10
                            sine_func(amplitude=1, frequency=0.5, phase=np.pi/4, offset=0.0),                   # 11
                            sine_func(amplitude=1, frequency=2, freq_unit='round/min', offset=0.0, phase=0.0)   # 12
                         ]
        
        self.ref_wz = self.traj_list[args_cli.traj_ind]  # rad/s
        
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
    rclpy.init(args=ros_args)
    node = ObserverNode(args_without_ros)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
