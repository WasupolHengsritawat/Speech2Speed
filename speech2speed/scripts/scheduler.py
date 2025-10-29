#!/home/absolutezeno/physical_ai/venv/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
# from speech2speed_interface import 
from geometry_msgs.msg import Twist
from speech2speed_interface.msg import TwistSimpleStamped
from speech2speed_interface.srv import TwistTraj


class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')

        self.create_timer(0.01, self.timer_callback) # 100 Hz timer
        self.create_service(TwistTraj,'Upsampled_Traj', self.service_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, 'llm_cmd_vel', 10)
        self.count = 0

        self.twist_traj = []

        self.current_time = self.get_clock().now()
        self.current_trajtime = 0.0

        self.base_time = self.get_clock().now()

    def service_callback(self, req, res):
        res = TwistTraj.Response()
        try: 
            self.get_logger().info(f"Count: {self.count}")
            self.twist_traj = req.twist_traj
            for vec in req.twist_traj:
                self.get_logger().info(f"Received message: time={vec.time.data}, linear=({vec.linear.x}, {vec.linear.y}, {vec.linear.z}), angular=({vec.angular.x}, {vec.angular.y}, {vec.angular.z})")
            self.count += 1
            self.base_time = self.get_clock().now()
            res.success = True
        except:
            res.success = False
        return res
    
    def timer_callback(self):
        if self.twist_traj:
            
            self.current_time = self.get_clock().now()
            self.current_trajtime = self.twist_traj[0].time.data

            if (self.current_time - self.base_time).nanoseconds >= (self.current_trajtime) * 1e9:
                next_twist = self.twist_traj.pop(0)
                twist_msg = Twist()

                twist_msg.linear.x = next_twist.linear.x
                twist_msg.linear.y = next_twist.linear.y
                twist_msg.linear.z = next_twist.linear.z
                twist_msg.angular.x = next_twist.angular.x
                twist_msg.angular.y = next_twist.angular.y
                twist_msg.angular.z = next_twist.angular.z
                self.cmd_vel_pub.publish(twist_msg)

                self.get_logger().info(f"Published cmd_vel: linear=({twist_msg.linear.x}, {twist_msg.linear.y}, {twist_msg.linear.z}), angular=({twist_msg.angular.x}, {twist_msg.angular.y}, {twist_msg.angular.z})")

def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
