#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from speech2speed_interface.msg import TwistSimpleStamped
from speech2speed_interface.srv import TwistTraj

class SchedulerNode(Node):
    """
    A ROS 2 node that receives a trajectory of Twist messages (with timestamps)
    via a service call and publishes them at the correct times using a timer.

    The node acts as a scheduler that ensures velocity commands are executed
    according to their specified timestamps.
    """
    def __init__(self):
        """
        Initialize the SchedulerNode.

        - Creates a timer to publish Twist messages at 100 Hz.
        - Creates a service to receive trajectory data (TwistTraj).
        - Initializes publishers, counters, and time tracking variables.
        """
        super().__init__('scheduler_node')

        # Timer runs at 100 Hz (every 0.01 seconds)
        self.create_timer(0.01, self.timer_callback)

        # Service to receive a list of Twist trajectories with timestamps
        self.create_service(TwistTraj, 'Upsampled_Traj', self.service_callback)

        # Publisher for sending upsampled Twist commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'upsampled_llm_cmd_vel', 10)

        # Counter for number of service requests received
        self.count = 0

        # List of Twist trajectories to be published over time
        self.twist_traj = []

        # Initialize time-related variables
        self.current_time = self.get_clock().now()
        self.current_trajtime = 0.0
        self.base_time = self.get_clock().now()

    def service_callback(self, req, res):
        """
        Handle incoming service requests containing trajectory data.

        Args:
            req (TwistTraj.Request): Request containing a list of Twist messages with timestamps.
            res (TwistTraj.Response): Response object to return success status.

        Returns:
            TwistTraj.Response: Indicates whether the trajectory was successfully received.

        Behavior:
        - Stores the received trajectory in self.twist_traj.
        - Resets the base time reference for playback.
        - Increments the request counter.
        - Logs the received trajectory size and success status.
        """
        res = TwistTraj.Response()
        try:
            # Log the current count
            self.get_logger().info(f"Count: {self.count}")

            # Store received trajectory
            self.twist_traj = req.twist_traj

            # Log number of received trajectory points
            self.get_logger().info(f"Received message: {len(req.twist_traj)} trajectory points.")

            # Update state variables
            self.count += 1
            self.base_time = self.get_clock().now()

            # Indicate success
            res.success = True
        except Exception as e:
            # Log failure and exception details
            self.get_logger().error(f"Failed to process trajectory: {e}")
            res.success = False
        return res
    
    def timer_callback(self):
        """
        Timer callback executed at 100 Hz to publish Twist messages at the correct times.

        Behavior:
        - Checks if a trajectory exists.
        - Compares current time against the next trajectory point’s target time.
        - Publishes the corresponding Twist message when its scheduled time arrives.
        - Removes the published point from the trajectory list.

        Notes:
        - Time comparison is done using nanoseconds for precision.
        - Logs each published Twist for debugging.
        """
        # Do nothing if no trajectory is queued
        if self.twist_traj:

            # Update current time and next scheduled time
            self.current_time = self.get_clock().now()
            self.current_trajtime = self.twist_traj[0].time.data

            # If enough time has passed, publish the next Twist
            if (self.current_time - self.base_time).nanoseconds >= (self.current_trajtime) * 1e9:

                # Retrieve and remove the next trajectory point
                next_twist = self.twist_traj.pop(0)
                twist_msg = Twist()

                # Copy motion parameters
                twist_msg.linear.x = next_twist.linear.x
                twist_msg.linear.y = next_twist.linear.y
                twist_msg.linear.z = next_twist.linear.z
                twist_msg.angular.x = next_twist.angular.x
                twist_msg.angular.y = next_twist.angular.y
                twist_msg.angular.z = next_twist.angular.z

                # Publish the Twist message
                self.cmd_vel_pub.publish(twist_msg)

                # Log the published command
                self.get_logger().info(
                    f"Published cmd_vel: "
                    f"linear=({twist_msg.linear.x}, {twist_msg.linear.y}, {twist_msg.linear.z}), "
                    f"angular=({twist_msg.angular.x}, {twist_msg.angular.y}, {twist_msg.angular.z})"
                )


def main(args=None):
    rclpy.init(args=args)
    node = SchedulerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
