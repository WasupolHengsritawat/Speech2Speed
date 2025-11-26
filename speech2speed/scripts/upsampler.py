#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
# from speech2speed_interface import 
from geometry_msgs.msg import Twist
from speech2speed_interface.msg import TwistSimpleStamped
from speech2speed_interface.srv import TwistTraj
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

import numpy as np
from scipy.interpolate import CubicSpline

def cubic_upsample(x_old, y_old, upsample_factor):
    """
    Perform cubic spline interpolation to upsample a signal.

    Args:
        x_old (np.ndarray): The original x-coordinates (e.g., time stamps).
        y_old (np.ndarray): The original y-values of the signal. Can be 1D or 2D.
        upsample_factor (int): The integer factor by which to increase the sampling rate.

    Returns:
        tuple[np.ndarray, np.ndarray]:
            - x_new: The new, higher-resolution x-coordinates.
            - y_new: The interpolated y-values at the new sample points.

    Behavior:
        - Constructs a cubic spline from the original signal.
        - Generates a new x-axis with higher sampling density.
        - Evaluates the spline at these new points for smooth interpolation.
    """
    # Create a cubic spline interpolation function based on the original data
    cubic_spline_func = CubicSpline(x_old, y_old)

    # Compute the number of new data points after upsampling
    num_points_new = len(x_old) * upsample_factor

    # Generate the new x-values (higher resolution) across the same domain
    x_new = np.linspace(x_old[0], x_old[-1], num_points_new)

    # Evaluate the spline to compute the upsampled signal values
    y_new = cubic_spline_func(x_new)

    return x_new, y_new


class UpsamplerNode(Node):
    """
    A ROS 2 node that receives a trajectory of Twist messages and produces
    an upsampled version of that trajectory using cubic spline interpolation.

    This node acts as an intermediary processor that:
    1. Receives a low-resolution trajectory.
    2. Interpolates additional intermediate points.
    3. Sends the refined trajectory to another node for time-synchronized execution.
    """
    def __init__(self):
        """
        Initialize the UpsamplerNode.

        - Creates a service named 'Traj' to receive incoming trajectories.
        - Creates clients to forward both the upsampled and original trajectories.
        - Defines the upsampling factor for trajectory refinement.
        """
        super().__init__('upsampler_node')

        # Service to receive the original trajectory
        self.create_service(TwistTraj, 'Traj', self.service_callback)

        # Client to send the upsampled trajectory to the scheduler node
        self.upsampled_traj_client = self.create_client(TwistTraj, 'Upsampled_Traj')

        # Client to send the original trajectory to another service (for logging/debug)
        self.temp_traj_client = self.create_client(TwistTraj, 'Traj_temp')

        # Factor controlling interpolation density (1 = no upsampling)
        self.upsample_factor = 1

    def service_callback(self, req_in, res_in):
        """
        Handle incoming trajectory data, perform cubic upsampling,
        and forward the results to downstream services.

        Args:
            req_in (TwistTraj.Request): The input trajectory request containing a list of TwistSimpleStamped messages.
            res_in (TwistTraj.Response): The response object to report success or failure.

        Returns:
            TwistTraj.Response: Response indicating if upsampling and forwarding succeeded.

        Behavior:
            - Extracts time and velocity data from the incoming trajectory.
            - Performs cubic spline interpolation to create a denser trajectory.
            - Constructs new Twist messages with upsampled data.
            - Sends the refined trajectory asynchronously to the target service.
            - Optionally sends the original trajectory to another topic for monitoring.
        """
        res_in = TwistTraj.Response()
        try:
            # Extract time and twist data arrays from incoming trajectory
            times = np.array([vec.time.data for vec in req_in.twist_traj])
            twists = np.array([
                [vec.linear.x, vec.linear.y, vec.linear.z,
                 vec.angular.x, vec.angular.y, vec.angular.z]
                for vec in req_in.twist_traj
            ])

            # Apply cubic spline upsampling to the time and twist arrays
            upsampled_t, upsampled_twists = cubic_upsample(times, twists, self.upsample_factor)

            # Separate interpolated components for clarity
            [upsampled_vx, upsampled_vy, upsampled_vz,
             upsampled_wx, upsampled_wy, upsampled_wz] = upsampled_twists.transpose()

            # Build a new request containing upsampled Twist messages
            req_out = TwistTraj.Request()
            for t, vx, vy, vz, wx, wy, wz in zip(
                upsampled_t, upsampled_vx, upsampled_vy, upsampled_vz,
                upsampled_wx, upsampled_wy, upsampled_wz
            ):
                msg = TwistSimpleStamped()
                msg.time = Float32(data=t)
                msg.linear = Vector3(x=vx, y=vy, z=vz)
                msg.angular = Vector3(x=wx, y=wy, z=wz)
                req_out.twist_traj.append(msg)

            # Send upsampled and original trajectories asynchronously
            self.upsampled_traj_client.call_async(req_out)
            self.temp_traj_client.call_async(req_in)

            # Indicate success
            res_in.success = True
        except Exception as e:
            # Log exception and indicate failure
            self.get_logger().error(f"Upsampling failed: {e}")
            res_in.success = False

        return res_in

    
def main(args=None):
    rclpy.init(args=args)
    node = UpsamplerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
