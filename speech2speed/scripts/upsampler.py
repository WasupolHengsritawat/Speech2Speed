#!/home/absolutezeno/physical_ai/venv/bin/python3

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
    Upsamples a signal using cubic spline interpolation.

    Args:
        x_old (np.ndarray): The original x-coordinates (e.g., time).
        y_old (np.ndarray): The original y-values of the signal.
        upsample_factor (int): The integer factor by which to increase the
                               sampling rate.

    Returns:
        tuple[np.ndarray, np.ndarray]: A tuple containing the new x-coordinates
                                       and the upsampled y-values.
    """
    # Create the cubic spline interpolation function
    cubic_spline_func = CubicSpline(x_old, y_old)

    # Calculate the new number of data points
    num_points_new = len(x_old) * upsample_factor

    # Define the new, higher-resolution x-axis
    x_new = np.linspace(x_old[0], x_old[-1], num_points_new)

    # Evaluate the spline at the new x-coordinates to get the upsampled y-values
    y_new = cubic_spline_func(x_new)

    return x_new, y_new

class UpsamplerNode(Node):
    def __init__(self):
        super().__init__('upsampler_node')

        self.create_service(TwistTraj,'Traj', self.service_callback)
        self.upsampled_traj_client = self.create_client(TwistTraj, 'Upsampled_Traj')

        self.upsample_factor = 2  # Define your upsample factor here

    def service_callback(self, req_in, res_in):
        res_in = TwistTraj.Response()
        try: 
            # Extract times and twists from the request
            times = np.array([vec.time.data for vec in req_in.twist_traj])
            twists = np.array([[vec.linear.x, vec.linear.y, vec.linear.z, vec.angular.x, vec.angular.y, vec.angular.z] for vec in req_in.twist_traj])

            # Upsample each component of the twist
            upsampled_t, upsampled_twists = cubic_upsample(times, twists, self.upsample_factor)
            [upsampled_vx, upsampled_vy, upsampled_vz, upsampled_wx, upsampled_wy, upsampled_wz] = upsampled_twists.transpose()

            # Prepare and send the upsampled trajectory to the next service
            req_out = TwistTraj.Request()
            for t, vx, vy, vz, wx, wy, wz in zip(upsampled_t, upsampled_vx, upsampled_vy, upsampled_vz, upsampled_wx, upsampled_wy, upsampled_wz):
                msg = TwistSimpleStamped()
                msg.time = Float32(data=t)
                msg.linear = Vector3(x=vx, y=vy, z=vz)
                msg.angular = Vector3(x=wx, y=wy, z=wz)
                req_out.twist_traj.append(msg)

            self.upsampled_traj_client.call_async(req_out)

            res_in.success = True
        except:
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
