#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pandas as pd
from datetime import datetime
import os


class DataLogger(Node):
    def __init__(self):
        super().__init__('dataLogger')

        # Create timestamp for file naming
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"llm_cmd_vel_{now}.csv"

        # Save in user's home directory (NOT inside install/)
        self.output_dir = os.path.expanduser("~/data_logs")
        os.makedirs(self.output_dir, exist_ok=True)

        self.filepath = os.path.join(self.output_dir, self.filename)

        # Create empty DataFrame
        self.df = pd.DataFrame(columns=[
            "timestamp",
            "linear_x", "linear_y", "linear_z",
            "angular_x", "angular_y", "angular_z"
        ])

        # Subscription
        self.subscription = self.create_subscription(
            Twist,
            'llm_cmd_vel',
            self.listener_callback,
            10
        )

        self.get_logger().info(f"dataLogger started → logging to {self.filepath}")

    def listener_callback(self, msg: Twist):
        timestamp = self.get_clock().now().nanoseconds / 1e9

        # Create row
        row = {
            "timestamp": timestamp,
            "linear_x": msg.linear.x,
            "linear_y": msg.linear.y,
            "linear_z": msg.linear.z,
            "angular_x": msg.angular.x,
            "angular_y": msg.angular.y,
            "angular_z": msg.angular.z
        }

        # Append to DataFrame
        self.df.loc[len(self.df)] = row

        # Print to console
        self.get_logger().info(
            f"Logged: L=({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.linear.z:.2f}) "
            f"A=({msg.angular.x:.2f}, {msg.angular.y:.2f}, {msg.angular.z:.2f})"
        )

        # Periodic save every 50 rows
        if len(self.df) % 50 == 0:
            self.df.to_csv(self.filepath, index=False)

    def destroy_node(self):
        # Final save on exit
        self.df.to_csv(self.filepath, index=False)
        self.get_logger().info(f"Final CSV saved → {self.filepath}")
        super().destroy_node()

def export_string(text: str, file_name: str = "saved_log.txt"):
    """
    Append a line of text to a log file inside ~/data_logs/.
    Creates the directory and file if they don't exist.
    """
    # Resolve directory path
    output_dir = os.path.expanduser("~/data_logs")
    os.makedirs(output_dir, exist_ok=True)

    # Full path to file
    file_path = os.path.join(output_dir, file_name)

    # Append text with newline
    with open(file_path, "a") as f:
        f.write(text + "\n")

    return file_path

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("CTRL-C detected — shutting down and saving CSV...")
    finally:
        # Make sure the final data is saved
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
