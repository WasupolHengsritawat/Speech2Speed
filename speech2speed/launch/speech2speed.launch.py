#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,  OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch.logging

def generate_launch_description():

    launch_description = LaunchDescription()
    package_name = 'speech2speed'

    agent = Node(
    	package=package_name,
    	executable="agent.py"
    )

    upsampler = Node(
    	package=package_name,
    	executable="upsampler.py"
    )

    scheduler = Node(
    	package=package_name,
    	executable="scheduler.py"
    )

    scheduler_temp = Node(
    	package=package_name,
    	executable="scheduler_temp.py"
    )

    observer = Node(
    	package=package_name,
    	executable="observer.py"
    )

    # Add the actions to the launch description  
    launch_description.add_action(agent)
    launch_description.add_action(upsampler)
    launch_description.add_action(scheduler)
    launch_description.add_action(scheduler_temp)
    launch_description.add_action(observer)

    return launch_description