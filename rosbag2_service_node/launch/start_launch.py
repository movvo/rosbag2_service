"""
   Copyright 2023 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: Bernat Gaston
   Contact: bernat.gaston@movvo.eu
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
   config = os.path.join(
   get_package_share_directory('rosbag2_service'),
   'config',
   'params.yaml'
   )
   return LaunchDescription([
      Node(
         package='rosbag2_service',
         namespace='rosbag2_service',
         executable='rosbag2_service',
         parameters = [config]
      )
    ])