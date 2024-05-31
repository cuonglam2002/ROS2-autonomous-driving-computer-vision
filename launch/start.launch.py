
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


  return LaunchDescription([


        Node(
                package='self_driving',
                executable='cam_node',
                name='camera',
                output='screen'),

        Node(
                package='self_driving',
                executable='computer_vision_node',
                name='img_process',
                output='screen'),     
        Node(
                package='self_driving',
                executable='vel_wheel_node',
                name='driver',
                output='screen'),
        # Node(
        #         package='self_driving',
        #         executable='traffic_signal_node',
        #         name='traffic_signal',
        #         output='screen'),    

    ])
