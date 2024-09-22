#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            executable='camera',
            name='camera_node',
            output='screen',
        ),
        Node(
            package='machine_dog_control',
            executable='machine_dog_control',
            name='machine_dog_controller',
            output='screen',
        ),
        Node(
            package='machine_dog_sim',
            executable='simulator',
            name='machine_dog_simulator',
            output='screen',
        ),
        # 添加其他需要启动的节点
    ])
