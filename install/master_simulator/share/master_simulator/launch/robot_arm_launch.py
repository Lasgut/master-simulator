#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Get the path to the robot_arm package
    simulator_pkg_dir = get_package_share_directory('master_simulator')

    # Path to the robot's URDF file (generated from xacro)
    robot_urdf_path = os.path.join(simulator_pkg_dir, 'urdf', 'robot_arm.urdf')

    # Read the URDF file
    with open(robot_urdf_path, 'r') as infp:
        robot_urdf = infp.read()


    # Start Ignition Gazebo with an empty world
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen'
    )

    # Node to spawn the robot in Ignition Gazebo
    spawn_entity = Node(
        package='ros_ign_gazebo', 
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'robot_arm', '-allow_renaming', 'true'],
        output='screen'
    )

    # Node to publish the robot URDF to the 'robot_description' topic
    robot_description_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_urdf}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(use_sim_time),    
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([simulator_pkg_dir, 'config', 'robot_arm.rviz'])],
    )




    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        robot_description_publisher,
        joint_state_publisher_gui,

        # start rviz
        rviz_node,
        
        # uncomment to spawn in gazebo
        #ign_gazebo,
        #spawn_entity,
    ])