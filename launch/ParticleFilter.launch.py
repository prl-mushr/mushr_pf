#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    publish_tf = LaunchConfiguration("publish_tf")
    n_particles = LaunchConfiguration("n_particles")
    n_viz_particles = LaunchConfiguration("n_viz_particles")
    odometry_topic = LaunchConfiguration("odometry_topic")
    motor_state_topic = LaunchConfiguration("motor_state_topic")
    servo_state_topic = LaunchConfiguration("servo_state_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    laser_ray_step = LaunchConfiguration("laser_ray_step")
    exclude_max_range_rays = LaunchConfiguration("exclude_max_range_rays")
    max_range_meters = LaunchConfiguration("max_range_meters")
    car_name = LaunchConfiguration("car_name")

    particle_filter_node = Node(
        package="mushr_pf",
        executable="particle_filter",
        name="pf",
        output="screen",
        parameters=[
            {
                "publish_tf": ParameterValue(publish_tf, value_type=bool),
                "n_particles": ParameterValue(n_particles, value_type=int),
                "n_viz_particles": ParameterValue(n_viz_particles, value_type=int),
                "odometry_topic": odometry_topic,
                "motor_state_topic": motor_state_topic,
                "servo_state_topic": servo_state_topic,
                "scan_topic": scan_topic,
                "laser_ray_step": ParameterValue(laser_ray_step, value_type=int),
                "exclude_max_range_rays": ParameterValue(exclude_max_range_rays, value_type=bool),
                "max_range_meters": ParameterValue(max_range_meters, value_type=float),
                "car_name": car_name,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "publish_tf",
                default_value="false",
                description="Set to false in sim, set to true on real robot",
            ),
            DeclareLaunchArgument("n_particles", default_value="1000"),
            DeclareLaunchArgument("n_viz_particles", default_value="60"),
            DeclareLaunchArgument("odometry_topic", default_value="/car/odom"),
            DeclareLaunchArgument("motor_state_topic", default_value="/car/sensors/core"),
            DeclareLaunchArgument(
                "servo_state_topic",
                default_value="/car/sensors/servo_position_command",
            ),
            DeclareLaunchArgument("scan_topic", default_value="/car/scan"),
            DeclareLaunchArgument("laser_ray_step", default_value="18"),
            DeclareLaunchArgument("exclude_max_range_rays", default_value="true"),
            DeclareLaunchArgument("max_range_meters", default_value="11.0"),
            DeclareLaunchArgument("car_name", default_value="car"),
            GroupAction(
                [
                    PushRosNamespace(car_name),
                    particle_filter_node,
                ]
            ),
        ]
    )
