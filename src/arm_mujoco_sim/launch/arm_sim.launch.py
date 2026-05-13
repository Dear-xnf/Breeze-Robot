from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="arm_mujoco_sim",
            executable="arm_sim_node",
            name="arm_control_node",
            output="screen",
            parameters=[
                {"joint_num": 7},
                {"max_vel": 1.0}
            ]
        )
    ])