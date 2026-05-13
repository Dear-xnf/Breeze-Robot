from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable 
import os


def generate_launch_description():

    no_color_env = SetEnvironmentVariable(
    name="RCUTILS_COLORIZED_OUTPUT",
    value="0"
    )
    # 1.  moveit 的 demo.launch.py
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mockway_moveit_config"),
                "launch",
                "demo.launch.py"
            )
        ),

    )

    # 2.  moveit_relay 节点
    relay_node = Node(
        package="moveit_relay",
        executable="moveit_relay_node",  
        name="moveit_relay_node",
        output="screen",
        emulate_tty=True,
    )

    #3.
    motor_control_node=Node(
        package ="arm_motor_control",
        executable="arm_motor_control_node",  
        name="arm_motor_control_node",
        output="screen",
        emulate_tty=True,
        
    )

    return LaunchDescription([
        no_color_env,
        moveit_launch,
        relay_node,
        motor_control_node,

    ])