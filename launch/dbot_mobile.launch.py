import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import add_debuggable_node, DeclareBooleanLaunchArg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare arguments and nodes
    declared_arguments = []
    nodes = []

    # Arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'dbot_mobile_urdf', 
            default_value=str(get_package_share_path('dbot_mobile') / 'urdf/dbot_mobile.urdf.xacro'), 
            description='Absolute path to robot urdf file'
        )
    )

    # Initialize Arguments
    dbot_urdf = LaunchConfiguration("dbot_mobile_urdf")

    # Robot State Publisher
    # Given the published joint states, publish tf for the robot links and the robot description
    robot_description = ParameterValue(
        Command(['xacro ', dbot_urdf]), 
        value_type=str
    )
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            {
                "robot_description" : robot_description,
            }
        ],
    )

    # Rviz
    # Run Rviz and load the default config to see the state of the move_group node
    rviz_config_path = get_package_share_path('dbot_mobile') / 'config/dbot_mobile_rviz.rviz'
    rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_config_path)],
            #parameters=[{'use_sim_time': False}]
            parameters=[]
    )

    # # Add nodes
    nodes.append(rsp_node)
    nodes.append(rviz2_node)
    
    return LaunchDescription(declared_arguments + nodes)