import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit,OnProcessStart
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
            parameters=[]
    )
    
    #Ros2 Control
    controller_params_file = os.path.join(get_package_share_directory('dbot_mobile'), 'config', 'dbot_mobile_ros2_controllers.yaml')
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description':robot_description},
            controller_params_file
        ]
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Dbot Mobile Controller
    dbot_mobile_controller_spawner =   Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dbot_mobile_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Delay Start Controller Manager
    delayed_controller_manager = TimerAction(period=2.0,actions=[controller_manager])

    # Delay Start
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # Delay Start
    delay_dbot_mobile_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[dbot_mobile_controller_spawner],
        )
    )

    # # Add nodes
    nodes.append(rsp_node)
    nodes.append(rviz2_node)
    nodes.append(delayed_controller_manager)
    nodes.append(delay_joint_state_broadcaster_spawner)
    nodes.append(delay_dbot_mobile_controller_spawner)
    
    return LaunchDescription(declared_arguments + nodes)