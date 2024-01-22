import os
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
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
            default_value=str(get_package_share_path('dbot_mobile') / 'urdf/dbot_mobile_gazebo_ros2_control.urdf.xacro'), 
            description='Absolute path to robot urdf file'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'dbot_mobile_gazebo_world', 
            default_value=str(get_package_share_path('dbot_mobile') / 'world/obstacles.world'), 
            description='Absolute path to gazenp world file'
        )
    )

    # Initialize Arguments
    dbot_urdf = LaunchConfiguration("dbot_mobile_urdf")
    gazebo_world = LaunchConfiguration("dbot_mobile_gazebo_world")

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
                "use_ros2_control:" : True,
                "use_sim_time" : True,
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
            parameters=[{'use_sim_time': True}]
            #parameters=[]
    )

    # Gazebo
    # Include the Gazebo launch file, provided by the gazebo_ros package
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    #gazebo_world = os.path.joint(get_package_share_directory('dbot_mobile'), 'world', 'obstacles.world')
    gazebo_params_file = os.path.join(get_package_share_directory('dbot_mobile'), 'config', 'gazebo_params.yaml')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        launch_arguments={
            'extra_gazebo_args':'--ros-args --params-file' + gazebo_params_file,
            'world' : gazebo_world
        }.items()
    )
    
    gazebo_spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', 'dbot_mobile'],
        output='screen',
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

    # # Add nodes
    nodes.append(rsp_node)
    nodes.append(rviz2_node)
    nodes.append(gazebo_launch)
    nodes.append(gazebo_spawn_entity)
    nodes.append(joint_state_broadcaster_spawner)
    nodes.append(dbot_mobile_controller_spawner)
    
    return LaunchDescription(declared_arguments + nodes)