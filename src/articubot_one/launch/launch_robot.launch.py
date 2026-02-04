import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration,Command
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    
    package_name="articubot_one"
    
    rsp=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', "rviz_view.launch.py")]),
        launch_arguments={'use_sim_time':'false', 'use_ros2_control':'true'}.items()
    )
    
    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')
     
    robot_description=Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    

    controller_manager=Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params_file],
    )
    
    delayed_controller_manager=TimerAction(period=3.0, actions=[controller_manager])
    
    
    
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_cont',
            '--param-file', os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml'),
            '--ros-args', 
            '-r', '/diff_cont/cmd_vel:=/cmd_vel'
            ],
    )
    delayed_diff_drive_spawner=RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )
    
    
    joint_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=[
            'joint_broad',
        ],
    )
    delayed_joint_broadcaster_spawner=RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broadcaster_spawner],
            )
    )
    
            
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            }],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/diff_cont/cmd_vel')
        ], 
        output='screen'
    )
    
    
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='gnome-terminal --',
        parameters=[{
            'speed': 1.0,
            'turn': 0.75,
        }],
        output='screen'
    )
    
    
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broadcaster_spawner,
        teleop_node,
        twist_stamper,
    ])
