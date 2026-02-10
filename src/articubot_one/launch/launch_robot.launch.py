from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from launch.substitutions import Command
from launch import LaunchDescription

def generate_launch_description():
    package_name="articubot_one"
    package_path=get_package_share_directory(package_name)
    
    xacro_file_path = os.path.join(package_path, 'description', 'robot.urdf.xacro')
    
    robot_description_config=Command(['xacro ', xacro_file_path, ' use_ros2_control:=true'])
    
    rviz_config_path=os.path.join(package_path, 'config', 'view_bot.rviz')
    
    controllers_config_path=os.path.join(package_path,'config', 'my_controllers.yaml')
    
    print(f"DEBUG: Controller path is: {controllers_config_path}")
    
    params={'robot_description': robot_description_config}
    
    robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )
    joint_state_broadcaster_spawner=Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad']
    )
    
    diff_drive_spawner=Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_cont',
            '--param-file', 
            controllers_config_path,
            '--ros-args',
            '-r',
            '/diff_cont/cmd_vel:=/cmd_vel'
        ],
    )
    
    teleop_node=Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='gnome-terminal --',
        parameters=[{
            'speed': 0.5,
            'turn': 0.25
        }],
        output='screen'
    )
 
    
    return LaunchDescription([
        robot_state_publisher,
        rviz_node,
        diff_drive_spawner,
        joint_state_broadcaster_spawner,
        teleop_node
    ])