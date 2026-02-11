import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro
from launch_ros.actions import SetParameter


def generate_launch_description():
    pkg_path=get_package_share_directory('articubot_one')

    use_sim_time=LaunchConfiguration('use_sim_time')
    use_ros2_control=LaunchConfiguration('use_ros2_control')
    SetParameter(name='use_sim_time', value=use_sim_time)

    xacro_file=os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    # robot_description_config= xacro.process_file(xacro_file)
    robot_description_config=Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    # rviz_config_path=os.path.join(pkg_path, 'config', 'view_bot.rviz')
    rviz_config_path=os.path.join(pkg_path, 'config', 'saved_configurations.rviz')

   
    
    params={'robot_description': robot_description_config, 'use_sim_time':use_sim_time}
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
    )

    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )
 

 
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description="Use sim time if true"
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description="Use ros2_control if true"
        ),
        
        robot_state_publisher,
        rviz_node,
   
    
    ])