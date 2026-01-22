import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time=LaunchConfiguration('use_sim_time')
    pkg_path=get_package_share_directory('articubot_one')
    xacro_file=os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config= xacro.process_file(xacro_file)
    rviz_config_path=os.path.join(pkg_path, 'config', 'view_bot.rviz')
    
    params={'robot_description': robot_description_config.toxml(), 'use_sim_time':use_sim_time}
    
    robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )
    robot_state_publisher_gui=Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time':use_sim_time}],
    )
    joint_state_publisher_node=Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{'use_sim_time':use_sim_time}],
        
    )
    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[params],
    )
    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
               '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
               '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
               ],
    output='screen'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description="Use sim time if true"
        ),
        
        robot_state_publisher,
        joint_state_publisher_node,
        # robot_state_publisher_gui,
        rviz_node,
        bridge,
    
    ])