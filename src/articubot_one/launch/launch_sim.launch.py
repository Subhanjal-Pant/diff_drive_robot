import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    package_name="articubot_one"
    # Setting the render engine to ogre1 because ogre2 is not supported by my computer hardware
    # Could be changed to orge2 or ogre1 depending upon the hardware
    # set_render_engine=SetEnvironmentVariable('GZ_RENDERING_ENGINE_GUESS', 'ogre')
    set_gz_config = SetEnvironmentVariable('GZ_CONFIG_PATH', '/usr/share/gz')
    world_path=os.path.join(get_package_share_directory(package_name),'worlds','lidar_world.sdf')
    rsp=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', "rviz_view.launch.py")]),
        launch_arguments={'use_sim_time':'true'}.items()
    )
    
    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            # 'gz_args': f'-r {world_path} --render-engine ogre',
            # 'on_exit_shutdown':'true',
            'gz_args': f'-r {world_path}',
            'use_sim_time':'true',
            }.items(),
    )
    
    spawn_entity=Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=['-topic', 'robot_description','-name', 'bot', '-x', '0', '-y', '0', '-z', '0.1'],output='screen'
    )
    
    # bridge_node=Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    #         '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
    #         '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
    #     ],
    #     output='screen'
    # )
    # teleop_node=Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop',
    #     prefix='gnome-terminal --',
    #     output='screen'
    # )
    



    # ros2 run teleop_twist_keyboard teleop_twist_keyboard
    
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='gnome-terminal --',
        output='screen'
    )
    
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # GZ -> ROS: These topics flow FROM Gazebo TO ROS
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # ROS -> GZ: Movement commands flow FROM ROS TO Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/camera/image_raw@/camera/image_raw[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo', 

        ],
        remappings=[
            ('/model/bot/tf', '/tf'),
        ],
        output='screen'
    )
    
    republisher_node=Node(
        package='image_transport',
        executable='republish',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in','/camera/image_raw'),
            ('out/compressed','/camera/image_compressed'),
            ],
        output='screen' 
    )
    republisher_node_to_raw=Node(
        package='image_transport',
        executable='republish',
        # arguments=['compressed', 'raw'],
        parameters=[{
        'in_transport': 'compressed',
        'out_transport': 'raw',
        'qos_overrides./camera/image_compressed.publisher.reliability': 'best_effort',
        }],

        remappings=[
            ('in/compressed','/camera/image_compressed'),
            ('out','/camera/image_raw_from_compressed'),
            ],
        output='screen' 
    )


    
    return LaunchDescription([
        # set_render_engine,
        set_gz_config,
        rsp,
        gazebo,
        spawn_entity,
        bridge,
        teleop_node,
        republisher_node,
        republisher_node_to_raw,
    ])
