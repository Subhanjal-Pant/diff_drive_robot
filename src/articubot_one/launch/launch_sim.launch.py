import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit



def generate_launch_description():
    
    package_name="articubot_one"
    use_sim_time=LaunchConfiguration('use_sim_time')
    set_gz_config = SetEnvironmentVariable('GZ_CONFIG_PATH', '/usr/share/gz')
    
    package_path=get_package_share_directory(package_name)
    world_path=os.path.join(package_path,'worlds','final_world.sdf')
    
    slam_param_file=os.path.join(package_path, 'config', 'mapper_params_online_async.yaml')
    nav2_params_file=os.path.join(package_path, 'config', 'nav2_params.yaml')
    twist_mux_param_file=os.path.join(package_path,'config','twist_mux.yaml')
    gazebo_params_file = os.path.join(package_path, 'config', 'gazebo_params.yaml')
    
    
    rsp=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(package_path, 'launch', "rviz_view.launch.py")]),
        launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control':'true'}.items()
    )
    
    gazebo=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'use_sim_time':use_sim_time,
            'extra_gazebo_args': '--ros-args --params '+ gazebo_params_file
            }.items(),
    )
    
    spawn_entity=Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=['-topic', 'robot_description','-name', 'bot', '-x', '0', '-y', '0', '-z', '0.1'],output='screen',
    )
    
    joint_state_broadcaster_spawner=Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager-timeout', '60']
    )
  
  
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='gnome-terminal --',
        parameters=[{
            'speed': 0.5,
            'turn': 0.5,
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_teleop'),
        ],
        output='screen'
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_param_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel_mux')
        ]
    )

    # ros2 run teleop_twist_keyboard teleop_twist_keyboard
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            }],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel_mux'),
            ('/cmd_vel_out', '/diff_cont/cmd_vel')
        ], 
        output='screen'
    )
    
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # '/model/bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo', 
            '/camera/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',

        ],
        parameters=[{'use_sim_time': use_sim_time}],
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

      
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_cont',
            '--controller-manager-timeout', '60',
            '--param-file', os.path.join(package_path, 'config', 'my_controllers.yaml'),
            '--ros-args', 
            '-r', '/diff_cont/cmd_vel:=/cmd_vel'
            ],
    )



    delay_joint_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_diff_drive_spawner_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                diff_drive_spawner],
        )
    )
    # subhanjal-pant@subhanjal-pant-Legion-5-15IRX10:~/Desktop/ROS/diff_drive_robot$ ros2 launch slam_toolbox online_async_launch.py params_file:=.src/articubot_one/config/mapper_params_online_async.yaml use_sim_time:=true
    slam_toolbox_node=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'params_file' : slam_param_file,
            'use_sim_time': use_sim_time
            }.items(),
    )
    
    nav_node=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py'),
        ]),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    
    # ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map_save.yaml -p use_sim_time:=true
    # ros2 run nav2_util lifecycle_bringup map_server
    # ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true
    # ros2 run nav2_util lifecycle_bringup amcl
    # ros2 run twist_mux twist_mux --ros-args --params-file ./src/articubot_one/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel
    return LaunchDescription([
        # set_render_engine,
        set_gz_config,
        rsp,
        gazebo,
        spawn_entity,
        bridge,
        teleop_node,
        
        # republisher_node,
        # republisher_node_to_raw,
        # joint_state_broadcaster_spawner,
        # diff_drive_spawner,
        delay_joint_broadcaster_after_spawn,
        delay_diff_drive_spawner_after_broadcaster,
        # delay_rviz_after_diff_drive,
        twist_stamper,
        twist_mux_node,
        slam_toolbox_node,
        nav_node,
        
    ])
