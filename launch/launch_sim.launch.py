import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition

from launch_ros.actions import Node



def generate_launch_description():

    activate_slam_arg = DeclareLaunchArgument(
        'activate_slam', default_value='false',
        description='Flag to activate SLAM'
    )

    activate_nav_arg = DeclareLaunchArgument(
        'activate_nav', default_value='false',
        description='Flag to activate Nav2'
    )

    activate_loc_arg = DeclareLaunchArgument(
        'activate_loc', default_value='false',
        description='Flag to activate localisation'
    )

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='articubot_one' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                                      'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'arena.world')
                                      }.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    storage_servo_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["storage_servo"],
    )


    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    slam_toolbox_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('articubot_one'), 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('activate_slam'))
    )

    delayed_slam_launch = TimerAction(
        period=5.0, 
        actions=[slam_toolbox_launch_description]
    )

    loc_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('articubot_one'), 'config', 'nav2_params.yaml'),
            'map':os.path.join(get_package_share_directory('articubot_one'), 'maps', 'map_gzbo.yaml'),
            'use_sim_time': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('activate_loc'))
    )

    delayed_loc_launch = TimerAction(
        period=10.0, 
        actions=[loc_launch_description]
    )

    nav_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory('articubot_one'), 'config', 'nav2_params.yaml'),
            'use_sim_time': 'true',
            'map_subscribe_transient_local': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('activate_nav'))
    )

    delayed_nav_launch = TimerAction(
        period=10.0, 
        actions=[nav_launch_description]
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('articubot_one'), 'config', 'main.rviz')]
    )

    depthai_examples_path = get_package_share_directory('depthai_examples')
    oakd_path = get_package_share_directory('oakd_cam')

    default_resources_path = os.path.join(oakd_path, 'resources')

    mxId         = LaunchConfiguration('mxId',      default = 'x')
    usb2Mode     = LaunchConfiguration('usb2Mode',  default = False)
    poeMode      = LaunchConfiguration('poeMode',   default = False)
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    tf_prefix    = LaunchConfiguration('tf_prefix',     default = 'oak')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    imuMode      = LaunchConfiguration('imuMode', default = '1')
    nnName                  = LaunchConfiguration('nnName', default = 'x')
    resourceBaseFolder      = LaunchConfiguration('resourceBaseFolder', default = default_resources_path)
    angularVelCovariance  = LaunchConfiguration('angularVelCovariance', default = 0.02)
    linearAccelCovariance = LaunchConfiguration('linearAccelCovariance', default = 0.02)
    enableRosBaseTimeUpdate       = LaunchConfiguration('enableRosBaseTimeUpdate', default = False)

    oakd_imu_node = Node(
            package='oakd_cam', executable='oakd_imu_node',
            output='screen',
            parameters=[{'mxId':                    mxId},
                        {'usb2Mode':                usb2Mode},
                        {'poeMode':                 poeMode},
                        {'resourceBaseFolder':      resourceBaseFolder},
                        {'tf_prefix':               tf_prefix},
                        {'imuMode':                 imuMode},
                        {'angularVelCovariance':    angularVelCovariance},
                        {'linearAccelCovariance':   linearAccelCovariance},
                        {'nnName':                  nnName},
                        {'enableRosBaseTimeUpdate': enableRosBaseTimeUpdate}
                        ])

    config_dir_oak = os.path.join(get_package_share_directory('oakd_cam'), 'config')

    oakd_imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[os.path.join(config_dir_oak, 'imu_filter.yaml')]
    )



    # Launch them all!
    return LaunchDescription([
        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        storage_servo_spawner,
        activate_loc_arg,
        activate_nav_arg,
        activate_slam_arg,
        delayed_slam_launch,
        delayed_loc_launch,
        delayed_nav_launch,
        rviz_node,
        # oakd_imu_node,
        # oakd_imu_filter_node
    ])
