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

    activate_cam_arg = DeclareLaunchArgument(
        'activate_cam', default_value='false',
        description='Flag to activate camera'
    )

    activate_sm_arg = DeclareLaunchArgument(
        'activate_sm', default_value='false',
        description='Flag to activate sm'
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

    yolov6_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('oakd_cam'), 'launch', 'yolov6_publisher.launch.py')]),
        condition=IfCondition(LaunchConfiguration('activate_cam'))
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

    sm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('bb_state_machine'), 'launch', 'bb_state_machine.launch.py')
        ]),
        condition=IfCondition(LaunchConfiguration('activate_sm'))
    )

    delayed_sm_launch = TimerAction(
        period=15.0, 
        actions=[sm_launch]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('articubot_one'), 'config', 'main.rviz')]
    )

    return LaunchDescription([
        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        storage_servo_spawner,
        yolov6_launch,
        activate_loc_arg,
        activate_nav_arg,
        activate_slam_arg,
        activate_cam_arg,
        activate_sm_arg,
        delayed_slam_launch,
        delayed_loc_launch,
        delayed_nav_launch,
        delayed_sm_launch,
        rviz_node
    ])
