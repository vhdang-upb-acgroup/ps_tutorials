import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    path_gz_ros_humble_pkg = get_package_share_directory('gz_ros_humble_pkg')

    gazebo_models_path, ignore_last_dir = os.path.split(path_gz_ros_humble_pkg)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    # rviz_launch_arg = DeclareLaunchArgument(
    #     'rviz', default_value='true',
    #     description='Open RViz.'
    # )

    world_arg = DeclareLaunchArgument(
        'world', default_value='world.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='ground_robot.urdf',
        description='Name of the URDF description to load'
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        path_gz_ros_humble_pkg,  # Replace with your package name
        "models",
        LaunchConfiguration('model')  # Replace with your URDF or Xacro file
    ])

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(path_gz_ros_humble_pkg, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'worlds': LaunchConfiguration('world'),
        }.items()
    )

    # # Launch rviz
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', os.path.join(path_gz_ros_humble_pkg, 'rviz', 'rviz.rviz')],
    #     condition=IfCondition(LaunchConfiguration('rviz')),
    #     parameters=[
    #         {'use_sim_time': True},
    #     ]
    # )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my__ground_robot",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0"  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': True},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    launchDescriptionObject = LaunchDescription()

    # launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(world_launch)
    # launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(joint_state_publisher_gui_node)

    return launchDescriptionObject