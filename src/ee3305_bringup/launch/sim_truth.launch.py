from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # ================ 1. PACKAGE SHARE DIRECTORIES ==============
    pkg_ee3305_bringup = FindPackageShare("ee3305_bringup")

    # ================ 2. LAUNCH ARGUMENTS ==============
    # Launch Arg: x
    arg_x = DeclareLaunchArgument(
        "x", default_value="-2.0", description="x position of the robot."
    )
    ld.add_action(arg_x)

    # Launch Arg: y
    arg_y = DeclareLaunchArgument(
        "y", default_value="-0.5", description="y position of the robot."
    )
    ld.add_action(arg_y)

    # Launch Arg: model
    arg_model = DeclareLaunchArgument(
        "model",
        default_value="burger",
        description='Same as TURTLEBOT3_MODEL environment variable. Defaults to "burger"',
    )
    ld.add_action(arg_model)

    # Launch Arg: world
    arg_world = DeclareLaunchArgument(
        "world",
        default_value="turtlebot3_house",
        description="Name of the Gazebo world file to load. Must be in turtlebot3_gazebo",
    )
    ld.add_action(arg_world)

    # Launcg Arg: map
    arg_map = DeclareLaunchArgument(
        "map",
        default_value="ee3305",
        description="Name of the map files (excluding extensions .pgm, .yaml) to load. Must be in ee3305_bringup/maps and built",
    )
    ld.add_action(arg_map)

    # Launch Arg: headless
    arg_headless = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="If set to True, open the Gazebo simulator and its GUI. If set to False, the GUI is gone and the simulation runs in the background.",
    )
    ld.add_action(arg_headless)

    # Launch Arg: libgl
    arg_libgl = DeclareLaunchArgument(
        "libgl",
        default_value="False",
        description="If set to True, LibGL is used (this is primarily for VirtualBox users). If set to False, the faster ogre2 renderer is used (cannot be used in VirtualBox, only for Dual boot).",
    )
    ld.add_action(arg_libgl)

    # ======= 3. LAUNCH FILES ===========
    # start the world
    launch_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ee3305_bringup, "launch", "sim_world.launch.py"]),
        ),
        launch_arguments={
            "use_sim_time": "True",
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "model": LaunchConfiguration("model"),
            "world": LaunchConfiguration("world"),
            "headless": LaunchConfiguration("headless"),
            "libgl": LaunchConfiguration("libgl"),
        }.items(),
    )
    ld.add_action(launch_world)

    # ================ 4. NODES (GROUND TRUTH) ======================
    # loads the map, persists the map topics, and then exits.
    node_map_loader = Node(
        package="ee3305_map_loader",
        executable="map_loader",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [pkg_ee3305_bringup, "params", "map_loader.yaml"]
            ),  # the param file
            {
                "filepath": PathJoinSubstitution(
                    [pkg_ee3305_bringup, "maps", LaunchConfiguration("map")]
                )
            },  # override the filepath param (i.e. path to map)
        ],
    )
    ld.add_action(node_map_loader)

    # for rviz
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_ee3305_bringup, "rviz", "sim_truth.rviz"]),
        ],
        output="screen",
    )
    ld.add_action(node_rviz)

    # publishes the static tf betw. map and odom (bcos no localization)
    node_static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "/map", "/odom"],
    )
    ld.add_action(node_static_tf_publisher)

    return ld
