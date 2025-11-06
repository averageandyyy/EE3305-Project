from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EqualsSubstitution,
    NotEqualsSubstitution,
)
from launch_ros.substitutions import FindPackageShare
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

    # Launch Arg: map
    arg_map = DeclareLaunchArgument(
        "map",
        default_value="ee3305",
        description="Name of the map files (excluding extensions .pgm, .yaml) to load. Must be in ee3305_bringup/maps and built",
    )
    ld.add_action(arg_map)

    # Launch Arg: cpp
    arg_cpp = DeclareLaunchArgument(
        "cpp",
        default_value="False",
        description="If True, runs the nodes from the ee3305_cpp. If False, runs the nodes from the ee3305_py package",
    )
    ld.add_action(arg_cpp)

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
    # start the gz world, ground truth positioning, and map publishers.
    launch_sim_truth = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ee3305_bringup, "launch", "sim_truth.launch.py"]),
        ),
        launch_arguments={
            "use_sim_time": "True",
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "model": LaunchConfiguration("model"),
            "world": LaunchConfiguration("world"),
            "map": LaunchConfiguration("map"),
            "headless": LaunchConfiguration("headless"),
            "libgl": LaunchConfiguration("libgl"),
        }.items(),
    )
    ld.add_action(launch_sim_truth)

    # ================ 4. NODES (GROUND TRUTH) ======================
    if_cond_cpp = IfCondition(EqualsSubstitution(LaunchConfiguration("cpp"), "True"))
    if_cond_not_cpp = IfCondition(
        NotEqualsSubstitution(LaunchConfiguration("cpp"), "True")
    )
    nodes_param_file = PathJoinSubstitution([pkg_ee3305_bringup, "params", "run.yaml"])

    # Run Cpp Behavior Node
    node_behavior = Node(
        condition=if_cond_cpp,
        package="ee3305_cpp",
        executable="behavior",
        output="screen",
        emulate_tty="True",
        parameters=[nodes_param_file],
    )
    ld.add_action(node_behavior)

    # Run Cpp Controller Node
    node_behavior = Node(
        condition=if_cond_cpp,
        package="ee3305_cpp",
        executable="controller",
        output="screen",
        emulate_tty="True",
        parameters=[nodes_param_file],
    )
    ld.add_action(node_behavior)

    # Run Cpp Planner Node
    node_behavior = Node(
        condition=if_cond_cpp,
        package="ee3305_cpp",
        executable="planner",
        output="screen",
        emulate_tty="True",
        parameters=[nodes_param_file],
    )
    ld.add_action(node_behavior)

    # Run Py Behavior Node
    node_behavior = Node(
        condition=if_cond_not_cpp,
        package="ee3305_py",
        executable="behavior",
        output="screen",
        emulate_tty="True",
        parameters=[nodes_param_file],
    )
    ld.add_action(node_behavior)

    # Run Py Controller Node
    node_behavior = Node(
        condition=if_cond_not_cpp,
        package="ee3305_py",
        executable="controller",
        output="screen",
        emulate_tty="True",
        parameters=[nodes_param_file],
    )
    ld.add_action(node_behavior)

    # Run Py Planner Node
    node_behavior = Node(
        condition=if_cond_not_cpp,
        package="ee3305_py",
        executable="planner",
        output="screen",
        emulate_tty="True",
        parameters=[nodes_param_file],
    )
    ld.add_action(node_behavior)

    return ld
