from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # ================ 1. PACKAGE SHARE DIRECTORIES ==============
    pkg_ee3305_bringup = FindPackageShare("ee3305_bringup")
    pkg_turtlebot3_cartographer = FindPackageShare("turtlebot3_cartographer")

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

    # launch the cartographer (slam)
    launch_turtlebot3_cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_turtlebot3_cartographer, "launch", "cartographer.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "True",
        }.items(),
    )
    ld.add_action(launch_turtlebot3_cartographer)

    return ld


#### Attempt at Teleop #####

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     ld = LaunchDescription()

#     # ================ 1. LAUNCH ARGUMENTS ==============
#     # Launch Arg: model
#     arg_model = DeclareLaunchArgument(
#         'model',
#         default_value='burger',
#         description='Same as TURTLEBOT3_MODEL environment variable. Defaults to "burger"'
#     )
#     ld.add_action(arg_model)

#     # ================ 2. ENV VARIABLES ==================
#     # Sets the environment variable for the turtlebot's model
#     env_turtlebot3_model = SetEnvironmentVariable(
#         'TURTLEBOT3_MODEL',
#         LaunchConfiguration('model')
#     )
#     ld.add_action(env_turtlebot3_model)

#     # ======= 3. RUN NODE ===========
#     node_teleop = Node(
#         package='turtlebot3_teleop',
#         executable='teleop_keyboard',
#         output='screen',
#         emulate_tty=True,
#         prefix = ['xterm -e'],
#     )
#     ld.add_action(node_teleop)

#     return ld
