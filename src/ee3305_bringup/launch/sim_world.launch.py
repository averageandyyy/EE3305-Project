from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    EqualsSubstitution,
    NotEqualsSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # ================ 1. PACKAGE SHARE DIRECTORIES ==============
    pkg_turtlebot3_gazebo = FindPackageShare("turtlebot3_gazebo")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")
    pkg_ee3305_bringup = FindPackageShare("ee3305_bringup")

    # ================ 2. LAUNCH ARGUMENTS ==============
    # Launch Arg: use_sim_time
    arg_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    ld.add_action(arg_use_sim_time)

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

    # ================ 3. ENV VARIABLES  ==============
    # Changes the renderer for Gz if using VBox.
    env_lib_gl_true = SetEnvironmentVariable(
        "LIBGL_ALWAYS_SOFTWARE",
        "1",
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration("libgl"), "True")),
    )
    ld.add_action(env_lib_gl_true)  # for single/dual boot

    env_lib_gl_false = SetEnvironmentVariable(
        "LIBGL_ALWAYS_SOFTWARE",
        "0",
        condition=IfCondition(
            NotEqualsSubstitution(LaunchConfiguration("libgl"), "True")
        ),
    )
    ld.add_action(env_lib_gl_false)  # for single/dual boot

    # For Gz to find the world models from turtlebot3_gazebo
    env_pkg_turtlebot3_gazebo = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg_turtlebot3_gazebo, "models"])
    )
    ld.add_action(env_pkg_turtlebot3_gazebo)

    # Sets the environment variable for the turtlebot's model
    env_turtlebot3_model = SetEnvironmentVariable(
        "TURTLEBOT3_MODEL", LaunchConfiguration("model")
    )
    ld.add_action(env_turtlebot3_model)

    # ======= 4. LAUNCH FILES ===========
    # Launch Gz depending on headless or with GUI
    launch_gz_sim_launch_file = PathJoinSubstitution(
        [pkg_ros_gz_sim, "launch", "gz_sim.launch.py"]
    )
    launch_gz_sim_world_file = PathJoinSubstitution(
        [pkg_ee3305_bringup, "worlds", [LaunchConfiguration("world"), ".world"]]
    )

    launch_gz_sim_extra_gz_args = " -r -v1" 
    # launch_gz_sim_extra_gz_args = ' -r -v1 --render-engine ogre' # DO NOT USE FOR VBOX OR BOOT: this may cause the last reading for VBox users to become 0.05. -r for autorun, -v for verbose, v1 for level 1 verbose.

    launch_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_gz_sim_launch_file]),
        condition=IfCondition(
            NotEqualsSubstitution(LaunchConfiguration("headless"), "True")
        ),
        launch_arguments={
            "gz_args": [
                launch_gz_sim_world_file,
                TextSubstitution(text=launch_gz_sim_extra_gz_args),
            ],
        }.items(),
    )
    ld.add_action(launch_gz_sim)

    launch_gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_gz_sim_launch_file]),
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("headless"), "True")
        ),
        launch_arguments={
            "gz_args": [
                launch_gz_sim_world_file,
                TextSubstitution(text=launch_gz_sim_extra_gz_args + " -s"),
            ],
        }.items(),
    )
    ld.add_action(launch_gz_sim_headless)

    # Launch the turtlebot's Robot State Publisher
    launch_turtlebot3_gazebo_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_turtlebot3_gazebo, "launch", "robot_state_publisher.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )
    ld.add_action(launch_turtlebot3_gazebo_robot_state_publisher)

    # Launch the turtlebot's Gazebo Spawner
    launch_turtlebot3_gazebo_spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_turtlebot3_gazebo, "launch", "spawn_turtlebot3.launch.py"]
            )
        ),
        launch_arguments={
            "x_pose": LaunchConfiguration("x"),
            "y_pose": LaunchConfiguration("y"),
        }.items(),
    )
    ld.add_action(launch_turtlebot3_gazebo_spawn_turtlebot3)

    # # Above are adapted from the contents of the launch files in turtlebot3_gazebo to launch the worlds.
    # # The worlds from the turtlebot3_gazebo package relies on strict physics solver.
    # # Need to use custom worlds modified from these files for good realtime factor (>90%) on Gz Harmonic, particularly for slam teleoperation.
    # launch_turtlebot3_gazebo_world = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             pkg_turtlebot3_gazebo, 'launch', [LaunchConfiguration('world'), '.launch.py']
    #         ])
    #     ),
    #     launch_arguments={
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'x_pose': LaunchConfiguration('x'),
    #         'y_pose': LaunchConfiguration('y'),
    #     }.items()
    # )
    # ld.add_action(launch_turtlebot3_gazebo_world)

    return ld
