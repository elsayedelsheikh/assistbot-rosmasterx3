# $LICENSE$
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="assistbot_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="assistbot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    world_path = ""
    # "world": get_package_share_directory('assistbot_gazebo') + "/worlds/guardian.world",
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "use_manipulator:=false",
            " ",
            "sim_gazebo_classic:=true",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    # )

    # Gazebo nodes
    gazebo_resources_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                    EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                        default_value=''),
                                                    '/usr/share/gazebo-11/models/:',
                                                    str(Path(get_package_share_directory("assistbot_description")).
                                                        parent.resolve())])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            "world": world_path,
        }.items(),
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_guardian",
        arguments=["-entity", "greenguardian", "-topic", "robot_description", "-x", "0.0", "-y", "0.0", "-z", "0.0"],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            gazebo_resources_path,
            gazebo,
            gazebo_spawn_robot,
            robot_state_pub_node,
        ]
    )