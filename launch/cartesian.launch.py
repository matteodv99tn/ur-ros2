import os.path
from ament_index_python.packages import get_package_share_path
from launch.event_handlers import OnExecutionComplete, OnProcessExit
import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.substitutions.find_package import get_package_share_directory
from ur_moveit_config.launch_common import load_yaml


package_name = "magician_ur"
this_package_share = get_package_share_path(package_name)



def launch_setup(context, *args, **kwargs):

    nodes_to_start = []

    is_simulation = LaunchConfiguration("is_simulation")
    ur_type = LaunchConfiguration("ur_type")
    srdf_xacro_path = LaunchConfiguration("srdf_xacro_path")
    ur_ip_address = LaunchConfiguration("ur_ip_address")
    controllers = LaunchConfiguration("controllers")
    load_moveit = LaunchConfiguration("load_moveit")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    launch_rqt_cm = LaunchConfiguration("launch_rqt_cm")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    description_package = LaunchConfiguration("description_package").perform(context)
    xacro_file = LaunchConfiguration("xacro_file").perform(context)

    ur_launch_path = os.path.join(this_package_share, "launch", "ur.launch.py")

    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_launch_path),
        launch_arguments={
            "is_simulation": is_simulation,
            "ur_type": ur_type,
            "description_package": description_package,
            "xacro_file": xacro_file,
            "srdf_xacro_path": srdf_xacro_path,
            "ur_ip_address": ur_ip_address,
            "controllers": controllers,
            "load_moveit": load_moveit,
            "launch_rviz": launch_rviz,
            "rviz_config": rviz_config,
            "launch_rqt_cm": launch_rqt_cm,
            "initial_joint_controller": initial_joint_controller,
        }.items(),
    )
    motion_handler_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motion_control_handle", "-c", "/controller_manager"],
        )
    nodes_to_start += [ur_launch, motion_handler_spawner]

    return nodes_to_start



def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "is_simulation",
            description="Load simulation environment or robot driver?",
            default_value="false",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            description="type of UR robot to be used in the simulation or driver.",
            #default_value="cartesian_compliance_controller",
            default_value="joint_position_controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rqt_cm",
            description="Launch rqt_controller_manager?",
            default_value="true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            description="Launch RViz?",
            default_value="true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            description="RViz configuration",
            default_value=os.path.join(this_package_share, "rviz", "cartesian.rviz")
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers",
            description=".yaml file name inside current package",
            default_value="ur_controllers.yaml"
            # default_value="default_controllers.yaml"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="type of UR robot to be used in the simulation or driver.",
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "load_moveit",
            description="Load moveit2 components?",
            default_value="false",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            description="Package where the robot description (as xacro file) is located.",
            default_value=package_name,
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "xacro_file",
            description="Name of the xacro file to be used for robot description.",
            default_value="ur_table.urdf.xacro",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_ip_address",
            description="IP address of the UR robot to be used in the driver.",
            default_value="192.168.0.100",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "srdf_xacro_path",
            description=
            "Path to the srdf xacro file to be used for semantic robot description.",
            default_value=os.path.join(
                get_package_share_directory("ur_moveit_config"),
                "srdf",
                "ur.srdf.xacro"
            ),
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
