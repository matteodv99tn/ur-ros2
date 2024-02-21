import os.path
from launch.event_handlers import OnProcessExit
import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.substitutions.find_package import get_package_share_directory
from ur_moveit_config.launch_common import load_yaml


package_name = "magician_ur"
this_package_share = get_package_share_directory(package_name)



def launch_setup(context, *args, **kwargs):

    nodes_to_start = []

    #  ____                                _
    # |  _ \ __ _ _ __ __ _ _ __ ___   ___| |_ ___ _ __ ___
    # | |_) / _` | '__/ _` | '_ ` _ \ / _ \ __/ _ \ '__/ __|
    # |  __/ (_| | | | (_| | | | | | |  __/ ||  __/ |  \__ \
    # |_|   \__,_|_|  \__,_|_| |_| |_|\___|\__\___|_|  |___/
    #
    is_simulation = LaunchConfiguration("is_simulation").perform(context)
    controllers = LaunchConfiguration("controllers").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    use_sim_time = LaunchConfiguration("is_simulation").perform(context) == "true"
    urdf_xacro_path = LaunchConfiguration("urdf_xacro_path").perform(context)
    srdf_xacro_path = LaunchConfiguration("srdf_xacro_path").perform(context)
    ur_type = LaunchConfiguration("ur_type").perform(context)
    ur_ip_address = LaunchConfiguration("ur_ip_address").perform(context)
    load_moveit = LaunchConfiguration("load_moveit").perform(context)

    controllers_full_path = os.path.join(
        get_package_share_directory(package_name), "config", controllers
    )

    #  ____  _             _
    # / ___|| |_ __ _ _ __| |_ _   _ _ __
    # \___ \| __/ _` | '__| __| | | | '_ \
    #  ___) | || (_| | |  | |_| |_| | |_) |
    # |____/ \__\__,_|_|   \__|\__,_| .__/
    #                               |_|
    robot_description_content = xacro.process(
        urdf_xacro_path,
        mappings={
            "name": "ur",
            "ur_type": ur_type,
            "sim_ignition": is_simulation,
            "simulation_controllers": controllers_full_path,
        }
    )
    robot_description = {"robot_description": robot_description_content}

    semantic_description_content = xacro.process(
        srdf_xacro_path, mappings={
            "name": "ur", "ur_type": ur_type
        }
    )
    semantic_description = {"robot_description_semantic": semantic_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[
            robot_description, semantic_description, {
                "use_sim_time": use_sim_time
            }
        ],
        condition=IfCondition(is_simulation),
    )

    nodes_to_start += [
        robot_state_publisher_node,
    ]

    #  ___            _ _   _
    # |_ _|__ _ _ __ (_) |_(_) ___  _ __
    #  | |/ _` | '_ \| | __| |/ _ \| '_ \
    #  | | (_| | | | | | |_| | (_) | | | |
    # |___\__, |_| |_|_|\__|_|\___/|_| |_|
    #     |___/
    ignition_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="log",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "ur",
            "-allow_renaming",
            "true",
        ],
        condition=IfCondition(is_simulation),
    )

    ignition_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments={"ign_args": " -r -v 1 empty.sdf"}.items(),
        condition=IfCondition(is_simulation),
    )

    nodes_to_start += [
        ignition_spawn_robot,
        ignition_simulator,
    ]

    #  _   _           _
    # | \ | | ___   __| | ___  ___
    # |  \| |/ _ \ / _` |/ _ \/ __|
    # | |\  | (_) | (_| |  __/\__ \
    # |_| \_|\___/ \__,_|\___||___/
    #
    ur_drivers_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"
        ], ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": ur_ip_address,
            "tf_prefix": "",
            "description_package": "ur_description",
            "description_file": "ur.urdf.xacro",
            "runtime_config_package": package_name,
            "controllers_file": controllers,
            "launch_rviz": "false",
            "headless_mode": "true",
            # "initial_joint_controller": "joint_trajectory_controller",
        }.items(),
        condition=UnlessCondition(is_simulation),
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "--controller-manager", "/controller_manager"
        ],
        output="log",
        condition=IfCondition(is_simulation),
    )

    initial_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="log",
        condition=IfCondition(is_simulation),
    )

    nodes_to_start += [
        ur_drivers_node,
        initial_controller_spawner,
        joint_state_broadcaster_node,
    ]

    #  __  __                ___ _   ____
    # |  \/  | _____   _____|_ _| |_|___ \
    # | |\/| |/ _ \ \ / / _ \| || __| __) |
    # | |  | | (_) \ V /  __/| || |_ / __/
    # |_|  |_|\___/ \_/ \___|___|\__|_____|
    #
    moveit_config_package = "ur_moveit_config"
    warehouse_sqlite_path = os.path.expanduser("~/.ros/warehouse_ros.sqlite")

    robot_description_kinematics = PathJoinSubstitution([
        FindPackageShare(moveit_config_package), "config", "kinematics.yaml"
    ])

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin":
                "ompl_interface/OMPLPlanner",
            "request_adapters":
                """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error":
                0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    change_controllers = is_simulation
    if change_controllers == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager":
            controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            semantic_description,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {
                "use_sim_time": use_sim_time == "true"
            },
            warehouse_ros_config,
        ],
        condition=IfCondition(load_moveit),
    )

    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            semantic_description,
        ],
        output="screen",
        condition=IfCondition(load_moveit),
    )

    nodes_to_start += [
        move_group_node,
        servo_node,
    ]

    #  ______     ___
    # |  _ \ \   / (_)____
    # | |_) \ \ / /| |_  /
    # |  _ < \ V / | |/ /
    # |_| \_\ \_/  |_/___|
    #
    rviz_parameters = None
    if load_moveit == "true":
        rviz_parameters = [
            robot_description,
            semantic_description,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            warehouse_ros_config,
        ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=rviz_parameters,
        condition=IfCondition(LaunchConfiguration("launch_rviz").perform(context)),
    )
    rqt_controller_manager = Node(
            package="rqt_controller_manager",
            executable="rqt_controller_manager",
            name="rqt_controller_manager",
            output="log",
            condition=IfCondition(LaunchConfiguration("launch_rqt_cm").perform(context)),
        )
    
    nodes_to_start += [
        rviz_node,
        rqt_controller_manager,
    ]

    return nodes_to_start



def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "is_simulation",
            description="Load simulation environment or robot driver?",
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
            "load_moveit",
            description="Load moveit2 components?",
            default_value="false",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            description="RViz configuration",
            default_value=os.path.join(this_package_share, "rviz", "view_robot.rviz")
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers",
            description=".yaml file name inside current package",
            default_value="default_controllers.yaml"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="type of UR robot to be used in the simulation or driver.",
            default_value="ur5",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_xacro_path",
            description="Path to the urdf xacro file to be used for robot description.",
            default_value=os.path.join(
                get_package_share_directory("ur_description"), "urdf", "ur.urdf.xacro"
            ),
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
            "launch_rqt_cm",
            description="Launch rqt_controller_manager?",
            default_value="false",
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
