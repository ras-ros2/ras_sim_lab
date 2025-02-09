import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.logging import get_logger
from ament_index_python.packages import get_package_share_directory
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ras_common.config.loaders.lab_setup import LabSetup as LabLoader
from ras_resource_lib.managers.asset_manager import AssetManager,AssetType
from ras_resource_lib.types.manipulator.component import ManipulatorComponent
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():
    LabLoader.init()
    AssetManager.init()
    robot_component : ManipulatorComponent = AssetManager.get_asset_component(LabLoader.robot_name,AssetType.MANIPULATOR)
    moveit_config = robot_component.moveit_config
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        arguments=["--log-level", "debug"],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("ras_app_main") + "/rviz/moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )


    # Node to spawn joint trajectory controller
    spawn_controllers_manipulator = Node(
        package="controller_manager", 
        executable="spawner",
        name="spawner_mani",
        arguments=['joint_trajectory_controller'],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Node to spawn joint state broadcaster
    spawn_controllers_state = Node(
        package="controller_manager", 
        executable="spawner",
        arguments=['joint_state_broadcaster'],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    sim_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ras_sim'),
            'launch', 'main.launch.py')]))

    return LaunchDescription(
        [
            rviz_node,
            run_move_group_node,
            sim_launch,
            spawn_controllers_manipulator,
            spawn_controllers_state
        ]
    )