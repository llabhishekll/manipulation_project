from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # moveit config
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    # return
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            Node(
                package="moveit2_scripts",
                executable="pick_and_place_sim",
                name="pick_and_place_sim_node",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
                    {"publish_robot_description_semantic": True},
                    {"use_sim_time": True},
                ],
                condition=IfCondition(use_sim_time),
            )
        ]
    )
