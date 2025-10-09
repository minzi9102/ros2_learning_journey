# 文件内容: my_ur_launcher/launch/test_moveit.launch.py (最终修正版)

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 正确的做法：明确指定从哪个包加载哪个文件
    # 从 ur_description 包加载 URDF
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
    )
    # 从 ur_moveit_config 包加载 SRDF
    robot_description_semantic_path = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "ur.srdf.xacro"]
    )
    # 从 ur_moveit_config 包加载 kinematics.yaml
    kinematics_path = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )

    # 使用 MoveItConfigsBuilder，但这次传入的是已经解析好的、正确的完整文件路径
    moveit_configs = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description(file_path=robot_description_path, mappings={"ur_type": "ur3"})
        .robot_description_semantic(file_path=robot_description_semantic_path)
        .robot_description_kinematics(file_path=kinematics_path)
        .trajectory_execution(file_path="config/controllers.yaml") # 这个文件在 ur_moveit_config 里，所以可以用相对路径
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    # 创建 move_group 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_configs.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group_node])