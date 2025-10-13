# 版权声明 (c) 2025 Minzi
# 基于 BSD-3-Clause 许可证开源

# 导入 os 库，用于处理文件路径等操作系统相关的任务。
import os

# 从 launch 包导入核心类和函数，用于定义启动描述。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
# 导入 "substitutions"，这些是启动时才会执行的“延迟替换”命令。
# 它们允许我们动态地构建命令、查找文件和获取配置值。
from launch.substitutions import (
    Command,              # 用于执行一个本地命令（如此处的 xacro）
    FindExecutable,       # 用于查找一个可执行文件的完整路径（如 xacro）
    LaunchConfiguration,  # 用于获取在启动时传入的参数值
    PathJoinSubstitution, # 用于安全地拼接文件路径
)
# 从 launch_ros 包导入 ROS 相关的启动操作。
from launch_ros.actions import Node
# 从 launch_ros 包导入 ROS 相关的 "substitutions"。
from launch_ros.substitutions import FindPackageShare
# 从 launch_ros 包导入用于处理 ROS 参数的工具。
from launch_ros.parameter_descriptions import ParameterValue


# === 第二部分：定义核心启动逻辑函数 ===
# 这个函数包含了启动文件的主要逻辑，例如加载文件、配置参数和定义要运行的节点。
# 它使用 OpaqueFunction 来确保在执行此函数之前，所有的启动参数（LaunchConfiguration）都已经被解析。
# context 参数包含了所有已解析的启动参数值。
def launch_setup(context, *args, **kwargs):
    # --- 接收启动时传入的参数 ---
    # 使用 LaunchConfiguration 获取在 generate_launch_description 中声明的参数的“值”。
    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")

    # --- 准备机器人模型 (URDF) ---
    # URDF (Unified Robot Description Format) 是一个 XML 文件，描述了机器人的物理结构，如连杆、关节等。
    # 这里我们使用 Command 来执行 'xacro' 命令，动态地从 .xacro 文件生成 URDF 的完整内容。
    robot_description_content = Command(
        [
            # 查找 xacro 可执行文件的完整路径
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            # 拼接 urdf.xacro 文件的完整路径
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            # --- 以下是传递给 xacro 文件的参数 ---
            # 仿真中不需要真实的机器人 IP，但 xacro 文件需要这个参数，所以提供一个虚拟值
            "robot_ip:=0.0.0.0",
            " ",
            # 指定关节限制参数文件的路径
            "joint_limit_params:=",
            PathJoinSubstitution([FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]),
            " ",
            # 指定运动学参数文件的路径
            "kinematics_params:=",
            PathJoinSubstitution([FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]),
            " ",
            # 指定物理特性参数文件的路径
            "physical_params:=",
            PathJoinSubstitution([FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]),
            " ",
            # 指定视觉效果参数文件的路径
            "visual_params:=",
            PathJoinSubstitution([FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]),
            " ",
            # 是否启用安全限制，这里设置为 false
            "safety_limits:=false",
            " ",
            # 传入机器人型号 (ur_type)
            "ur_type:=",
            ur_type,
            " ",
            # 为机器人模型命名
            "name:=ur",
            " ",
            # 关节和连杆的前缀，用于多机器人场景，默认为空
            "prefix:=",
            LaunchConfiguration("prefix"),
        ]
    )
    # 将上面命令生成的内容封装成一个 ROS 参数。
    # "robot_description" 是 ROS 系统中标准的机器人模型参数名。
    # ParameterValue 确保命令的输出结果（一个长字符串）被正确处理。
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # --- 准备 MoveIt 配置 (SRDF) ---
    # SRDF (Semantic Robot Description Format) 文件是对 URDF 的补充，
    # 它定义了机器人的语义信息，如规划组（手臂、手爪）、末端执行器、默认姿态、禁用的碰撞对等。
    robot_description_semantic_content = Command(
        [
            # 同样使用 xacro 来解析 srdf.xacro 文件
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            # 拼接 srdf.xacro 文件的完整路径
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file]),
            " ",
            # 向 SRDF 文件传递参数
            "name:=ur",
            " ",
            "prefix:=",
            LaunchConfiguration("prefix"),
        ]
    )
    # 将 SRDF 内容封装成名为 "robot_description_semantic" 的 ROS 参数。
    # MoveIt 会自动寻找这个参数来获取语义信息。
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # --- 准备运动学求解器配置 ---
    # 这个文件 (kinematics.yaml) 定义了每个规划组使用哪个逆运动学（IK）求解器插件。
    kinematics_yaml = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # --- 准备规划场景监视器参数 (Planning Scene Monitor) ---
    # 这些参数通常用于配置 MoveIt 的核心组件之一 PlanningSceneMonitor，
    # 它负责维护和发布当前的规划场景（包含机器人自身状态和环境中的障碍物）。
    # 注意：在这个特定的启动文件中，这个参数字典被定义了，但没有传递给下面的 tutorial_node。
    # 它通常会被传递给 move_group 节点。
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "use_sim_time": use_sim_time,
    }

    # --- 定义并配置你的 C++ 教程节点 ---
    # Node action 代表我们要启动一个节点。
    tutorial_node = Node(
        package="ur_planning_scene_tutorial",    # 节点所在功能包的名称
        executable="planning_scene_tutorial",    # C++ 节点的可执行文件名（在 CMakeLists.txt 中定义）
        output="screen",                         # 将节点的日志（如 RCLCPP_INFO）直接输出到终端，方便调试
        # 'parameters' 列表是关键部分，它将前面准备好的所有配置注入到 C++ 节点中。
        parameters=[
            robot_description,                   # 注入 URDF 模型
            robot_description_semantic,          # 注入 SRDF 模型
            kinematics_yaml,                     # 注入运动学求解器配置
            {"use_sim_time": use_sim_time},      # 告知节点是否使用仿真时间
        ],
    )

    # launch_setup 函数必须返回一个包含所有要执行的“动作”（Actions）的列表。
    # 在这里，我们只启动一个节点。
    return [tutorial_node]

# === 第一部分：定义主函数 ===
# `generate_launch_description` 是每个 ROS 2 Python Launch 文件的入口点。
# 当你用 `ros2 launch` 命令运行此文件时，ROS 2 会自动调用这个函数。
def generate_launch_description():
    # 创建一个空列表，用于存放所有我们要声明的启动参数。
    declared_arguments = []

    # --- 声明启动时可以接收的命令行参数 ---
    # 使用 DeclareLaunchArgument 可以让你的启动文件更加灵活和可配置。
    # 你可以在 `ros2 launch` 命令后面通过 `arg_name:=value` 的方式来覆盖默认值。
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",  # 参数名
            default_value="ur5e",  # 默认值
            description="要使用的 UR 机器人型号。",  # 参数的描述信息
            choices=["ur3", "ur5", "ur10", "ur3e", "ur5e", "ur10e", "ur16e", "ur20", "ur30"], # 可选值的列表
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="是否使用仿真时间（例如，从 /clock 话题获取时间）。",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="包含机器人 URDF 文件的功能包。",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO 文件的名称。",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="包含机器人 MoveIt 配置文件的功能包。",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="SRDF/XACRO 文件的名称。",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="为关节和连杆添加前缀（用于多机器人系统）。",
        )
    )

    # --- 返回最终的启动描述 ---
    # LaunchDescription 是一个容器，包含了所有要在启动时执行的动作。
    # 这里我们将声明的参数列表和 OpaqueFunction（它会调用我们的 launch_setup 函数）组合在一起。
    # 这种模式确保了所有参数都先被声明和解析，然后才执行依赖这些参数值的核心逻辑。
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])