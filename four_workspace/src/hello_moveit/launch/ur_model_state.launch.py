# 导入 ROS 2 Launch 系统所需的核心模块
from launch import LaunchDescription                     # 用于定义整个启动描述（包含所有要启动的节点和参数）
from launch.actions import DeclareLaunchArgument, OpaqueFunction  # DeclareLaunchArgument：声明启动参数；OpaqueFunction：用于延迟执行函数（在参数解析后）
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable  # 各种“替换”工具，用于动态构造路径、命令等
from launch_ros.substitutions import FindPackageShare    # 用于查找 ROS 2 包的安装路径
from launch_ros.actions import Node                      # 用于定义要启动的 ROS 2 节点
from launch_ros.parameter_descriptions import ParameterValue  # 用于安全地设置参数值类型（避免类型错误）

# 定义一个函数，用于在所有启动参数解析完成后执行（因为要用到参数值）
def launch_setup(context, *args, **kwargs):
    # 获取启动时传入的参数值（这些值在 generate_launch_description 中声明）
    ur_type = LaunchConfiguration("ur_type")                    # 机器人型号，如 ur3、ur5 等
    description_package = LaunchConfiguration("description_package")  # 机器人描述包名（通常为 ur_description）
    description_file = LaunchConfiguration("description_file")        # URDF/XACRO 文件名
    moveit_config_package = LaunchConfiguration("moveit_config_package")  # MoveIt 配置包名（通常为 ur_moveit_config）
    moveit_config_file = LaunchConfiguration("moveit_config_file")        # SRDF/XACRO 文件名
    prefix = LaunchConfiguration("prefix")                      # 关节名称前缀（用于多机器人场景，一般为空）

    # === 第一部分：通过 xacro 命令加载 URDF（机器人本体模型） ===
    
    # 构造关节限制参数文件的完整路径：{description_package}/config/{ur_type}/joint_limits.yaml
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    # 构造默认运动学参数文件路径（如 DH 参数）
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    # 构造物理参数文件路径（如质量、惯量）
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    # 构造视觉参数文件路径（如颜色、透明度）
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    # 使用 xacro 命令动态生成 URDF 内容（xacro 是一种支持变量和宏的 URDF 扩展）
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # 找到系统中的 xacro 可执行文件路径
            " ",                                                  # 空格分隔命令参数
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),  # xacro 源文件路径
            " ",
            "robot_ip:=0.0.0.0",  # 传入 dummy IP（因为本例不连接真实机器人，仅做模型加载）
            " ",
            "joint_limit_params:=", joint_limit_params,   # 传入关节限制参数文件路径
            " ",
            "kinematics_params:=", kinematics_params,     # 传入运动学参数
            " ",
            "physical_params:=", physical_params,         # 传入物理参数
            " ",
            "visual_params:=", visual_params,             # 传入视觉参数
            " ",
            "safety_limits:=", "true",                   # 关闭安全限制（模型测试时可关闭）
            # "safety_limits:=", "false",                   # 关闭安全限制（模型测试时可关闭）
            " ",
            "ur_type:=", ur_type,                         # 指定机器人型号（如 ur3）
            " ",
            "name:=", "ur",                               # 机器人名称（固定为 "ur"）
            " ",
            "prefix:=", prefix,                           # 关节前缀（如 "" 或 "robot1_"）
        ]
    )
    # 将生成的 URDF 字符串封装为 ROS 参数（类型强制为 str，避免类型推断错误）
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # === 第二部分：通过 xacro 加载 SRDF（语义机器人描述，包含规划组、虚拟关节等） ===
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # 调用 xacro
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file]),  # SRDF.xacro 文件路径
            " ",
            "name:=ur",        # 机器人名称（需与 URDF 一致）
            " ",
            "prefix:=", prefix,  # 同样传入前缀，确保 SRDF 中的关节名与 URDF 匹配
            " ",
        ]
    )
    # 将生成的 SRDF 内容作为 ROS 参数传入
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # === 第三部分：加载运动学求解器配置（kinematics.yaml）===
    # 注意：这里直接使用了 ur_moveit_config 包中的 kinematics.yaml
    # ⚠️ 如果你的规划组叫 "ur_manipulator"，请确保该 YAML 文件中 key 也是 "ur_manipulator"
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )

    # === 第四部分：定义并配置你的自定义节点 ===
    tutorial_node = Node(
        package="hello_moveit",                          # 节点所在包名
        executable="ur_robot_model_and_state",           # 可执行文件名（C++ 或 Python 节点）
        output="screen",                                 # 将日志输出到终端（方便调试）
        parameters=[
            robot_description,           # 传入 URDF（robot_description）
            robot_description_semantic,  # 传入 SRDF（robot_description_semantic）
            robot_description_kinematics,  # 传入 kinematics.yaml（启用 IK 求解器）
        ],
    )

    # 返回要启动的节点列表（LaunchDescription 会启动这些节点）
    return [tutorial_node]


# 主函数：定义整个 launch 文件的结构
def generate_launch_description():
    # 创建一个列表，用于存放所有可配置的启动参数
    declared_arguments = []

    # 声明 "ur_type" 参数：用户可指定机器人型号（带默认值和可选范围）
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",  # 默认使用 UR5
            description="Type of UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],  # 限制可选值
        )
    )
    # 声明机器人描述包名（默认为官方 ur_description）
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Robot description package.",
        )
    )
    # 声明 URDF 文件名（默认为 ur.urdf.xacro）
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO file name.",
        )
    )
    # 声明 MoveIt 配置包名（默认为 ur_moveit_config）
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package.",
        )
    )
    # 声明 SRDF 文件名
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="SRDF/XACRO file name.",
        )
    )
    # 声明关节前缀（默认为空字符串，注意默认值写成 '""' 是为了在 xacro 中正确解析为空）
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix for joint names (useful in multi-robot setups).",
        )
    )

    # 返回完整的 LaunchDescription：
    # - 包含所有声明的参数（用户可通过命令行覆盖，如 ur_type:=ur3）
    # - 包含一个 OpaqueFunction，它会在参数解析完成后调用 launch_setup 函数
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])