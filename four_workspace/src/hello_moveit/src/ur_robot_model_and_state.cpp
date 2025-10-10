// MoveIt 相关头文件
#include <moveit/robot_model_loader/robot_model_loader.h>  // 用于从参数服务器加载机器人模型
#include <moveit/robot_model/robot_model.h>                // 机器人模型类（包含连杆、关节、规划组等信息）
#include <moveit/robot_state/robot_state.h>                // 机器人状态类（表示某一时刻的关节配置）

int main(int argc, char** argv)
{
  // 初始化 ROS 2 客户端库
  rclcpp::init(argc, argv);

  // 配置节点选项：允许自动声明通过命令行传入的未声明参数
  // 最佳实践是显式声明参数，但此处为简化教程启用自动声明
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("robot_model_and_state_tutorial", node_options);
  const auto& LOGGER = node->get_logger();  // 获取日志记录器

  // BEGIN_TUTORIAL
  // 开始使用 RobotModel
  // ^^^^^^^^^^^^^^^^^^^
  // 使用 RobotModel 类非常简单。通常，更高层的 MoveIt 组件（如 PlanningScene）会返回一个
  // RobotModel 的智能指针。应尽可能使用该指针。本例中，我们将从 RobotModelLoader 开始，
  // 它会从 ROS 参数服务器读取机器人描述（URDF + SRDF），并为我们构建一个 RobotModel 对象。
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(LOGGER, "机器人模型参考坐标系: %s", kinematic_model->getModelFrame().c_str());

  // 使用 RobotModel 创建一个 RobotState 对象，用于表示机器人的具体位姿（关节值）。
  // 我们将其初始化为默认关节值（通常来自 URDF 中的 <limit> 或 <default>）。
  // 然后获取一个 JointModelGroup（关节模型组），例如 UR 机器人的 "ur_manipulator" 规划组。
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();  // 设置为默认关节值
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("ur_manipulator");

  // 获取该规划组中所有关节的名称列表
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // 获取关节值
  // ^^^^^^^^^^^^
  // 从当前 RobotState 中提取指定规划组的关节值
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "关节 %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // 关节限位处理
  // ^^^^^^^^^^^^^^
  // 注意：setJointGroupPositions() 本身不会检查关节限位，但调用 enforceBounds() 可强制将值限制在合法范围内。
  /* 将第一个关节设为超出其限位的值（例如 5.57 弧度） */
  joint_values[0] = 5.57;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  /* 检查当前状态是否满足所有关节限位 */
  RCLCPP_INFO_STREAM(LOGGER, "当前状态 " << (kinematic_state->satisfiesBounds() ? "有效" : "无效"));

  /* 强制将关节值限制在合法范围内，并再次检查 */
  kinematic_state->enforceBounds();
  RCLCPP_INFO_STREAM(LOGGER, "修正后状态 " << (kinematic_state->satisfiesBounds() ? "有效" : "无效"));

  // 正向运动学（FK）
  // ^^^^^^^^^^^^^^^^^
  // 为规划组设置一组随机关节值，然后计算末端执行器（"tool0"）在世界坐标系下的位姿。
  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");
  RCLCPP_INFO_STREAM(LOGGER, "为机械臂设置一组随机的关节值，并计算正运动学，结果如下：\n");
  /* 打印末端执行器的平移和旋转（基于模型参考坐标系） */
  RCLCPP_INFO_STREAM(LOGGER, "平移:\n" << end_effector_state.translation() << "\n");
  RCLCPP_INFO_STREAM(LOGGER, "旋转:\n" << end_effector_state.rotation() << "\n");

  // 逆向运动学（IK）
  // ^^^^^^^^^^^^^^^^^
  // 现在尝试求解逆运动学：给定末端执行器的目标位姿，反求满足该位姿的关节值。
  // 需要提供：
  //   - 目标位姿（即上面计算出的 end_effector_state）
  //   - 求解超时时间（0.1 秒）
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // 如果找到 IK 解，则打印关节值
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(LOGGER, "关节 %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    RCLCPP_INFO(LOGGER, "未找到逆运动学解");
  }

  // 获取雅可比矩阵（Jacobian）
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 雅可比矩阵描述了关节速度与末端执行器线速度/角速度之间的关系。
  // 我们计算规划组末端连杆（通常是最后一个连杆）相对于参考点（此处为原点）的雅可比矩阵。
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);  // 参考点在末端连杆坐标系中的位置
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(
      joint_model_group,
      kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),  // 末端连杆
      reference_point_position,
      jacobian
  );
  RCLCPP_INFO_STREAM(LOGGER, "雅可比矩阵:\n" << jacobian << "\n");
  // END_TUTORIAL

  // 关闭 ROS 2
  rclcpp::shutdown();
  return 0;
}