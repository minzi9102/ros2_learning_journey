// 引入 C++ 标准库头文件
#include <cstdio>    // 用于标准输入输出，这里虽然没直接用，但作为基础库通常会包含
#include <memory>    // 用于智能指针，比如 std::make_shared
#include <vector> // 需要引入 <vector> 头文件

// 引入 ROS2 和 MoveIt 的核心头文件
#include <rclcpp/rclcpp.hpp>  // ROS2 C++ 客户端库的核心功能
#include <moveit/move_group_interface/move_group_interface.h> // MoveIt 的主要接口类 MoveGroupInterface

// C++ 程序主函数入口
int main(int argc, char * argv[])
{
  // ---------------------------------------------------------------------------
  // 1. 初始化 ROS2
  // ---------------------------------------------------------------------------
  // 初始化ROS2客户端库，这是所有ROS2程序运行前必须调用的第一步
  rclcpp::init(argc, argv);
  
  // 创建一个ROS2节点。节点是ROS网络中的一个执行单元。
  // std::make_shared 用于创建一个指向 rclcpp::Node 对象的共享指针
  auto const node = std::make_shared<rclcpp::Node>(
    "nihao_moveit", // 节点的名称，在ROS网络中必须是唯一的
    // 节点选项：允许从外部（如launch文件或命令行）自动声明和覆盖参数
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // 创建一个日志记录器（Logger），用于在控制台打印信息、警告或错误
  auto const logger = rclcpp::get_logger("nihao_moveit");

  // ---------------------------------------------------------------------------
  // 2. 设置 MoveIt
  // ---------------------------------------------------------------------------
  // 为了方便使用，通过 using 声明来简化 MoveGroupInterface 的类型名称
  using moveit::planning_interface::MoveGroupInterface;
  // 创建 MoveGroupInterface 的实例
  // 这是与 MoveIt 通信的核心对象，用于规划和执行动作
  // 参数1: 需要一个ROS2节点来进行通信
  // 参数2: "panda_arm" 是你在机器人 SRDF 文件中定义的“规划组”的名称。
  //        它告诉 MoveIt 你想要控制的是哪一部分，比如机械臂部分，而不是手爪或整台机器人。
//   auto move_group_interface = MoveGroupInterface(node, "panda_arm");
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // ---------------------------------------------------------------------------
  // 3. 设定目标位姿 (Set a target Pose)
  // ---------------------------------------------------------------------------

  //   // 使用一个立即执行的 lambda 表达式来创建一个目标位姿对象
  // auto const target_pose = []{
  //   geometry_msgs::msg::Pose msg; // geometry_msgs::msg::Pose 是ROS中用于表示位姿的标准消息类型
  //   // 设置姿态：使用四元数表示。w=1.0, x=y=z=0 表示没有任何旋转的“标准”姿态。
  //   msg.orientation.w = 1.0;
  //   // 设置位置：相对于规划坐标系（通常是机器人基座）的 x, y, z 坐标（单位：米）
  //   msg.position.x = 0.28;
  //   msg.position.y = -0.2;
  //   msg.position.z = 0.5;
  //   return msg;
  // }(); // lambda表达式定义完后加()表示立即调用
  
  // // 将上面创建的目标位姿设置为 MoveGroup 的目标
  // move_group_interface.setPoseTarget(target_pose);
    // 使用 std::map<string, double> 来设置目标，无需关心顺序
    std::map<std::string, double> target_joints;
    // 假设 "panda_arm" 包含以下关节，并为其设置目标值（弧度）
    target_joints["shoulder_pan_joint"] = 0.0;
    target_joints["shoulder_lift_joint"] = -0.785;
    target_joints["elbow_joint"] = 1.571;
    target_joints["wrist_1_joint"] = -2.356;
    target_joints["wrist_2_joint"] = 0.0;
    target_joints["wrist_3_joint"] = 1.571;

    // 使用 map 设置目标
    bool success_set_target = move_group_interface.setJointValueTarget(target_joints);

  // 检查目标设置是否成功（例如，目标值是否在关节限位内）
  if (!success_set_target)
  {
      RCLCPP_ERROR(logger, "Failed to set joint value target.");
      rclcpp::shutdown();
      return 1;
  }

  // ---------------------------------------------------------------------------
  // 4. 进行运动规划 (Create a plan to that target pose)
  // ---------------------------------------------------------------------------
  // 再次使用一个立即执行的 lambda 和 C++17 的结构化绑定来获取规划结果
  // auto const [success, plan] 会自动接收 lambda 返回的 pair，并解构成两个变量
  auto const [success, plan] = [&move_group_interface]{
    // 创建一个用于存储规划结果的 Plan 对象
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    // 调用 .plan() 方法进行运动规划。规划器会尝试找到一条从当前状态到目标位姿的路径。
    // 规划结果会填充到传入的 msg 对象中。
    // 函数的返回值是一个枚举，表示规划是否成功。我们将其转换为布尔值。
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    // 返回一个包含成功状态和具体规划路径的 std::pair
    return std::make_pair(ok, msg);
  }();

  // ---------------------------------------------------------------------------
  // 5. 执行规划 (Execute the plan)
  // ---------------------------------------------------------------------------
  // 检查上一步的规划是否成功
  if (success) {
    // 如果规划成功，就调用 .execute() 方法，让机器人按照生成的轨迹（plan）运动
    move_group_interface.execute(plan);
  } else {
    // 如果规划失败，使用日志记录器打印一条错误信息
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // ---------------------------------------------------------------------------
  // 6. 关闭 ROS2
  // ---------------------------------------------------------------------------
  // 关闭ROS2客户端库，释放资源
  rclcpp::shutdown();
  // 程序正常退出
  return 0;
}