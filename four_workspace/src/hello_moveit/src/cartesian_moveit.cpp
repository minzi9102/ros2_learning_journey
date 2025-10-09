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
    "cartesian_moveit", // 节点的名称，在ROS网络中必须是唯一的
    // 节点选项：允许从外部（如launch文件或命令行）自动声明和覆盖参数
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // 创建一个日志记录器（Logger），用于在控制台打印信息、警告或错误
  auto const logger = rclcpp::get_logger("cartesian_moveit");

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
  // 3. 设定目标位姿,使用笛卡尔空间轨迹 (Set a target Pose)
  // ---------------------------------------------------------------------------
  RCLCPP_INFO(logger, "Planning a Cartesian path...");
  // 创建一个存储路点(waypoints)的 vector
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // a. 获取当前位姿作为路径的起始点
  auto const start_pose = move_group_interface.getCurrentPose().pose;
  waypoints.push_back(start_pose);

  // b. 在当前位姿的基础上定义几个路点
  geometry_msgs::msg::Pose target_pose1 = start_pose;

  // 沿 X 轴正方向移动 10 cm
  target_pose1.position.x += 0.1;

  // 沿 Y 轴正方向移动 10 cm
  target_pose1.position.y += 0.1;
  waypoints.push_back(target_pose1);

  // c. 沿 Z 轴负方向移动 10 cm
  geometry_msgs::msg::Pose target_pose2 = target_pose1;
  target_pose2.position.z -= 0.1;
  waypoints.push_back(target_pose2);

  // d. 回到起始位姿
  waypoints.push_back(start_pose);

    // ---------------------------------------------------------------------------
    // 4. 进行笛卡尔运动规划
    // ---------------------------------------------------------------------------
    // 参数说明:
    // 1. waypoints: 需要经过的路点向量
    // 2. eef_step: 笛卡尔空间中的插值步长，单位：米。值越小，路径越精细。
    // 3. jump_threshold: 关节空间中的跳跃阈值。设为0表示禁用跳跃检测。
    // 4. trajectory: 存储规划结果的轨迹对象
    // 5. avoid_collisions: 是否进行碰撞检测
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction < 1.0)
    {
        RCLCPP_ERROR(logger, "Cartesian path planning failed! (fraction: %f)", fraction);
    }
    else
    {
        RCLCPP_INFO(logger, "Cartesian path planning succeeded!");
        // ---------------------------------------------------------------------------
        // 5. 执行规划 (Execute the plan)
        // ---------------------------------------------------------------------------
        move_group_interface.execute(trajectory);
    }


  // ---------------------------------------------------------------------------
  // 6. 关闭 ROS2
  // ---------------------------------------------------------------------------
  // 关闭ROS2客户端库，释放资源
  rclcpp::shutdown();
  // 程序正常退出
  return 0;
}