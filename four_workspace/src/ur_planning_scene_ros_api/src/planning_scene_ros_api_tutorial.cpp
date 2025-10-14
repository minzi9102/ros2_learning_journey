#include <rclcpp/rclcpp.hpp>                    // ROS 2 的 C++ 客户端库，用于创建节点、发布/订阅等
#include <geometry_msgs/msg/pose.hpp>           // 用于表示三维空间中位置和姿态的消息类型

// MoveIt 相关头文件：用于操作规划场景（planning scene）、碰撞检测、机器人状态等
#include <moveit_msgs/msg/planning_scene.hpp>                // 规划场景消息，用于描述环境中的障碍物、机器人状态等
#include <moveit_msgs/msg/attached_collision_object.hpp>     // 表示附着在机器人上的碰撞物体
#include <moveit_msgs/srv/get_state_validity.hpp>            // 用于检查机器人状态是否有效的服务（本例未使用）
#include <moveit_msgs/msg/display_robot_state.hpp>           // 用于在 RViz 中显示机器人状态（本例未使用）
#include <moveit_msgs/srv/apply_planning_scene.hpp>          // 通过服务同步应用规划场景更改

#include <moveit/robot_model_loader/robot_model_loader.h>    // 从 URDF/SRDF 加载机器人模型
#include <moveit/robot_state/robot_state.h>                  // 表示机器人当前状态（关节位置、附着物体等）
#include <moveit/robot_state/conversions.h>                  // 在 ROS 消息与 MoveIt 内部状态之间转换

#include <rviz_visual_tools/rviz_visual_tools.hpp>           // 用于在 RViz 中可视化调试信息（如标记、提示等）

// 定义一个日志器，用于输出日志信息（如 INFO、ERROR 等）
static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_ros_api_tutorial");

int main(int argc, char** argv)
{
  // 初始化 ROS 2 通信系统
  rclcpp::init(argc, argv);

  // 创建节点选项：允许通过命令行或 launch 文件自动声明参数
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // 创建一个名为 "planning_scene_ros_api_tutorial" 的 ROS 2 节点
  auto node = rclcpp::Node::make_shared("planning_scene_ros_api_tutorial", node_options);

  // 创建一个单线程执行器（executor），用于运行该节点的回调函数
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // 启动一个后台线程来运行 executor（即持续处理 ROS 消息、服务等）
  // 使用 detach() 让主线程继续执行，而不阻塞
  std::thread([&executor]() { executor.spin(); }).detach();

  // ==================== 教程开始 ====================

  // 可视化工具设置
  // ^^^^^^^^^^^^^^^^^
  // rviz_visual_tools 是一个辅助工具，可以在 RViz 中可视化机器人、轨迹、碰撞物体等，
  // 并提供“下一步”按钮来逐步执行脚本（用于教学/调试）
  rviz_visual_tools::RvizVisualTools visual_tools("base_link", "planning_scene_ros_api_tutorial", node);
  visual_tools.loadRemoteControl();   // 加载 RViz 中的“下一步”控制面板
  visual_tools.deleteAllMarkers();    // 清除 RViz 中之前的所有可视化标记

  // ROS API：通过话题发布规划场景差异（diff）
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // MoveIt 的规划场景由 move_group 节点维护。用户不能直接替换整个场景，
  // 而是通过发布“差异（diff）”来增量更新场景。
  // 这里我们创建一个发布者，向 "planning_scene" 话题发布 PlanningScene 消息。

  // 创建一个发布者，用于发布规划场景的差异
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
      node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

  // 等待至少一个订阅者（通常是 move_group 节点）连接到该话题
  // 如果没有订阅者，发布的消息会被丢弃
  while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500)); // 每 500 毫秒检查一次
  }

  // 在 RViz 中弹出提示，等待用户点击“next”继续
  visual_tools.prompt("按 RVizVisualToolsGui 窗口中的 'next' 继续演示");

  // 定义一个附着在机器人上的碰撞物体（例如：机器人抓取的箱子）
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "tool0";  // 指定该物体附着在哪个机器人连杆上（通常是末端执行器）

  // 设置物体的坐标系（必须是 TF 中存在的有效坐标系）
  attached_object.object.header.frame_id = "tool0";

  // 给物体一个唯一 ID，用于后续识别和操作（如移除、附着等）
  attached_object.object.id = "box";

  // 定义物体相对于附着连杆的位姿（pose）
  geometry_msgs::msg::Pose pose;
  pose.position.z = 0.0;      // 在 tool0 坐标系下，物体位于 z 轴正方向 0.11 米处
  pose.orientation.w = 1.0;    // 单位四元数，表示无旋转（w=1, x=y=z=0）

  // 定义一个立方体（box）形状
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;               // 类型为立方体
  primitive.dimensions.resize(3);               // 立方体需要 3 个维度：长、宽、高
  primitive.dimensions[0] = 0.075;              // X 方向尺寸（米）
  primitive.dimensions[1] = 0.075;              // Y 方向尺寸
  primitive.dimensions[2] = 0.075;              // Z 方向尺寸

  // 将形状和位姿添加到物体中
  attached_object.object.primitives.push_back(primitive);        // 添加几何形状
  attached_object.object.primitive_poses.push_back(pose);        // 添加对应位姿

  // 指定操作类型为“添加”（ADD），表示要将此物体加入场景
  attached_object.object.operation = attached_object.object.ADD;

  // 设置“接触连杆”列表：当物体附着在机器人上时，
  // 这些连杆与物体之间的碰撞将被忽略（避免自碰撞误报）
  // 此处只忽略与 "tool0" 的碰撞
  attached_object.touch_links = std::vector<std::string>{ "tool0","wrist_3_link","'wrist_1_link","'wrist_2_link"};

  // 第一步：将物体添加到环境中（尚未附着到机器人）
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  RCLCPP_INFO(LOGGER, "将物体添加到世界中（位于机械手当前位置）");

  // 创建一个规划场景消息（仅包含差异部分）
  moveit_msgs::msg::PlanningScene planning_scene;
  // 将物体添加到“世界”中的碰撞物体列表（注意：此时还未附着到机器人）
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;  // 标记这是一个“差异”消息，只更新指定部分

  // 发布该差异，move_group 节点会将其合并到当前规划场景中
  planning_scene_diff_publisher->publish(planning_scene);

  // 等待用户点击“next”继续
  visual_tools.prompt("按 RVizVisualToolsGui 窗口中的 'next' 继续演示");

  // 插曲：同步 vs 异步更新规划场景
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 有两种方式更新规划场景：
  // 1. **异步**：通过话题发布（如上），不等待确认（可能尚未生效）
  // 2. **同步**：通过服务调用，阻塞直到更新完成（更可靠）

  // 下面演示如何使用服务进行同步更新（虽然前面已用话题发布过）

  // 创建一个客户端，用于调用 apply_planning_scene 服务
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr planning_scene_diff_client =
      node->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");

  // 等待服务可用
  planning_scene_diff_client->wait_for_service();

  // 构造服务请求，将之前定义的 planning_scene 作为请求内容
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene = planning_scene;

  // 异步发送服务请求，并获取 future 对象用于等待响应
  std::shared_future<std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response>> response_future;
  response_future = planning_scene_diff_client->async_send_request(request).future.share();

  // 等待最多 1 秒钟获取服务响应
  std::chrono::seconds wait_time(1);
  std::future_status fs = response_future.wait_for(wait_time);

  if (fs == std::future_status::timeout)
  {
    RCLCPP_ERROR(LOGGER, "服务调用超时。");
  }
  else
  {
    // 获取响应
    std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene_Response> planning_response;
    planning_response = response_future.get();

    if (planning_response->success)
    {
      RCLCPP_INFO(LOGGER, "服务成功添加了物体。");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "服务未能成功添加物体。");
    }
  }

  // 注意：使用服务的方式会阻塞直到确认更新完成，而话题方式不会

  // 第二步：将物体从环境中移除，并附着到机器人上（模拟“抓取”）
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // 首先定义一个“移除”操作的消息，用于从环境中删除该物体
  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = "box";                      // 要移除的物体 ID
  remove_object.header.frame_id = "tool0";       // 坐标系（虽不关键，但需有效）
  remove_object.operation = remove_object.REMOVE; // 操作类型为“移除”

  RCLCPP_INFO(LOGGER, "将物体附着到机械手，并从环境中移除。");

  // 清空之前的差异内容，确保只包含本次操作
  planning_scene.world.collision_objects.clear();                 // 清空世界中的物体
  planning_scene.robot_state.attached_collision_objects.clear();  // 清空机器人附着物体

  // 添加“移除世界物体” + “附着到机器人”两个操作
  planning_scene.world.collision_objects.push_back(remove_object); // 从世界中删除
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object); // 附着到机器人
  planning_scene.robot_state.is_diff = true;  // 标记机器人状态部分为差异
  planning_scene.is_diff = true;              // 整体为差异消息

  // 发布更新
  planning_scene_diff_publisher->publish(planning_scene);
  visual_tools.prompt("按 RVizVisualToolsGui 窗口中的 'next' 继续演示");

  // 第三步：将物体从机器人上分离，并放回环境中（模拟“放置”）
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // 定义一个“分离”操作的消息
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";                          // 要分离的物体 ID
  detach_object.link_name = "tool0";                        // 从哪个连杆分离
  detach_object.object.operation = attached_object.object.REMOVE; // 操作为“移除附着”

  RCLCPP_INFO(LOGGER, "将物体从机器人上分离，并放回环境中。");

  // 清空差异内容
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();

  // 添加“分离附着物体” + “重新添加到世界”两个操作
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object); // 分离
  planning_scene.robot_state.is_diff = true;

  // 注意：这里重新使用最初定义的 attached_object.object（包含形状和位姿）来放回世界
  planning_scene.world.collision_objects.push_back(attached_object.object); // 放回世界
  planning_scene.is_diff = true;

  // 发布更新
  planning_scene_diff_publisher->publish(planning_scene);
  visual_tools.prompt("按 RVizVisualToolsGui 窗口中的 'next' 继续演示");

  // 第四步：彻底从环境中删除该物体
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  RCLCPP_INFO(LOGGER, "从环境中彻底删除该物体。");

  // 清空所有内容，只保留“移除”操作
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object); // 再次移除
  planning_scene.is_diff = true;

  // 发布最终删除操作
  planning_scene_diff_publisher->publish(planning_scene);

  // 等待用户结束演示
  visual_tools.prompt("按 RVizVisualToolsGui 窗口中的 'next' 结束演示");

  // 关闭 ROS 2 通信
  rclcpp::shutdown();
  return 0;
}