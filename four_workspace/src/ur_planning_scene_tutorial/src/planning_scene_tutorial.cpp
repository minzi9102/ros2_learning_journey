// 引入 rclcpp/rclcpp.hpp，这是 ROS 2 C++ 客户端库的核心头文件，包含了节点、执行器等基本功能。
#include <rclcpp/rclcpp.hpp>

// 引入 MoveIt 相关头文件
// robot_model_loader 用于从参数服务器加载机器人的 URDF 和 SRDF 模型。
#include <moveit/robot_model_loader/robot_model_loader.h>
// planning_scene 提供了表示机器人及其周围环境（世界）的核心类。
#include <moveit/planning_scene/planning_scene.h>
// kinematic_constraints/utils.h 提供了一些方便的工具函数来创建运动学约束。
#include <moveit/kinematic_constraints/utils.h>

// -------------------- 用户自定义状态可行性检测函数示例 --------------------
//
// 除了 MoveIt 内置的约束，我们也可以向 PlanningScene 类指定用户自定义的约束。
// 这可以通过使用 setStateFeasibilityPredicate 函数指定一个回调函数来实现。
// 下面是一个简单的用户自定义回调函数示例，它检查 UR 机器人 "shoulder_pan_joint" 关节的角度是正还是负。
//
// @param robot_state 当前需要被检测的机器人状态。
// @param verbose 是否打印详细的调试信息（在这个例子中未使用）。
// @return 如果状态是“可行”的（这里指关节角度 > 0），则返回 true，否则返回 false。
bool stateFeasibilityTestExample(const moveit::core::RobotState& robot_state, bool /*verbose*/)
{
  // 获取名为 "shoulder_pan_joint" 的关节的当前位置（角度值）。
  const double* joint_values = robot_state.getJointPositions("shoulder_pan_joint");
  // 判断该关节的角度值是否大于 0.0。
  return (joint_values[0] > 0.0);
}
// -------------------- 示例函数结束 --------------------

// 创建一个全局的日志记录器（Logger），用于后续打印信息。
// 日志的名称是 "planning_scene_tutorial"，方便在 ROS 2 的日志系统中过滤和查看。
static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_tutorial");

int main(int argc, char** argv)
{
  // 初始化 ROS 2 C++ 客户端库。这是每个 ROS 2 程序的入口点。
  rclcpp::init(argc, argv);
  
  // 创建节点选项对象，用于配置节点的行为。
  rclcpp::NodeOptions node_options;
  // 设置此选项为 true，将允许节点自动从命令行或启动文件中声明和加载参数。
  node_options.automatically_declare_parameters_from_overrides(true);
  
  // 创建一个名为 "planning_scene_tutorial" 的 ROS 2 节点。
  auto planning_scene_tutorial_node = rclcpp::Node::make_shared("planning_scene_tutorial", node_options);

  // 创建一个单线程执行器，用于处理节点的回调函数（如订阅、服务等）。
  rclcpp::executors::SingleThreadedExecutor executor;
  // 将我们创建的节点添加到执行器中。
  executor.add_node(planning_scene_tutorial_node);
  // 在一个新线程中运行执行器，使其“旋转”（spin），即开始处理事件循环。
  // .detach() 使这个线程在后台独立运行，主线程可以继续执行后续代码。
  std::thread([&executor]() { executor.spin(); }).detach();

  // -------------------- MoveIt 教程开始 --------------------
  
  // ## 设置 ##
  //
  // PlanningScene 类可以很容易地通过 RobotModel 或 URDF/SRDF 文件来设置和配置。
  // 然而，这并不是官方推荐的实例化 PlanningScene 的方式。
  // 官方推荐使用 PlanningSceneMonitor，它能根据机器人关节的实际数据和传感器信息来创建和维护当前的规划场景。
  // 在本教程中，为了演示，我们直接实例化一个 PlanningScene 类。

  // 创建一个 RobotModelLoader 对象，它会自动从 ROS 参数服务器加载名为 "robot_description" 的机器人模型描述。
  robot_model_loader::RobotModelLoader robot_model_loader(planning_scene_tutorial_node, "robot_description");
  // 从加载器中获取机器人模型的智能指针。
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  // 使用加载的机器人运动学模型来实例化一个规划场景（PlanningScene）对象。
  planning_scene::PlanningScene planning_scene(kinematic_model);


  // ## 碰撞检测 ##
  //
  // ### 自碰撞检测 ###
  //
  // 我们要做的第一件事是检查机器人当前状态是否处于“自碰撞”状态，
  // 即机器人当前的关节配置是否会导致其自身的某些部分相互碰撞。
  // 为此，我们需要构造一个 CollisionRequest（碰撞请求）对象和一个 CollisionResult（碰撞结果）对象，
  // 并将它们传递给碰撞检测函数。检测结果会保存在 CollisionResult 对象中。
  // 注意：自碰撞检测使用的是机器人的“无填充”（unpadded）版本，即直接使用 URDF 中定义的碰撞网格，没有额外的安全边距。

  // 创建碰撞请求对象，用于指定碰撞检测的参数。
  collision_detection::CollisionRequest collision_request;
  // 创建碰撞结果对象，用于存储碰撞检测的结果。
  collision_detection::CollisionResult collision_result;
  // 调用 checkSelfCollision 函数来检查机器人当前默认状态下的自碰撞。
  planning_scene.checkSelfCollision(collision_request, collision_result);
  // 打印日志，显示当前状态是否处于自碰撞中。
  RCLCPP_INFO_STREAM(LOGGER, "测试 1: 当前状态 " << (collision_result.collision ? "处于" : "不处于") << " 自碰撞");

  // ### 改变机器人状态 ###
  //
  // 现在，让我们改变机器人的当前状态。PlanningScene 内部维护着一个当前状态。
  // 我们可以获取它的引用，改变它，然后为新的机器人配置再次进行碰撞检测。
  // 特别注意：在进行新的碰撞检测请求之前，我们需要清除上一次的 collision_result。

  // 获取一个指向当前状态的可修改引用。
  moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  // 将当前状态设置为一个随机的关节位置。
  current_state.setToRandomPositions();
  // 清除上一次的碰撞结果，为新的检测做准备。
  collision_result.clear();
  // 再次进行自碰撞检测。
  planning_scene.checkSelfCollision(collision_request, collision_result);
  // 打印日志，显示随机状态下的自碰撞情况。
  RCLCPP_INFO_STREAM(LOGGER, "测试 2: 当前状态 " << (collision_result.collision ? "处于" : "不处于") << " 自碰撞");

  // ### 针对特定规划组的检测 ###
  // 更高效的碰撞检测
  //
  // 现在，我们只对 UR 机械臂的末端工具（tool0）进行碰撞检测，
  // 即检查末端工具与机器人其他部分之间是否存在碰撞。
  // 我们可以通过在碰撞请求中指定规划组的名称 "tool0" 来实现这一点。

  // 在碰撞请求中指定要检测的规划组名称。
  collision_request.group_name = "tool0";
  // 将机器人设置为一个新的随机位置。
  current_state.setToRandomPositions();
  // 清除上一次的碰撞结果。
  collision_result.clear();
  // 再次进行自碰撞检测，但这次只针对 "tool0" 规划组。
  planning_scene.checkSelfCollision(collision_request, collision_result);
  // 打印日志，显示对 "tool0" 组进行检测的结果。
  RCLCPP_INFO_STREAM(LOGGER, "测试 3: 当前状态 " << (collision_result.collision ? "处于" : "不处于") << " 自碰撞");

  // ### 获取碰撞接触信息 ###
  //
  // 首先，手动将 UR 机械臂设置到一个我们已知的会发生内部（自）碰撞的位置。
  // 注意，这个状态实际上超出了 UR 机械臂的关节限制，我们也可以直接检查这一点。

  // 定义一组会导致碰撞的关节角度值。
  std::vector<double> joint_values = { 0.0, -1.57, 3.14, -1.57, 0.0, 0.0 };
  // 获取 "ur_manipulator" 规划组的模型信息。
  const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("ur_manipulator");
  // 将 "ur_manipulator" 规划组的关节位置设置为我们上面定义的值。
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  // 检查当前状态是否满足关节限制（例如，角度范围），并打印结果。
  RCLCPP_INFO_STREAM(LOGGER, "测试 4: 当前状态 " << (current_state.satisfiesBounds(joint_model_group) ? "有效" : "无效"));

  // 现在，我们可以获取在 UR 机械臂的特定配置下可能发生的任何碰撞的接触信息。
  // 我们可以通过在碰撞请求中填充相应的字段来请求接触信息，并将要返回的最大接触点数量指定为一个大数。

  // 在请求中设置 `contacts` 为 true，表示我们需要详细的接触信息。
  collision_request.contacts = true;
  // 设置希望返回的最大接触点对数量。
  collision_request.max_contacts = 1000;
  
  // 清除上一次的碰撞结果。
  collision_result.clear();
  // 再次执行自碰撞检测。
  planning_scene.checkSelfCollision(collision_request, collision_result);
  // 打印日志，显示当前状态是否处于自碰撞。
  RCLCPP_INFO_STREAM(LOGGER, "测试 5: 当前状态 " << (collision_result.collision ? "处于" : "不处于") << " 自碰撞");
  
  // 创建一个迭代器来遍历碰撞结果中的接触点图谱（ContactMap）。
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  // 遍历所有检测到的接触点对。
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
    // 打印发生碰撞的两个链接（link）的名称。
    RCLCPP_INFO(LOGGER, "接触发生在: %s 和 %s 之间", it->first.first.c_str(), it->first.second.c_str());
  }

  // ### 修改允许碰撞矩阵 (Allowed Collision Matrix, ACM) ###
  //
  // ACM 提供了一种机制，可以告诉碰撞世界忽略某些对象之间的碰撞：
  // 这些对象可以是机器人的不同部分，也可以是机器人与世界中的物体。
  // 我们可以告诉碰撞检测器忽略上面报告的所有链接之间的碰撞。
  // 这样一来，即使这些链接实际上处于碰撞状态，碰撞检测器也会忽略它们并返回“未碰撞”。
  
  // 从规划场景中获取当前的允许碰撞矩阵 (ACM) 的一个副本。
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  // 创建当前机器人状态的一个副本。
  moveit::core::RobotState copied_state = planning_scene.getCurrentState();

  // 再次创建一个迭代器来遍历刚才检测到的接触点。
  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
  {
    // 在 ACM 中将被检测到碰撞的链接对设置为“允许碰撞”（true）。
    acm.setEntry(it2->first.first, it2->first.second, true);
  }
  // 清除上一次的碰撞结果。
  collision_result.clear();
  // 使用修改后的 ACM 和状态副本再次进行自碰撞检测。
  planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
  // 打印日志。由于 ACM 忽略了这些碰撞，结果应该是“不处于自碰撞”。
  RCLCPP_INFO_STREAM(LOGGER, "测试 6: 当前状态 " << (collision_result.collision ? "处于" : "不处于") << " 自碰撞");

  // ### 完整的碰撞检测 ###
  //
  // 虽然我们一直在进行自碰撞检测，但我们也可以使用 checkCollision 函数，
  // 它会同时检查自碰撞和与环境的碰撞（当前环境是空的）。
  // 这是你在规划器中最常使用的一组碰撞检测函数。
  // 注意：与环境的碰撞检测将使用机器人的“带填充”（padded）版本。
  // 填充（Padding）有助于让机器人与环境中的障碍物保持更远的距离。
  
  // 清除上一次的碰撞结果。
  collision_result.clear();
  // 调用 checkCollision，它会检查自碰撞和与世界物体的碰撞。
  planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
  // 打印日志，显示结果。
  RCLCPP_INFO_STREAM(LOGGER, "测试 7: 当前状态 " << (collision_result.collision ? "处于" : "不处于") << " 碰撞");

  // ## 约束检测 ##
  //
  // PlanningScene 类还包含易于使用的函数调用来检查约束。约束可以是两种类型：
  // (a) 从 KinematicConstraint 集合中选择的约束：即关节约束、位置约束、方向约束和可见性约束。
  // (b) 通过回调函数指定的用户自定义约束。我们将首先看一个简单的运动学约束的例子。
  //
  // ### 检查运动学约束 ###
  //
  // 我们将首先为 UR 机械臂 "ur_manipulator" 组的末端执行器定义一个简单的位置和方向约束。
  // 注意这里使用了便利函数来填充约束（这些函数位于 moveit_core 的 kinematic_constraints 目录下的 utils.h 文件中）。

  // 获取规划组中最后一个链接（通常是末端执行器）的名称。
  std::string end_effector_name = joint_model_group->getLinkModelNames().back();
  
  // 创建一个 geometry_msgs::msg::PoseStamped 对象来存储期望的位姿。
  geometry_msgs::msg::PoseStamped desired_pose;
  // 设置期望的方向（四元数，w=1.0 表示没有旋转）。
  desired_pose.pose.orientation.w = 1.0;
  // 设置期望的位置坐标。
  desired_pose.pose.position.x = 0.3;
  desired_pose.pose.position.y = -0.185;
  desired_pose.pose.position.z = 0.5;
  // 设置该位姿所在的参考坐标系。
  desired_pose.header.frame_id = "base";
  
  // 使用工具函数 `constructGoalConstraints` 将期望的位姿转换为 MoveIt 的约束消息格式。
  moveit_msgs::msg::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);

  // 现在，我们可以使用 PlanningScene 类中的 isStateConstrained 函数来检查一个状态是否满足这个约束。

  // 将状态副本设置为一个新的随机位置。
  copied_state.setToRandomPositions();
  // 更新状态（例如，更新正向运动学计算）。
  copied_state.update();
  // 检查这个随机状态是否满足我们定义的目标约束。
  bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
  // 打印日志，显示随机状态是否受约束。
  RCLCPP_INFO_STREAM(LOGGER, "测试 8: 随机状态 " << (constrained ? "受约束" : "不受约束"));
  
  // 有一种更高效的检查约束的方法（当你需要反复检查同一个约束时，例如在规划器内部）。
  // 我们首先构造一个 KinematicConstraintSet 对象，它会预处理 ROS 的约束消息，以便进行快速处理。

  // 创建一个运动学约束集对象。
  kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
  // 将目标约束添加到集合中，并提供场景的变换信息。
  kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
  // 使用这个预处理过的约束集来检查状态。
  bool constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
  // 打印日志，结果应该和上面一样。
  RCLCPP_INFO_STREAM(LOGGER, "测试 9: 随机状态 " << (constrained_2 ? "受约束" : "不受约束"));
  
  // 还有一种更直接的方法是使用 KinematicConstraintSet 类自身的功能。

  // 直接调用约束集的 `decide` 方法来评估状态是否满足约束。
  kinematic_constraints::ConstraintEvaluationResult constraint_eval_result =
      kinematic_constraint_set.decide(copied_state);
  // 打印日志，显示评估结果。
  RCLCPP_INFO_STREAM(LOGGER, "测试 10: 随机状态 " << (constraint_eval_result.satisfied ? "满足约束" : "不满足约束"));

  // ### 用户自定义约束 ###
  //
  // 现在，我们将使用本文件开头定义的 `stateFeasibilityTestExample` 函数。

  // 使用 setStateFeasibilityPredicate 将我们的自定义回调函数注册到规划场景中。
  // 现在，每当调用 isStateFeasible 时，这个用户定义的回调函数就会被执行。
  planning_scene.setStateFeasibilityPredicate(stateFeasibilityTestExample);
  // 调用 isStateFeasible 来检查状态是否“可行”（即是否满足我们的自定义回调函数）。
  bool state_feasible = planning_scene.isStateFeasible(copied_state);
  // 打印日志，显示随机状态是否可行。
  RCLCPP_INFO_STREAM(LOGGER, "测试 11: 随机状态 " << (state_feasible ? "可行" : "不可行"));
  
  // 每当调用 isStateValid 时，会进行三项检查：
  // (a) 碰撞检测
  // (b) 运动学约束检测
  // (c) 使用用户自定义回调函数进行的可行性检测

  // 调用 isStateValid，它将执行所有类型的检查：碰撞、运动学约束和自定义可行性。
  bool state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "ur_manipulator");
  // 打印日志，显示随机状态是否“有效”。
  RCLCPP_INFO_STREAM(LOGGER, "测试 12: 随机状态 " << (state_valid ? "有效" : "无效"));

  // -------------------- MoveIt 教程结束 --------------------

  // 关闭 ROS 2 C++ 客户端库，释放资源。
  rclcpp::shutdown();
  // 程序正常退出。
  return 0;
}