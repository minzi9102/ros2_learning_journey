#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 上面两行是标准的 Python 脚本文件头。
# #!/usr/bin/env python3 告诉系统这是一个可执行脚本，并且使用 python3 解释器来运行。
# -*- coding: utf-8 -*- 指定文件编码为 UTF-8，这样就可以在代码中包含中文字符。


"""
UR3 轨迹发布节点 (这是一个多行注释，通常用作模块的说明文档)
功能：
发布一条 trajectory_msgs/JointTrajectory 类型的消息到名为
/scaled_joint_trajectory_controller/joint_trajectory 的话题上。
目的是让 UR3 机器人（尤其是在仿真环境中）执行一个简单的预设轨迹。

运行方式：
确保 ROS 2 环境已经正确设置，并且相关的控制器节点正在运行。
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.0.100 use_fake_hardware:=true
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3 \
  use_fake_hardware:=true \
  launch_rviz:=true
然后在另一个终端运行这个脚本：
cd ./ros2_ws2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ur3_trajectory_publisher trajectory_publisher.py
"""

# ---------- 导入 ROS 2 相关功能包 ----------
# 导入所有编写 ROS 2 Python 节点所必需的库和消息类型。

import rclpy  # 导入 ROS 2 的 Python 客户端库（rcl：ROS Client Library），提供了所有核心功能，如初始化、节点创建、消息收发等。
from rclpy.node import Node  # 从 rclpy.node 模块中导入 Node 类。我们自定义的节点类需要继承这个基类。
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # 导入机器人轨迹相关的消息类型。
                                                                        # JointTrajectory: 用于定义一条完整的运动轨迹，包含多个轨迹点。
                                                                        # JointTrajectoryPoint: 用于定义轨迹中的一个具体点，包含位置、速度、加速度等信息。
from builtin_interfaces.msg import Duration  # 导入 Duration 消息类型，它用于表示一个时间段（例如，几秒几纳秒）。


# ---------- 自定义节点 ----------
# 定义一个名为 Ur3TrajectoryPublisher 的类，它继承自 rclpy.node.Node。
# 这意味着我们的类将拥有 ROS 2 节点的所有基本功能。
class Ur3TrajectoryPublisher(Node):
    # 类的构造函数（或初始化方法），当创建这个类的实例时会自动调用。
    def __init__(self):
        # 调用父类（Node）的构造函数，并为这个节点指定一个名字 'ur3_trajectory_publisher'。
        # 这个名字在 ROS 2 网络中必须是唯一的，用于识别该节点。
        super().__init__('ur3_trajectory_publisher')

        # 使用 self.create_publisher() 方法创建一个发布者实例。
        # 发布者的作用是向一个特定的话题发送消息。
        self.publisher_ = self.create_publisher(
            JointTrajectory,  # 参数1: 消息类型，指定这个发布者将要发送的是 JointTrajectory 类型的消息。
            '/scaled_joint_trajectory_controller/joint_trajectory',  # 参数2: 话题名称，这是消息将被发送到的地址。
                                                                   # 注意：这个话题是控制器节点订阅的，所以我们必须发布到这里。
            10  # 参数3: 服务质量(QoS)设置中的队列大小。如果消息发布得很快但网络处理不过来，
                # 这里可以缓存最多 10 条消息，防止旧消息丢失。
        )

        # 使用 self.create_timer() 方法创建一个定时器。
        # 定时器会在指定的时间间隔后，自动调用一个回调函数。
        self.timer = self.create_timer(
            10.0,  # 参数1: 定时周期（秒）。这里设置为 1.0 秒，意味着 1 秒后会触发一次。
            self.timer_callback  # 参数2: 回调函数。当定时器触发时，这个函数将被执行。
        )
        
        # 使用节点的日志记录器打印一条信息到控制台，表示节点已经成功启动。
        # .info() 表示这是一条参考信息。
        self.get_logger().info("UR3 Trajectory Publisher 节点已启动！")
      #除了 .info() 之外，主要还有以下四种类型，按严重性由低到高排列：.debug()；.info()；.warn()；.error()；.fatal()


    # ---------- 定时器回调函数 ----------
    # 这个函数会在上面创建的定时器触发时被调用。在这个例子中，它只会被执行一次。
    def timer_callback(self):
        # 创建一个 JointTrajectory 消息类型的空实例，准备填充数据。
        msg = JointTrajectory()

        # 设置消息中的 joint_names 字段。
        # 这是一个字符串列表，包含了轨迹中将要控制的所有关节的名称。
        # 这里的顺序非常重要，必须和 URDF 文件中定义的关节顺序以及后面 positions 列表中的数值一一对应。
        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # 创建一个 JointTrajectoryPoint 消息类型的实例。一条轨迹可以包含多个点，但这里我们只创建一个点。
        point = JointTrajectoryPoint()
        #基础轨迹：只有一个轨迹点
        {
        # # 设置这个轨迹点的目标位置（positions）。
        # # 这是一个浮点数列表，单位是弧度（radians）。
        # # 列表中的每个值分别对应上面 joint_names 列表中的一个关节。
        # # 例如，-1.0 对应 'shoulder_lift_joint'，1.0 对应 'elbow_joint'。
        # point.positions = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]
        
        # # 设置 time_from_start 字段。
        # # 这指定了从轨迹开始执行后，应该花费多长时间到达这个轨迹点。
        # # 这里设置为 2 秒，意味着机器人应该在 2 秒内从当前位置平滑运动到 point.positions 指定的目标位置。
        # point.time_from_start = Duration(sec=2)
      
        # # 将配置好的轨迹点（point）添加到轨迹消息（msg）的 points 列表中。
        # msg.points.append(point)
        }

        # 复杂轨迹：多个轨迹点，形成一个动作序列，但是都是逐个生成并添加到轨迹中
        {
         # 创建第一个轨迹点
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=2)  # 2秒到达第一个点

        # 创建第二个轨迹点
        point2 = JointTrajectoryPoint()
        point2.positions = [1.57, -1.0, 1.0, 0.0, 1.57, 0.0]  # 90度转动第一个关节
        point2.time_from_start = Duration(sec=4)  # 4秒到达第二个点

        # 创建第三个轨迹点
        point3 = JointTrajectoryPoint()
        point3.positions = [1.57, -0.5, 0.5, -0.5, 1.57, 0.0]  # 改变手臂姿态
        point3.time_from_start = Duration(sec=6)  # 6秒到达第三个点

        # 创建第四个轨迹点（回到初始位置）
        point4 = JointTrajectoryPoint()
        point4.positions = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]
        point4.time_from_start = Duration(sec=8)  # 8秒到达第四个点

        # 按顺序将所有点添加到轨迹中
        msg.points.append(point1)
        msg.points.append(point2)
        msg.points.append(point3)
        msg.points.append(point4)
        }

        # # 更复杂轨迹：多个轨迹点，形成一个动作序列，通过字典和循环简化代码
        # {
        # # 1. 用字典写“路点”——想加多少就加多少
        # waypoints = [
        #     {"angles": [0.0, -1.0, 1.0, 0.0, 0.0, 0.0], "t": 2},
        #     {"angles": [0.5, -1.2, 0.8, -0.5, 0.2, 0.1], "t": 4},
        #     {"angles": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "t": 6},
        #     {"angles": [-0.5, -0.5, 0.5, 0.5, -0.2, -0.1], "t": 8},
        # ]

        # # 2. 自动生成 JointTrajectoryPoint
        # for wp in waypoints:
        #     point = JointTrajectoryPoint()
        #     point.positions = wp["angles"]
        #     point.time_from_start = Duration(sec=wp["t"])
        #     msg.points.append(point)
        # }
        
        # 为整个轨迹消息（msg）的头部（header）设置时间戳。
        # 这通常是一个好习惯，表示这条消息是在什么时间创建的。
        # self.get_clock().now().to_msg() 获取当前 ROS 时间并转换为消息格式。
        msg.header.stamp = self.get_clock().now().to_msg()

        # 使用之前创建的发布者，将填充好的轨迹消息（msg）发布出去。
        self.publisher_.publish(msg)
        
        # 在控制台打印一条日志，确认消息已经发布。
        self.get_logger().info("已经发布轨迹点！")

        # 因为我们只想发布一次轨迹指令，所以在发布完成后，调用定时器的 cancel() 方法。
        # 这会停止定时器，防止它在 1 秒后再次触发 timer_callback 函数。
        self.timer.cancel()


# ---------- 主函数：程序的入口点 ----------
# 这是 Python 脚本执行时首先运行的函数。
def main(args=None):
    # 初始化 rclpy 库。这是每个 ROS 2 Python 程序必须做的第一件事。
    rclpy.init(args=args)

    # 创建 Ur3TrajectoryPublisher 类的实例，也就是创建我们的节点对象。
    node = Ur3TrajectoryPublisher()

    # 进入一个循环，使节点保持运行状态，以便它可以处理回调函数（如定时器）。
    # 这个函数会一直阻塞，直到程序被外部中断（例如，在终端按下 Ctrl+C）。
    rclpy.spin(node)

    # 当 rclpy.spin() 退出后（例如按下了 Ctrl+C），下面的代码会执行。
    # 销毁节点，释放它所占用的所有资源。
    node.destroy_node()
    
    # 关闭 rclpy 客户端库，清理所有 ROS 2 相关的资源。
    rclpy.shutdown()


# ---------- Python 脚本的标准入口检查 ----------
# 这是一个 Python 的惯用写法。
# 当这个文件作为主程序直接运行时，__name__ 的值是 '__main__'，if 条件成立，main() 函数被调用。
# 如果这个文件被其他脚本作为模块导入，__name__ 的值将是模块名，main() 就不会被执行。
if __name__ == '__main__':
    main()