#!/usr/bin/env python3
"""
A script to demonstrate MoveItPy motion planning API for UR3 robot.
Adapted from moveit2_tutorials/doc/examples/motion_planning_python_api/scripts/motion_planning_python_api_tutorial.py
"""

import time
import rclpy
from rclpy.logging import get_logger

# MoveItPy imports
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

# ROS messages
from geometry_msgs.msg import PoseStamped


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    logger.info("Planning trajectory...")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    if plan_result and plan_result.success:
        logger.info("Executing plan...")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed!")

    time.sleep(sleep_time)


def main():
    ###################################################################
    # MoveItPy Setup for UR3
    ###################################################################
    rclpy.init()
    logger = get_logger("ur3_moveit_py_demo")

    # Create MoveItPy instance
    ur3_robot = MoveItPy(node_name="ur3_moveit_py")
    ur3_arm = ur3_robot.get_planning_component("ur_manipulator")  # UR 默认规划组名
    logger.info("MoveItPy instance for UR3 created")

    ###########################################################################
    # Plan 1 - Move to a safe "home" joint configuration (if defined in SRDF)
    ###########################################################################
    try:
        ur3_arm.set_start_state_to_current_state()
        ur3_arm.set_goal_state(configuration_name="home")  # 可选：确保你的 SRDF 中有 <group_state name="home" ...>
        plan_and_execute(ur3_robot, ur3_arm, logger, sleep_time=2.0)
    except Exception as e:
        logger.warn(f"'home' state not found or failed: {e}. Skipping Plan 1.")

    ###########################################################################
    # Plan 2 - Set goal with PoseStamped (Cartesian goal)
    ###########################################################################
    ur3_arm.set_start_state_to_current_state()

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"  # UR3 的 base 坐标系
    pose_goal.header.stamp = ur3_robot.get_node().get_clock().now().to_msg()
    pose_goal.pose.position.x = 0.3
    pose_goal.pose.position.y = 0.0
    pose_goal.pose.position.z = 0.4
    pose_goal.pose.orientation.w = 1.0  # 无旋转（identity quaternion）

    # Set goal using tool0 (standard UR end-effector link)
    ur3_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool0")

    plan_and_execute(ur3_robot, ur3_arm, logger, sleep_time=2.0)

    ###########################################################################
    # Plan 3 - Move to another Cartesian pose
    ###########################################################################
    ur3_arm.set_start_state_to_current_state()

    pose_goal2 = PoseStamped()
    pose_goal2.header.frame_id = "base_link"
    pose_goal2.header.stamp = ur3_robot.get_node().get_clock().now().to_msg()
    pose_goal2.pose.position.x = 0.2
    pose_goal2.pose.position.y = -0.2
    pose_goal2.pose.position.z = 0.3
    pose_goal2.pose.orientation.w = 1.0

    ur3_arm.set_goal_state(pose_stamped_msg=pose_goal2, pose_link="tool0")
    plan_and_execute(ur3_robot, ur3_arm, logger, sleep_time=2.0)

    ###########################################################################
    # Plan 4 - Joint space goal via RobotState
    ###########################################################################
    robot_model = ur3_robot.get_robot_model()
    robot_state = RobotState(robot_model)

    # Define a safe joint configuration for UR3 (radians)
    joint_values = {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -1.57,
        "elbow_joint": 1.57,
        "wrist_1_joint": -1.57,
        "wrist_2_joint": -1.57,
        "wrist_3_joint": 0.0,
    }

    robot_state.set_joint_group_positions("ur_manipulator", list(joint_values.values()))
    ur3_arm.set_start_state_to_current_state()
    ur3_arm.set_goal_state(robot_state=robot_state)

    plan_and_execute(ur3_robot, ur3_arm, logger, sleep_time=2.0)

    logger.info("UR3 MoveItPy demo completed.")


if __name__ == "__main__":
    main()