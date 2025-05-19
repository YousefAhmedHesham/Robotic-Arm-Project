#!/usr/bin/env python3

import time

import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint

from geometry_msgs.msg import PoseStamped


def plan_and_execute(robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
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

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():

    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning components
    robot_py_node = MoveItPy(node_name="moveit_py")
    robotic_arm = robot_py_node.get_planning_component("arm_group")
    robotic_hand = robot_py_node.get_planning_component("hand_group")
    logger.info("MoveItPy instance created")

    # Move arm to position_2 before starting the open/close loop
    robotic_arm.set_start_state_to_current_state()
    robotic_arm.set_goal_state(configuration_name="position_2")
    plan_and_execute(robot_py_node, robotic_arm, logger, sleep_time=0.0)

    
    for _ in range(15):
        # Move arm to position_0
        robotic_arm.set_start_state_to_current_state()
        robotic_arm.set_goal_state(configuration_name="position_0")
        plan_and_execute(robot_py_node, robotic_arm, logger, sleep_time=0.0)
    for _ in range(15):
        # Move arm to position_1
        robotic_arm.set_start_state_to_current_state()
        robotic_arm.set_goal_state(configuration_name="position_1")
        plan_and_execute(robot_py_node, robotic_arm, logger, sleep_time=0.0)
    for _ in range(15):
        # Move arm to position_2
        robotic_arm.set_start_state_to_current_state()
        robotic_arm.set_goal_state(configuration_name="position_2")
        plan_and_execute(robot_py_node, robotic_arm, logger, sleep_time=0.0)
    for _ in range(15):
        # Move arm to position_3
        robotic_arm.set_start_state_to_current_state()
        robotic_arm.set_goal_state(configuration_name="position_3")
        plan_and_execute(robot_py_node, robotic_arm, logger, sleep_time=0.0)
    for _ in range(15):
        # Move arm to position_4
        robotic_arm.set_start_state_to_current_state()
        robotic_arm.set_goal_state(configuration_name="position_4")
        plan_and_execute(robot_py_node, robotic_arm, logger, sleep_time=0.0)
    for _ in range(7):
        # Open hand
        robotic_hand.set_start_state_to_current_state()
        robotic_hand.set_goal_state(configuration_name="open_hand")
        plan_and_execute(robot_py_node, robotic_hand, logger, sleep_time=1.0)
    for _ in range(7):
        # Close hand
        robotic_hand.set_start_state_to_current_state()
        robotic_hand.set_goal_state(configuration_name="close_hand")
        plan_and_execute(robot_py_node, robotic_hand, logger, sleep_time=1.0)


if __name__ == "__main__":
    main()