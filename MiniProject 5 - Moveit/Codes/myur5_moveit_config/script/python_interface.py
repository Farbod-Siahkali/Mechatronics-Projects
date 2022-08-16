#!/usr/bin/python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
from math import tau, pi

if __name__ == '__main__':
	
    # initialize moveit_commander and ros node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface", anonymous=True)
    # define robot
    robot = moveit_commander.RobotCommander()
    # define scene
    scene = moveit_commander.PlanningSceneInterface()
    # define move_group
    group_name = 'arm'
    move_group = moveit_commander.MoveGroupCommander(group_name)

    planning_frame = move_group.get_planning_frame()
    print(f'============ Planning frame : {planning_frame}')
    group_names = robot.get_group_names()
    print(f'============ Group Names : {group_names}')
    print("============ Robot's Current State ==========")
    print(robot.get_current_state())

    # Starting position
    print("Go to strating point.")
    joint_goal = move_group.get_current_joint_values()

    joint_goal[0] = -pi / 4
    joint_goal[1] = -pi / 2
    joint_goal[2] =  pi / 2
    joint_goal[3] = -pi / 2
    joint_goal[4] = -pi / 2
    joint_goal[5] = 0

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # carthesian path
    #R
    print("Planning R\n")
    waypoints = []
    scale = 1.0
    wpose = move_group.get_current_pose().pose
    wpose.position.z += scale * 0.2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += scale * 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z -= scale * 0.1
    waypoints.append(copy.deepcopy(wpose))
    
    wpose.position.y -= scale * 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z -= scale * 0.1
    wpose.position.y += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))

    ##come back to start pose
    wpose.position.y += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))

    #O
    print("Planning O\n")
    wpose.position.z += scale * 0.2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += scale * 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z -= scale * 0.2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1
    waypoints.append(copy.deepcopy(wpose))

    #S
    print("Planning S\n")
    wpose.position.y += scale * 0.2
    wpose.position.z += scale * 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z -= scale * 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += scale * 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += scale * 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += scale * 0.1
    wpose.position.y -= scale * 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z += scale * 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += scale * 0.05
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z -= scale * 0.05
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
    waypoints, # waypoints to follow
    0.01, # eef_step
    0.0) # jump_threshold
    move_group.execute(plan, wait=True)
