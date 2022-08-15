#!/usr/bin/env python 
import sys
import rospy
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
group = moveit_commander.MoveGroupCommander("manipulator", ns="ur5", robot_description="ur5/robot_description", wait_for_servers=100)

pose_goal1 = PoseStamped()
pose_goal1.header.frame_id = "ur5/base_link"
pose_goal1.pose.orientation.w = 1.0
pose_goal1.pose.position.x = 0.6
pose_goal1.pose.position.y = 0.6
pose_goal1.pose.position.z = 0.4

pose_goal2 = PoseStamped()
pose_goal2.header.frame_id = "ur5/base_link"
pose_goal2.pose.orientation.w = 1.0
pose_goal2.pose.position.x = 0.6
pose_goal2.pose.position.y = 0.4
pose_goal2.pose.position.z = 0.4

pose_goal3 = PoseStamped()
pose_goal3.header.frame_id = "ur5/base_link"
pose_goal3.pose.orientation.w = 1.0
pose_goal3.pose.position.x = 0.6
pose_goal3.pose.position.y = 0.2
pose_goal3.pose.position.z = 0.4

pose_goal4 = PoseStamped()
pose_goal4.header.frame_id = "ur5/base_link"
pose_goal4.pose.orientation.w = 1.0
pose_goal4.pose.position.x = 0.6
pose_goal4.pose.position.y = 0.0
pose_goal4.pose.position.z = 0.4

pose_goal5 = PoseStamped()
pose_goal5.header.frame_id = "ur5/base_link"
pose_goal5.pose.orientation.w = 1.0
pose_goal5.pose.position.x = 0.6
pose_goal5.pose.position.y = -0.2
pose_goal5.pose.position.z = 0.4

pose_goal6 = PoseStamped()
pose_goal6.header.frame_id = "ur5/base_link"
pose_goal6.pose.orientation.w = 1.0
pose_goal6.pose.position.x = 0.6
pose_goal6.pose.position.y = -0.4
pose_goal6.pose.position.z = 0.4

pose_goal7 = PoseStamped()
pose_goal7.header.frame_id = "ur5/base_link"
pose_goal7.pose.orientation.w = 1.0
pose_goal7.pose.position.x = 0.6
pose_goal7.pose.position.y = -0.6
pose_goal7.pose.position.z = 0.4

while True:
    wpose = group.get_current_pose().pose
    wpose_copy = deepcopy(wpose)
    wpose_copy.position.x+=0.2
    waypoints = [pose_goal1.pose, pose_goal2.pose, pose_goal3.pose, pose_goal4.pose, pose_goal5.pose, pose_goal6.pose, pose_goal7.pose]

    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

    group.execute(plan, wait=True)

# Calling `stop()` ensures that there is no residual movement
group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
group.clear_pose_targets()
