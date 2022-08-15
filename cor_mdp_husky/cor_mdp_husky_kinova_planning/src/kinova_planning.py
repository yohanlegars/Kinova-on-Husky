#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import tf
import numpy as np
import geometry_msgs.msg
from std_srvs.srv import Trigger, TriggerResponse
from check_for_humans.srv import camera_goal, camera_goalResponse



def angle_from_vectors(a, b):
    angle = np.arccos(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))  # gives only positive output
    return angle


class MoveKinova(object):
    """ This class encapsulates everything needed to communicate with the Kinova manipulator.
        We use MoveIt group commanders interface.
        Other ROS nodes can use services to control the robot arm.
        The code provides 3 functionalities:
            1: moving the arm to a resting pose
            2: move the arm to a standing pose, looking at a certain point with the camera
            3: move the arm smoothly while tracking a certain point with the camera"""

    def __init__(self):
        super(MoveKinova, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('kinova_planning_node')
        rospy.Service('look_at_point', camera_goal, self.inspect_point) # can be called to rotate camera to point
        rospy.Service('track_point', camera_goal, self.track_point)  # can be called to track point with camera moving left to right
        rospy.Service('go_resting', Trigger, self.go_resting) # can be called to move arm to resting position.
        rospy.Service('go_standing', Trigger, self.go_standing)  # can be called to move arm to standing position.

        try:
            arm_group_name = "kinova_arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(
                                                rospy.get_namespace() + 'move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
            self.point_of_interest = None
            self.orientation_tolerance = 0.05
            self.position_tolerance = 0.1
            self.arm_group.set_goal_orientation_tolerance(self.orientation_tolerance)
            self.arm_group.set_goal_position_tolerance(self.position_tolerance)
            self.arm_group.set_num_planning_attempts(3)
            self.arm_group.set_planner_id("RRTConnect")

            reference_frame = "kinova_arm_base_link"
            self.reference_frame = reference_frame
            self.arm_group.set_pose_reference_frame(reference_frame)  # from initial /odom (static) to base of kinova arm

            self.tf_listener = tf.TransformListener()

            self.eef_offset = 0.1
            self.eef_reach = 0.4
            self.eef_height = 0.5

            self.camera_rotation = tf.transformations.quaternion_from_euler(-0.5 * np.pi, 0, -0.5 * np.pi)
            self.camera_rotation_pose = geometry_msgs.msg.Quaternion(
                *tf.transformations.quaternion_from_euler(-0.5 * np.pi, 0, -0.5 * np.pi))

            self.pose_left = None
            self.pose_right = None

            self.resting_pose = geometry_msgs.msg.PoseStamped()
            self.resting_pose.header.frame_id = self.reference_frame
            self.resting_pose.header.stamp = rospy.Time(0)
            self.resting_pose.pose.position.x = 0
            self.resting_pose.pose.position.y = -0.1
            self.resting_pose.pose.position.z = 0.25
            self.resting_pose.pose.orientation = self.camera_rotation_pose

            self.standing_pose = geometry_msgs.msg.PoseStamped()
            self.standing_pose.header.frame_id = self.reference_frame
            self.standing_pose.header.stamp = rospy.Time(0)
            self.standing_pose.pose.position.x = 0
            self.standing_pose.pose.position.y = 0
            self.standing_pose.pose.position.z = 0.8
            self.standing_pose.pose.orientation = self.camera_rotation_pose

        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True
            rospy.loginfo("Initialization successful")

    def set_point_of_interest(self, point_of_interest):
        source_frame = point_of_interest.header.frame_id
        time = point_of_interest.header.stamp

        if source_frame is not None:
            pass
        else:
            rospy.logerr("No source frame specified in PoseStamped message")
            raise Exception("No source frame specified in PoseStamped message")

        if time is not None:
            pass
        else:   # if the pose msg is not time stamped
            time = rospy.Time(0)

        if source_frame == "map":   # if pose is in the correct static frame
            self.point_of_interest = point_of_interest
        else:   # transform from source frame to target reference frame
            try:
                self.tf_listener.waitForTransform(source_frame, "map", time, rospy.Duration(4.0))
                point_of_interest = self.tf_listener.transformPose("map", point_of_interest)
                self.point_of_interest = point_of_interest
            except Exception as e:
                rospy.logerr("could not transform point of interest source frame to target frame")
                rospy.logerr(e)

    def get_point_of_interest_armframe(self):
        """ Used to get the point of interest in the kinova arm frame.
            Since arm frame is not static, this location is not constant over time.
            This function is used to get the latest available transform"""
        source_frame = self.point_of_interest.header.frame_id
        time = rospy.Time(0)

        try:
            self.tf_listener.waitForTransform(source_frame, self.reference_frame, time, rospy.Duration(4.0))
            poi_armframe = self.tf_listener.transformPose(self.reference_frame, self.point_of_interest)
        except Exception as e:
            rospy.logerr("could not transform point of interest source frame to target frame")
            rospy.logerr(e)

        return poi_armframe

    def compute_z_angle(self, x, y):
        z_angle = angle_from_vectors([1, 0], [x, y])
        if y < 0:  # if y-component is negative, we have a negative rotation around Z
            z_angle *= -1
        return z_angle


    def compute_eef_positions(self, offset, reach, height):
        """Find the end effector position from 2 poses.
        The arm will move between these 2 positions while monitoring the point of interest.
        The 2 positions are both on a line normal to the vector between the POI and the Kinova base: one to the left
        of it, and one to the right.
        """

        time = rospy.Time(0)

        pose_left_armframe = geometry_msgs.msg.PoseStamped()
        pose_left_armframe.header.frame_id = self.reference_frame
        pose_left_armframe.header.stamp = time

        pose_right_armframe = geometry_msgs.msg.PoseStamped()
        pose_right_armframe.header.frame_id = self.reference_frame
        pose_right_armframe.header.stamp = time

        poi_armframe = self.get_point_of_interest_armframe()
        z_angle = self.compute_z_angle(poi_armframe.pose.position.x, poi_armframe.pose.position.y)

        def rotate_point_2d(point, angle): # checked, this works
            R = [[np.cos(angle), -1 * np.sin(angle)],
                 [np.sin(angle), np.cos(angle)]]

            rotated_point = np.matmul(R, point)
            return rotated_point

        point_left = [0, reach]
        point_right = [0, -reach]

        point_left = rotate_point_2d(point_left, z_angle)
        point_right = rotate_point_2d(point_right, z_angle)

        pose_left_armframe.pose.position.x = point_left[0] + offset
        pose_left_armframe.pose.position.y = point_left[1]
        pose_left_armframe.pose.position.z = height
        pose_right_armframe.pose.position.x = point_right[0] + offset
        pose_right_armframe.pose.position.y = point_right[1]
        pose_right_armframe.pose.position.z = height

        return pose_left_armframe, pose_right_armframe

    def compute_eef_orientation(self, arm_target_pose_armframe):
        """"compute the transformation from the current kinova end effector orientation
        to the orientation where it is looking right at the point of interest"""

        ur5_pos_armframe = self.get_point_of_interest_armframe()

        d_pos = [ur5_pos_armframe.pose.position.x - arm_target_pose_armframe.pose.position.x,
                 ur5_pos_armframe.pose.position.y - arm_target_pose_armframe.pose.position.y,
                 ur5_pos_armframe.pose.position.z - arm_target_pose_armframe.pose.position.z,
                 1]


        # we first find the rotation around the Z-axis, in the XY plane.
        z_angle = self.compute_z_angle(*d_pos[:2])

        rot_z = [[np.cos(z_angle), -1*np.sin(z_angle), 0, 0],
                 [np.sin(z_angle), np.cos(z_angle), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]

        q_eef = tf.transformations.quaternion_from_matrix(rot_z)
        q_camera = tf.transformations.quaternion_multiply(q_eef, self.camera_rotation)

        arm_target_pose_armframe.pose.orientation.w = q_camera[3]
        arm_target_pose_armframe.pose.orientation.x = q_camera[0]
        arm_target_pose_armframe.pose.orientation.y = q_camera[1]
        arm_target_pose_armframe.pose.orientation.z = q_camera[2]

        return arm_target_pose_armframe

    def compute_set_eef_poses(self):
        # first compute the positions for the poses
        pose_left_armframe, pose_right_armframe = self.compute_eef_positions(self.eef_offset,
                                                                             self.eef_reach,
                                                                             self.eef_height)

        # now compute the orientations that make the camera aligned at each position
        pose_left_armframe = self.compute_eef_orientation(pose_left_armframe)
        pose_right_armframe = self.compute_eef_orientation(pose_right_armframe)

        # saving the left and right pose as class members
        self.pose_left = pose_left_armframe
        self.pose_right = pose_right_armframe


    def move_arm_eef_target(self, pose, end_effector_link = ""):
        """ move the arm to get the EEF to a desired pose
            because the controller fails sometimes, try execution again if it fails
            up until n_tries, after that, log an error and stop trying.
            input: desired EEF PoseStamped
            output: True if Pose is reached, false if it is not"""
        result = False
        n_tries = 0

        while not result and n_tries < 3:
            self.arm_group.set_pose_target(pose)
            result = self.arm_group.go()
            n_tries += 1
            if result is not True and n_tries < 3:
                rospy.logwarn("Execution failed, trying again")
            elif result is not True and n_tries >= 3:
                rospy.logerr("Execution failed, stop trying")
            else:
                rospy.loginfo("Configuration reached!")
        return result

    def move_arm_cartesian(self, pose1_in, pose2_in):
        """ move the arm between two poses in a cartesian (straight) path
            used to move the arm EEF between two poses while keeping the EEF orientation smooth along the path,
            to keep the camera field of view focussed on the point of interest that we are tracking
            input: Two PoseStamped messages
            output: True if execution is successful, False if it is not
            """
        pose1 = pose1_in.pose
        pose2 = pose2_in.pose

        (plan, fraction) = self.arm_group.compute_cartesian_path(
                            [pose1, pose2],  # waypoints to follow
                            0.01,  # eef_step
                            0.0)  # jump_threshold

        status = self.arm_group.execute(plan, wait=True)
        self.arm_group.stop()
        return status

    def track_point(self, req):
        self.set_point_of_interest(req.Location)
        self.compute_set_eef_poses()

        rospy.loginfo("First go from initial to pose left")
        result = self.move_arm_eef_target(self.pose_left)
        rospy.loginfo("Now tracking while moving right")
        result = self.move_arm_cartesian(self.pose_left, self.pose_left)
        rospy.loginfo("Now tracking while moving left")
        result = self.move_arm_cartesian(self.pose_left, self.pose_left)

        return camera_goalResponse(result)

    def inspect_point(self, req):

        inspection_pose = self.standing_pose # start with the standing pose, not correct orientation yet

        self.set_point_of_interest(req.Location)
        poi = self.get_point_of_interest_armframe()

        z_angle = self.compute_z_angle(poi.pose.position.x, poi.pose.position.y)

        rot_z = [[np.cos(z_angle), -1 * np.sin(z_angle), 0, 0],
                 [np.sin(z_angle), np.cos(z_angle), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]

        q_eef = tf.transformations.quaternion_from_matrix(rot_z)
        q_camera = tf.transformations.quaternion_multiply(q_eef, self.camera_rotation)
        inspection_pose.pose.orientation.w = q_camera[3]
        inspection_pose.pose.orientation.x = q_camera[0]
        inspection_pose.pose.orientation.y = q_camera[1]
        inspection_pose.pose.orientation.z = q_camera[2]

        self.arm_group.set_goal_position_tolerance(0.3)     # set orientation tolerance wider, we don't really care
        rospy.loginfo("Move to look at specified point")
        result = self.move_arm_eef_target(inspection_pose)
        self.arm_group.set_goal_position_tolerance(self.position_tolerance)  # set it back

        if result:
            return camera_goalResponse(True)
        else:
            return camera_goalResponse(False)

    def go_resting(self, request):
        """ callback function for a service to make the kinova arm retract into resting position
            used when we want the arm within the footprint of Husky, for safety, before starting to drive"""
        rospy.loginfo("Moving to resting configuration")
        result = self.move_arm_eef_target(self.resting_pose)

        if result:
            return TriggerResponse(True, "")
        else:
            return TriggerResponse(False, "")

    def go_standing(self, request):
        """ callback function for a service to make the kinova arm stand up straight.
            used for debugging purposes of services."""
        rospy.loginfo("Moving to Standing configuration")
        result = self.move_arm_eef_target(self.standing_pose)

        if result:
            return TriggerResponse(True, "")
        else:
            return TriggerResponse(False, "")


def main():

    kinova = MoveKinova() # create an instance of custom Kinova Class
    kinova.go_resting("") # use service to put the kinova arm in an initial resting pose
    rospy.loginfo("Ready to receive commands")

    rospy.spin() # wait for service calls




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
