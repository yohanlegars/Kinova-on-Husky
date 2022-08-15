#! /usr/bin/env python
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
import numpy as np
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Empty

import lidar_prop_functions
from lidar_prop_functions import lidar_data_analysis


# Callback to activate the check_for_humans subroutine.
def callback_start(msg_string, args):
    lidar_analysis = args
    print("data are:", msg_string.data)
    
    if msg_string.data == 'starting_humans_check':
        print ("\nState machine says: start check for humans sub routine")
        analyze_point_cloud(lidar_analysis) # function that checks for humans

# Callback to deactivate the check_for_humans subroutine.
def callback_stop(msg_string, args):
    
    if msg_string.data == 'stop_check_for_humans':
        args.pub_stop.publish(Empty()) 
        
        # We want to set a variables that stops the process also outside this function.
        args.kill_program = True
        quit()
        

def callback_lidar(pc2_msg, args):
    
    # the input args contains the class to analyze the data
    lidar_analysis = args 
    
    # To properly analyze data we need to transform the data from PointCloud2 format, to a simple array.
    xyz_lidar = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg)
    
    '''Print to debug - TO BE removed'''
    # print("shape of xyz easy: ", xyz_lidar.shape)  
    
    # Removing from the pointcloud all the points that belongs to regions where we already know that 
    # there are no pedestrians.
    if lidar_analysis.something_to_remove == True:
        
        # First thing we transform the pointcloud in the map frame because there the measures are 
        # absolute. Also the center of the clusters will be define in that reference frame.
        HT_matrix = lidar_analysis.tf_transform("map", "/velodyne")
        xyz_lidar_in_map = lidar_analysis.transform_point_cloud(xyz_lidar, HT_matrix, False)
        # Cleaning the pointcloud according to the blacklisted areas.
        clean_xyz_lidar_in_map = lidar_analysis.remove_blacklisted_areas(np.array(xyz_lidar_in_map))
        
        # Transforming back to "lidar/velodyne" frame the pointcloud.
        HT_matrix = lidar_analysis.tf_transform("/velodyne", "map")
        clean_xyz_lidar = lidar_analysis.transform_point_cloud(clean_xyz_lidar_in_map, HT_matrix, False)
        clean_xyz_lidar = np.array(clean_xyz_lidar)   
           
    else:
        clean_xyz_lidar = xyz_lidar
    
    '''Print to debug - TO BE removed'''   
    # print("shape of xyz clean: ", clean_xyz_lidar.shape) 
    
    # If GUI says to stop the process, then we stop this function with quit() 
    if lidar_analysis.kill_program == True:
        quit()
    
    # Clustering the pointcloud received as input.
    group_of_clusters, number_of_clusters = lidar_analysis.clustering(clean_xyz_lidar)
    print("Number of clusters: ", number_of_clusters)
    
    # Filtering the pcl to keep only the points that look like a human.
    possible_pedestrians = lidar_analysis.cluster_with_dimensions_of_a_pedestrian(group_of_clusters)
    print("Possible pedestrians: ", len(possible_pedestrians))
    
    
    '''TESTING STUFF - TO BE removed'''
    # if lidar_analysis.first_time == True:
    #     prova_centri = [np.array([1, 1, 1]), np.array([2, 0, 3])]
    #     lidar_analysis.blacklist_areas(prova_centri)
    
    # fake_pcl = np.array([[10, 10, 10], [1, 1, 100], [12, 1, 4], [1, 1.1, 0.9], [2.1, 0.2, 3.2], [1, 100, 1], [100, 1, 1], [0.6, 1, 1.1], [12, 1, 4], [0.6, 1.4, 1.1]])
    # clean_fake_pcl = lidar_analysis.remove_blacklisted_areas(fake_pcl)
    # print("fake clean pointcloud is: ", clean_fake_pcl)
    '''END - TESTING STUFF'''
    
    
    # If a possible pedestrian is detected, then we can compute its 3D location in space.
    if len(possible_pedestrians) > 0:
        
        # The first thing to do when a possible human is on the workspace is to stop the base of the 
        # robot.
        print("Possible human detected!! The robot base will be stopped to check with the camera.")
        result_stop_robot, message_base = lidar_analysis.stop_robot()
        print(message_base)
        
        centers = lidar_analysis.center_of_clusters(possible_pedestrians)
        HT_matrix = lidar_analysis.tf_transform("/velodyne", "map")
        # 'centers_in_map_tf' is used to command the robotic arm that points the camera towards that 
        # direction.
        centers_in_map_tf = lidar_analysis.transform_point_cloud(centers, HT_matrix, False)
        
        for center in centers_in_map_tf:
        
            center_message = geometry_msgs.msg.PoseStamped()
            center_message.header.frame_id = "/map" # or whichever frame
            center_message.header.stamp = rospy.Time(0) # not needed for this case
            center_message.pose.position.x = center[0]
            center_message.pose.position.y = center[1]
            center_message.pose.position.z = center[2]

            # Once a region proposal is found with the LIDAR, we move the arm and the camera towards the 
            # proposal
            print("The service to align the camera with the Lidar proposal is being called.")
            result_move_arm = lidar_analysis.move_arm(center_message)
            if result_move_arm == False:
                rospy.logwarn("Something went wrong, the camera is not aligned with the lidar proposal. Please restart the robot.")
        
            # Once the arm is in position and the camera is pointing towards the region proposal of the 
            # lidar, we run the OpenCV package to check with the camera
            print("Checking the lidar proposal with the camera...\n")
            result_image_recognition, message_camera = lidar_analysis.camera_detection()
            print(message_camera)
            
            if result_image_recognition:
                pedestrian = True
                break
            else:
                pedestrian = False

        # If one or more possible_pedestrian are actual pedestrian we need to alert the operators. 
        if pedestrian == True:
            _ = lidar_analysis.wait_for_human_to_move()
        # Otherwise we blacklist that area of the pointcloud to avoid false positive repetition.
        else: 
            lidar_analysis.blacklist_areas(centers_in_map_tf)
            print("Blacklisting part of the point cloud to avoid false positive detections")
        
    # Note that the function  "plot_lidar_proposals" is quite slow; check suggestions where it is defined
    # to understand how to use it.
    plotting = False # Plotting in RViz the clusters that look like a pedestrian
    if plotting: 
        if len(possible_pedestrians) > 0:
            
            pcl_in_map_tf = lidar_analysis.transform_point_cloud(possible_pedestrians, HT_matrix, True) 
            
            possible_pedestrian_in_map_tf = pcl_in_map_tf
            lidar_analysis.plot_lidar_proposals(possible_pedestrian_in_map_tf)
        
        # This 'else' is useful only if we plot in RViz the pointcloud of the possible pedestrians.
        # Its scope is to ensure that when no pedestrian are detected the pointcloud on Rviz is updated 
        else:
            fake_possible_pedestrian = [np.array([0, 0, 0])]
            lidar_analysis.plot_lidar_proposals(fake_possible_pedestrian)
    
    # We subscribe to a topic where a message is published when check_for_humans subroutine needs to 
    # end.        
    rospy.Subscriber("/sm_messages/stop_humans_check", String, callback_stop, lidar_analysis, queue_size=5)
    
    # New line to improve the readability in the terminal window    
    print("\n")
    
    
def analyze_point_cloud(lidar_analysis):
     
    # /velodyne_points is the topic where the point cloud is collected.
    rospy.Subscriber("/velodyne_points", PointCloud2, callback_lidar, lidar_analysis, queue_size=5)
    rospy.spin()

if __name__ == '__main__':
    
    rospy.init_node('analyze_point_cloud', anonymous=False)
    
    # Initialization of the class to analyze the pointcloud data.
    lidar_analysis = lidar_data_analysis()
    
    # The code is activated only when the state machine gives the authorization publishing a message on a topic.
    print("\n Check for human sub-routine is waiting for the green light from the state machine. When ready it will start automatically.")
    sub = rospy.Subscriber("/sm_messages/start_humans_check", String, callback_start, lidar_analysis, queue_size=5)
    
    rospy.spin()
