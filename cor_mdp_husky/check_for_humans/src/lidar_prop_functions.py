from sklearn.cluster import DBSCAN
import numpy as np
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import rospy
import tf
from std_srvs.srv import Trigger
from std_msgs.msg import Empty
from check_for_humans.srv import camera_goal


class lidar_data_analysis:
    
    def __init__(self):
        self.pcl_pub = rospy.Publisher("/lidar_proposals", PointCloud2, queue_size=1)
        self.listener = tf.TransformListener()
        self.tf = tf.TransformerROS()
        
        # The following three variables are used to blacklist part of the world where we have already 
        # checked and there are no humans.
        self.x_boundaries = [] # x for the width
        self.y_boundaries = [] # y for the depth
        self.z_boundaries = [] # z for the height
        
        self.first_time = True
        # This variable is changed to true as soon as the first area is blacklisted.
        self.something_to_remove = False
        
        # Once the exploration of the environment is done, this node needs to be stop. To do so we wait
        # for a message published by the gui. When this happens, we publish another message to 
        # communicate with the state machine that changes state.
        self.pub_stop = rospy.Publisher('/sm_messages/humans_check_done', Empty, queue_size=10)
        self.kill_program = False


    # In the machine learning course we studied 3 algorithm to cluster: K-means, Gaussian Mixture Model and DBSCAN.
    # The first one the required as hyperparameter the number of cluster we want to end up with, information that
    # currently we do not know, therefore is reasonable to try DBSCAN.
    def clustering(self, lidar_points):
        
        cluster_algorithm = DBSCAN(eps=0.28, min_samples=15).fit(lidar_points) # hyperparameters found with trial and error
        labels = cluster_algorithm.labels_ # for each point in lidar_points we have a label that assigns that point 
        # to one of the clusters
        
        # To find the number of clusters identified by the algorithm I can simply count the different labels present
        number_of_clusters = len(np.unique(labels))
        group_of_clusters = []
        for i in range(number_of_clusters):
            group_of_clusters.append(lidar_points[labels == (i-1)])

        return group_of_clusters, number_of_clusters

    # Thanks to the following function, it is possible to filter the clusters, and keep only the ones
    # that are shaped as a human
    def cluster_with_dimensions_of_a_pedestrian(self, lidar_clusters):
            
            # Final list containg only the clusters similar to pedestrians
            lidar_pedestrian_like = []
            
            # I have to find the xyz dimension of each cluster
            x_min = [] # x for the width
            x_max = []
            y_min = [] # y for the depth 
            y_max = []
            z_min = [] # z for the height
            z_max = []
            for cluster in lidar_clusters:
                x_min.append(np.min(cluster[:, 0]))
                x_max.append(np.max(cluster[:, 0]))
                y_min.append(np.min(cluster[:, 1]))
                y_max.append(np.max(cluster[:, 1]))
                z_min.append(np.min(cluster[:, 2]))
                z_max.append(np.max(cluster[:, 2]))
            
            # We have suppose some reasonable minimum and maximum dimension for a pedestrian cluster.
            # width -> between 0.2 and 1.2 (meters) [x_max - x_min]
            # depth -> between 0.2 and 1.2 [y_max - y_min]
            # height -> between 1.1 and 2.2 [z_max - z_min]
            
            widths = np.subtract(x_max, x_min)
            depths = np.subtract(y_max, y_min)
            heights = np.subtract(z_max, z_min)
            
            # Uncomment the following to visualize the dimensions of the clusters in the terminal
            # print("along the x direction we have:", widths)
            # print("along the y direction we have:", depths)
            # print("along the z direction we have:", heights)
            
            # Filtering by width, height and depth
            i = 0
            for (width, height, depth) in zip(widths, heights, depths):
                # according to the position of the pedestrian with reference to the lidar, the pointcloud
                # might be flat with respect to one of the two dimension (x or y). But the other has to be 
                # between 0.2 and 1.2.
                if (width < 1.1) and (depth < 1.1): 
                    if (width > 0.2) or (depth > 0.2): 
                        if 1.0 < height < 2.2: # The height is always a requirement
                            lidar_pedestrian_like.append(lidar_clusters[i])
                i += 1
                            
            return lidar_pedestrian_like
    
    # This function publishes on RViz the pointcloud given as input. Unlikely is quite slow. We suggest
    # to use it only in static situation (the environment does not change, and the robot is still).  
    def plot_lidar_proposals(self, lidar_pedestrian_like):
        
        # give time to roscore to make the connections
        rospy.sleep(1.)
        
        cloud_points = lidar_pedestrian_like
        
        # header for the message to be published
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map' 
        
        # create pcl from points
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
        
        # publish    
        rospy.loginfo("happily publishing lidar proposal pointcloud!")
        self.pcl_pub.publish(scaled_polygon_pcl)
    
    # Given two frames, it computes the homogeneous transformation matrix to go from one to the other.    
    def tf_transform(self, from_frame, to_frame):
        rate = rospy.Rate(10.0)
        try:
            (translation, rotation) = self.listener.lookupTransform(to_frame, from_frame, rospy.Time(0))
            
            # from the translation and rotation we can compute the homogenous transformation matrix to
            # transform the lidar point cloud from the reference frame /velodyne to /map.
            HT_matrix = self.tf.fromTranslationRotation(translation, rotation)
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            HT_matrix = np.eye(4) # just an identity matrix to avoid problem with other functions that 
            # requires HT_matrix to be defined.
            rospy.loginfo("exemption in the transformation calculation")
            pass
        
        return HT_matrix
    
    # function to change the reference frame with respect to which the point cloud is expressed
    def transform_point_cloud(self, point_cloud, HT_matrix, pcl_is_a_list):
        
        # the input point_cloud might be a list of 3D array, where each array contains a series of 3D coordinates
        # that identify the points of the specific cluster. To transform the point_cloud we need to 
        # work with arrays. If this is the case, set pcl_is_a_list = True.
        if pcl_is_a_list:
            point_cloud = np.concatenate(point_cloud, axis=0)
            point_cloud = point_cloud.reshape(-1, 3)
        
        # adding a column for the multiplication with the HT_matrix
        point_cloud_4d = np.c_[point_cloud, np.ones(point_cloud.shape[0])]
    
        # The new point_cloud is computed, and transformed in a list 
        new_point_cloud_4d = (np.matmul(HT_matrix, point_cloud_4d.T)).T
        new_point_cloud = new_point_cloud_4d[:, :-1].tolist()
        
        return new_point_cloud
    
    # Given one or more clusters in pointcloud format (each cluster has to be a individual array), 
    # the following function returns a unique 3D coordinated point that identifies the position of 
    # the center of the cluster.
    # In our specific application, we will use this information to point the camera mounted on the 
    # manipulator. 
    def center_of_clusters(self, clusters):
        
        x_min = [] # x for the width
        x_max = []
        y_min = [] # y for the depth
        y_max = []
        z_min = [] # z for the height 
        z_max = []
        for cluster in clusters:
            x_min.append(np.min(cluster[:, 0]))
            x_max.append(np.max(cluster[:, 0]))
            y_min.append(np.min(cluster[:, 1]))
            y_max.append(np.max(cluster[:, 1]))
            z_min.append(np.min(cluster[:, 2]))
            z_max.append(np.max(cluster[:, 2]))
        
        # Calculating the center x,y, and z of the clusters. Lists are also transformed into arrays.
        center_widths = np.add(x_max, x_min)/2
        center_depths = np.add(y_max, y_min)/2
        center_heights = np.add(z_max, z_min)/2    
        
        # Reorganization in a unique array of arrays, where each array is an xyz coordinate.
        centers = (np.stack((center_widths, center_depths, center_heights))).T
            
        return centers
    
    # If a cluster that look like a pedestrian is not a pedestrian, we need to neglect that space of the 
    # map from future human detections.
    # Given some points in space, this function computes a bounding box (with the dimension similar to 
    # a human). All the points of the future clusters that belong to these bounding boxes will be deleted
    # from the pointcloud.
    def blacklist_areas(self, centers):
        
        # The first time the variable are initialized as lists, so this 'if' is not necessary.
        if self.first_time == False:
            # Transforming the arrays into lists, where appending element is easier
            self.x_boundaries = self.x_boundaries.flatten()
            self.y_boundaries = self.y_boundaries.flatten()
            self.z_boundaries = self.z_boundaries.flatten()
            self.x_boundaries = self.x_boundaries.tolist()
            self.y_boundaries = self.y_boundaries.tolist()
            self.z_boundaries = self.z_boundaries.tolist()
        else:
            self.first_time = False
            self.something_to_remove = True
        
        # The bounding boxes will have a dimension of 1x1x2 meters (xyz).
        for center in centers:
            self.x_boundaries.append(center[0] - 0.5)
            self.x_boundaries.append(center[0] + 0.5)
            self.y_boundaries.append(center[1] - 0.5)
            self.y_boundaries.append(center[1] + 0.5)
            self.z_boundaries.append(center[2] - 1)
            self.z_boundaries.append(center[2] + 1)
            
        
        # Now I organize the boundaries of the bounding boxes as arrays of couples. Each couple 
        # identifies the extremis of x, or y, or z for one bounding box.  
        self.x_boundaries = np.array(self.x_boundaries).reshape(-1, 2)
        self.y_boundaries = np.array(self.y_boundaries).reshape(-1, 2)
        self.z_boundaries = np.array(self.z_boundaries).reshape(-1, 2)
        
    def remove_blacklisted_areas(self, pointcloud):
        # pointcloud has to be an array nx3
        # self.x_boundaries, self.y_boundaries, and self.z_boundaries are arrays nx2, where each couple 
        # delimits one dimension of a bounding box.
        
        j = 0
        # We check each point of the pointcloud
        for point in pointcloud:
            
            # For each point of the pointcloud we check each bounding box
            for i in range(self.x_boundaries.shape[0]):
                if self.x_boundaries[i, 0] < point[0] < self.x_boundaries[i, 1]:
                    if self.y_boundaries[i, 0] < point[1] < self.y_boundaries[i, 1]:
                        if self.z_boundaries[i, 0] < point[2] < self.z_boundaries[i, 1]:
                            
                          pointcloud = np.delete(pointcloud, j, axis=0)
                          # reducing the index j, because now 'pointcloud' has one row less.
                          j -= 1
                          # If we determine that the point has to be deleted, i.e. it belongs to a 
                          # blacklisted area, then we do not check if it belongs to the other boxes. 
                          # We do not care about it.
                          break
            j += 1        
        
        clean_pointcloud = pointcloud.reshape(-1, 3)
        
        return clean_pointcloud          
    
    # This function is a service client to cancel the current goal of the robot base. In this way we can
    # ensure that the robot will be stopped.
    def stop_robot(self):
        
        rospy.wait_for_service('cancel_goal')
        try:
            cancel_goal = rospy.ServiceProxy('cancel_goal', Trigger)
            stopped_robot = cancel_goal()
            
            return stopped_robot.success, stopped_robot.message
        
        except rospy.ServiceException as e:
            print("Service cancel_goal call failed: %s"%e)
            
    # This function is a service client to move the arm, hence the camera, to a location proposed by the
    # LIDAR.
    def move_arm(self, Location):
        
        rospy.wait_for_service('look_at_point')
        try:
            move_arm = rospy.ServiceProxy('look_at_point', camera_goal)
            arm_moved = move_arm(Location)
            
            return arm_moved.success
        
        except rospy.ServiceException as e:
            print("Service look_at_point call failed: %s"%e)
        
    
    # This function is a service client to cancel the current goal of the robot base. In this way we can
    # ensure that the robot will be stopped.
    def camera_detection(self):
        
        rospy.wait_for_service('/visual_human_detection/look_for_humans')
        try:
            camera_detection = rospy.ServiceProxy('/visual_human_detection/look_for_humans', Trigger)
            detection = camera_detection()
            
            return detection.success, detection.message
        
        except rospy.ServiceException as e:
            print("Service '/visual_human_detection/look_for_humans' call failed: %s"%e)    
            
    # This function is a service client to advise that a human is present on field. 
    def wait_for_human_to_move(self):
        
        rospy.wait_for_service('human_detected')
        try:
            wait_for_human_to_move = rospy.ServiceProxy('human_detected', Trigger)
            free_from_humans = wait_for_human_to_move()
            
            return free_from_humans.success, free_from_humans.message
        
        except rospy.ServiceException as e:
            print("Service human_detected call failed: %s"%e)
        
