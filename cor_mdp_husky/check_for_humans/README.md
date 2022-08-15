# Information about the package *Check for Humans*

## Table of Contents  
- [1. Goal of the package](#1-goal-of-the-package)
- [2. Description of the implementation](#2-description-of-the-implementation)
- [3. Installation process <a name="about"></a>](#3-installation-process)
- [4. Extra information](#4-extra-information)


## 1. Goal of the package
The ultimate goal of this package is to detect humans in the environment during the initial exploration of the robot.
The humans' recognition relies on two main features:
- Detection of possible pedestrians with the LiDAR
- Check over the LiDAR prediction with an OpenCV library for pedestrian recognition from a video stream.

In this package, only the first point is implemented. Indeed, the analysis of the images is handled in another package: [`camera_detector`](../camera_detector). But, once the LiDAR detects a possible pedestrian, through the node implemented in this package we call a *service* that interacts with the [`camera_detector`](../camera_detector) package.

The usage of the LiDAR for proposing possible locations where a human might be is particularly efficient since the clustering algorithm is pretty fast and the accuracy is quite high. Moreover, the LiDAR always performs 360-degree scans, which allows us to detect possible people all around the robot without moving continuously the camera or the base of the robot.

## 2. Description of the implementation
The code relies on a *class* called `lidar_data_analysis` (in the file [`lidar_prop_functions.py`](src/lidar_prop_functions.py)). The sequence of actions taken to analyze the point cloud are the following:
- Collect the point cloud from the topic `/velodyne_points`.
- Remove from the point cloud the points that belong to areas where we have already verified that there are no humans.
- Clustering the point cloud
- Identifying which cluster might be a person
- If a possible person is found:
    - Use a *service* to stop the motion of the robot base
    - Use a *service* to move the camera mounted on the robotic arm toward the possible person
    - Run the OpenCV package to recognize people in the image frame shot by the camera on the arm:
        - if a person is detected &rarr; alert the operator through the GUI.
        - if the LiDAR proposal was a false positive &rarr; blacklist that area of the workspace to avoid other false detection.
- Keep iterating the steps above until all the environment has been explored.        

## 3. Installation process

**N.B These steps are also listed in the main README of the repository. You do not need to execute them if you have already done them once.**

The main package needed to run this code is the algorithm for clustering (DBSCAN). To install it you need to have `pip`. If you do NOT have it, then follow the next three steps:
- Step 1:
    ``` bash
    sudo apt-get update
    ```
- Step 2:
    ``` bash
    sudo apt-get install python-pip python-dev build-essential
    ```
- Step 3:
    ``` bash
    sudo python -m easy_install pip
    ```

Now that you have `pip` you can install the actual library for the clustering algorithm:
``` bash
pip install -U scikit-learn
```

One more package you might need if not already installed `rospkg`. To install it run:
``` bash
sudo apt-get install python-rospkg
```

There is one last package you need to remove a warning from the code execution:
``` bash
sudo apt install libcanberra-gtk-module libcanberra-gtk3-module
```



## 4. Extra information

Note that the current implementation of the package would not work in an environment where there are dynamic obstacles other than humans. This is due to the function `lidar_analysis.blacklist_areas(self, pointcloud)` (in the file [`lidar_prop_functions.py`](src/lidar_prop_functions.py)). This function removes from the detected point cloud certain areas where the LiDAR proposal, previously,  had a false positive. But if the false positive was a dynamic obstacle, then the blacklisted area should be dynamic too. 