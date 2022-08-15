# The camera_detector package

This package consists in the implementation of a simple object detection module for identifying humans within a camera video feed. The aim of this package is to embed this functionality within a node, which can provide its detections to other nodes in a ros program, as a service.

## File Structure

The package is organised in the following manner:

```
camera_detector
 │
 ├─── CMakeLists.txt
 │
 ├─── package.xml
 │
 ├─── README.md
 │
 │
 ├─── include
 │     └─── camera_detector
 │           └─── camera_detector.h
 │
 ├─── launch
 │     └─── camera_detector.launch
 │
 └─── src
       ├─── camera_detector.cpp
       └─── camera_detector_main.cpp
```

## Package Summary

The node is implemented as a class, ```CamDetector```, which subscribes to an image topic, and continuously performs detection of humans using the topic's messages as inputs.

The class ```CamDetector``` is declared in ```camera_detector.h```, and implemented in ```camera_detector.cpp```. ```camera_detector_main.cpp``` encapsulates the class into a continuously running ros node.

the launch file ```camera_detector.launch``` can be used to add our ready-to-go camera detection module within any other ROS setup.

## Building the code

Before the code can be built, some packages must be installed as prerequisites. Those are [```vision_msgs```](http://wiki.ros.org/vision_msgs) and [```vision_opencv```](http://wiki.ros.org/action/show/vision_opencv).

You can verify that you have those packages installed correctly on your computer using the following commands:
```
:<any_directory>$ apt list ros-melodic-vision-msgs
:<any_directory>$ apt list ros-melodic-vision-opencv
```

If any (or both) of these packages are missing from your current installations, you an simply install them using these commands:
```
:<any_directory>$ sudo apt update
:<any_directory>$ sudo apt install ros-melodic-vision-msgs
:<any_directory>$ sudo apt install ros-melodic-vision-opencv
```

Then you can build the package using this command.
```
:<your_catkin_ws_path>$ catkin build camera_detector
```
That's it, you're ready to use the code as you see fit now!

## Running the code

You can choose to run the ```camera_detector``` directly, or by using the provided launch file. Feel free to include this launch file in any other project.

For running the node directly:
```
:<your_catkin_ws_path>$ source devel/setup.bash
:<your_catkin_ws_path>$ rosrun camera_detector camera_detector_node
```

Using the launch file:
```
:<your_catkin_ws_path>$ source devel/setup.bash
:<your_catkin_ws_path>$ roslaunch camera_detector camera_detector.launch
```

## Interfacing with the camera object detector node

While the node is running, other nodes/agents can call its service to obtain information about the detection. You can run this command manually:
```
:<any_directory>$ rosservice call /visual_human_detection/look_for_humans
```
Note that it is assumed here that the node is within the ```/visual_human_detection``` namespace. Leave this part out or adapt it to your particular setup if you decide to run the node without a namespace or using a different namespace.

Upon being called, the service should send back a response with two arguments, ```message``` and ```success``` (we used the Trigger service from the [```std_srvs```](http://wiki.ros.org/std_srvs) package for our purposes). ```message``` provides a short message specifying whether humans were detected, and if so, how many. ```success``` is boolean, either ```true``` if humans were indeed found within the image at the moment the service was called, or ```false``` otherwise.

