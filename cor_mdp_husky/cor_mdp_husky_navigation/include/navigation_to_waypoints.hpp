#ifndef NAVIGATION_TO_WAYPOINTS_HPP_
#define NAVIGATION_TO_WAYPOINTS_HPP_
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>




typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class navigation_to_waypoints
{

private:

MoveBaseClient ac_;



public:

void move_husky(std::array<float, 7> waypoint);
navigation_to_waypoints();



};
#endif 