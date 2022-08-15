
#include "navigation_to_waypoints.hpp"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
navigation_to_waypoints::navigation_to_waypoints(): ac_("move_base", true)
{

}
void navigation_to_waypoints::move_husky(std::array<float, 7> waypoint)
{
       
        
        
        //wait for the action server to come up
        while(!ac_.waitForServer(ros::Duration(5.0))){
            ROS_INFO("waiting for the move_base action server to come up");
        }

        
        move_base_msgs::MoveBaseGoal goal;
        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = waypoint[0];
        goal.target_pose.pose.position.y = waypoint[1];
        goal.target_pose.pose.position.z = waypoint[2];

        goal.target_pose.pose.orientation.x = waypoint[3];
        goal.target_pose.pose.orientation.y = waypoint[4];
        goal.target_pose.pose.orientation.z = waypoint[5];
        goal.target_pose.pose.orientation.w = waypoint[6];

        ROS_INFO("Sending goal");

        ac_.sendGoal(goal);

        ac_.waitForResult();

        if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 1 meter forward");
        else 
            ROS_INFO("The base failed to move forward 1 meter for some reason");
        

    }


