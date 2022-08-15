import rospy
import smach
import smach_ros

from std_msgs.msg import Empty
from std_msgs.msg import String
from control_msgs.msg import FollowJointTrajectoryActionFeedback

# first state to ensure that the state machine is correctly started. It does not execute anything particular.
# It waits 2.5 seconds, then jump in the following state.
class setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['setup_done'])

    def execute(self, userdata):
        rospy.loginfo('State Machine has just started')
        rospy.sleep(2.5)
        return 'setup_done'

# CHECK_MAIN_ROBOT state: it verifies that HUSKY is currently able to read the real-time information of the main robot
# published on ROS. 
def monitor_cb_main_robot(ud, msg): # related to the monitoring state. Arguments are the userdata and the message. 
    #This needs to return False when we want the monitor state to terminate.
    
    # This messages is shown when the state ends and jumps to the next one
    rospy.loginfo('Checking that the main robot is online and send messages correctly on ROS')
    return False 

# (This state might be better if implement with an action to ensure that the GUI has started)
# This state publishes on a node a string saying "GUI_active". In the GUI node there is a line of code that 
# activates the code only if this message is read correctly.
class starting_GUI(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['GUI_active'])
        self.pub = rospy.Publisher('/sm_messages/start_GUI', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('Activation of the GUI node')
        
        self.pub.publish(String("starting_GUI"))
        rospy.sleep(2) # little stop of the state machine to give time to the GUI node to start
        return 'GUI_active'

# GUI_GREEN_LIGHT: this state ensures that HUSKY is activated on purpose by the operator through the custom GUI
def monitor_cb_GUI(ud, msg):
    
    rospy.loginfo('GUI is now available and functioning')
    return False


class starting_humans_check(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['humans_check_active'])
        self.pub = rospy.Publisher('/sm_messages/start_humans_check', String, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('Activation of the check_for_humans node')
        
        self.pub.publish(String("starting_humans_check"))
        rospy.sleep(2) # little stop of the state machine to give time to the node to start
        return 'humans_check_active'

# HUMANS_CHECK_DONE: we wait to finish the check for humans routing before switching state
def monitor_wait_for_human_check(ud, msg):
    
    rospy.loginfo('No humans on site. HUSKY and the main robot can start operating')
    return False


# HUSKY_MONITORING: this state is active when HUSKY operates correctly. It is ended when the button QUIT on 
# the GUI is pressed.
def monitor_quit(ud, msg):
    
    rospy.loginfo('HUSKY is ready to monitor any given point of interest.')
    return False


                

# define state End that closes the process.
class End(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('The process has ended')
        
        return 'outcome1'
    
    
    




'''Concurrent states not anymore used'''    
# class check_for_humans(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['check_for_humans_done'])
#         self.pub = rospy.Publisher('/sm_messages/check_for_humans', String, queue_size=10)

#     def execute(self, userdata):
#         rospy.loginfo('Activation of the check_for_humans node')
        
#         self.pub.publish(String("starting_check_for_humans"))
#         rospy.sleep(2) # little stop of the state machine to give time to the check_for_humans node to start
#         return 'check_for_humans_done'

# class mapping(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['mapping_done'])
#         self.pub = rospy.Publisher('/sm_messages/mapping', String, queue_size=10)

#     def execute(self, userdata):
#         rospy.loginfo('Activation of the mapping node')
        
#         self.pub.publish(String("starting_mapping"))
#         rospy.sleep(2) # little stop of the state machine to give time to the mapping node to start
#         return 'mapping_done'

# # callback function for the two monitor states of the concurrent container for mapping and check for humans
# def monitor_cb_concurrent_1(ud, msg): 
#     return False 

# # outcome_map: This is an outcome map for determining the outcome of this container. Each outcome of the container is mapped
# # to a dictionary mapping child labels onto outcomes. If none of the child-outcome maps is satisfied, the concurrence will 
# # terminate with the default outcome.
# # gets called when ALL child states are terminated (concurrent container)
# def out_cb_check_and_mapping(outcome_map):
    
#     if (outcome_map['CHECK_FOR_HUMANS'] == 'check_for_humans_done' and outcome_map['MAPPING'] == 'mapping_done') :
#         return 'concurrent_1_done'
#     else:
#         return 'concurrent_1_reset'