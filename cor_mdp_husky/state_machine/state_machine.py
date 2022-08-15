#!/usr/bin/env python

import rospy
import smach
import smach_ros
import sm_functions

# Importing messages sent by the state machine or read by it
from std_msgs.msg import Empty
from std_msgs.msg import String
from control_msgs.msg import FollowJointTrajectoryActionFeedback

def main():
    # Create a ROS node for the state machine
    rospy.init_node('node_state_machine')
    
    '''Concurrent states not anymore used''' 
    # # Defining the first parallel states: CHECK_FOR_HUMANS and MAPPING
    # humans_and_mapping = smach.Concurrence(outcomes=['concurrent_1_done', 'concurrent_1_reset'],
    #                                     default_outcome='concurrent_1_reset',
    #                                     child_termination_cb=None,
    #                                     outcome_cb=sm_functions.out_cb_check_and_mapping)
    # # adding the states to the concurrence container
    # with humans_and_mapping:
    #     smach.Concurrence.add('CHECK_FOR_HUMANS', sm_functions.check_for_humans())
    #     smach.Concurrence.add('MAPPING', sm_functions.mapping())
    #     smach.Concurrence.add('CHECK_FOR_HUMANS_DONE', smach_ros.MonitorState("/sm_messages/check_for_humans", 
    #                                                                         Empty, sm_functions.monitor_cb_concurrent_1))
    #     smach.Concurrence.add('MAPPING_DONE', smach_ros.MonitorState("/sm_messages/mapping", 
    #                                                                         Empty, sm_functions.monitor_cb_concurrent_1))
        
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end_sm'])

    # Open the container for the main structure of the state machine
    with sm:
        # Add states to the container
        smach.StateMachine.add('SETUP', sm_functions.setup(), 
                               transitions={'setup_done':'CHECK_MAIN_ROBOT'})
        smach.StateMachine.add('CHECK_MAIN_ROBOT', smach_ros.MonitorState("/ur5/arm_controller/follow_joint_trajectory/feedback", 
                                                                            FollowJointTrajectoryActionFeedback, sm_functions.monitor_cb_main_robot), transitions={'invalid':'STARTING_GUI', 'valid':'CHECK_MAIN_ROBOT', 'preempted':'CHECK_MAIN_ROBOT'})
        smach.StateMachine.add('STARTING_GUI', sm_functions.starting_GUI(), 
                               transitions={'GUI_active':'GUI_GREEN_LIGHT'})
        smach.StateMachine.add('GUI_GREEN_LIGHT', smach_ros.MonitorState("/sm_messages", Empty, sm_functions.monitor_cb_GUI), transitions={'invalid':'STARTING_HUMANS_CHECK', 'valid':'GUI_GREEN_LIGHT', 'preempted':'GUI_GREEN_LIGHT'})
        
        '''Concurrent states not anymore used''' 
        # smach.StateMachine.add('HUMANS_AND_MAPPING', humans_and_mapping, transitions={'concurrent_1_done':'BAR', 'concurrent_1_reset':'GUI_GREEN_LIGHT'}) 
        
        smach.StateMachine.add('STARTING_HUMANS_CHECK', sm_functions.starting_humans_check(), 
                               transitions={'humans_check_active':'HUMANS_CHECK_DONE'})
        smach.StateMachine.add('HUMANS_CHECK_DONE', smach_ros.MonitorState("/sm_messages/humans_check_done", Empty, sm_functions.monitor_wait_for_human_check), transitions={'invalid':'HUSKY_MONITORING', 'valid':'HUMANS_CHECK_DONE', 'preempted':'HUMANS_CHECK_DONE'})
        
        smach.StateMachine.add('HUSKY_MONITORING', smach_ros.MonitorState("/sm_messages/quit", 
                                                                            Empty, sm_functions.monitor_quit), transitions={'invalid':'END', 'valid':'HUSKY_MONITORING', 'preempted':'HUSKY_MONITORING'})
        smach.StateMachine.add('END', sm_functions.End(), 
                               transitions={'outcome1':'end_sm'})

    # Create and start the introspection server. This is needed to be able to see the state machine graph in SMACH VIEWER
    sis = smach_ros.IntrospectionServer('state_machine_graph', sm, '/HUSKY_SM')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


# Given the structure of the code, the following command just called the function main() that contains all the process of the
# state machine.
if __name__ == '__main__':
    main()