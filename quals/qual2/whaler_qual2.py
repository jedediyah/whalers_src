#!/usr/bin/env python

import rospy

from val_arm_controller import ArmController
from val_walk_controller import WalkController

def run():
    # Run parameters
    step_size = .43
    first_step = step_size/2.0
    
    transfer_time = .4
    swing_time = .40
    BUTTON_POSITION = [-1.34, 0.71, 1.57, 0.0, 0.0, 0.0, 0.0]  # Step size .4175
    
    # Initialize controllers 
    arms = ArmController()
    walk = WalkController() 
    
    #######################################
    # Distance to door is 3.4 meters (with hand extended)
    # Walk to door
    arms.sendLeftArm('tight')
    arms.sendRightArm(BUTTON_POSITION)
    rospy.sleep(0.1) 
    
    
    walk.addStepToWalkPlan(first_step, transfer_time, swing_time)
    for i in range(1,8):
        walk.addStepToWalkPlan(first_step + i*step_size, transfer_time, swing_time)
    walk.go()
   
    arms.sendRightArm('back')
    rospy.sleep(0.1)  # Can this be smaller?
    
    #######################################
    # Walk through door
    
    step_size = .47
    swing_time = .55
    walk.addStepToWalkPlan(1.95*step_size, transfer_time, swing_time)  # Step over door jam
    swing_time = .45
    walk.addStepToWalkPlan(1.95*step_size, transfer_time, swing_time)   # make bigger???
    
     
    walk.addStepToWalkPlan(3.8*step_size, transfer_time, swing_time)  
    
    
    walk.addStepToWalkPlan(3.7*step_size, transfer_time, swing_time)
    walk.addStepToWalkPlan(5*step_size, transfer_time, swing_time)
    walk.addStepToWalkPlan(5*step_size, transfer_time, swing_time)
    walk.go()
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('whaler_qual2_run')
        rospy.loginfo('starting test...')
        
        rospy.loginfo('Waiting for start signal...')
        # Autorun after 25.0 seconds
        while rospy.get_time() < 25.0:
            rospy.sleep(0.1) 
        rospy.loginfo('starting test')
        run()
    except rospy.ROSInterruptException:
        rospy.loginfo('Failed to start test')
        pass
    
