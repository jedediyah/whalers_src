#!/usr/bin/env python

import rospy

from val_arm_controller import ArmController
from val_walk_controller import WalkController

import random

def run():    
    # Run parameters
    step_size = .43
    first_step = step_size/2.0
    
    t_times = [.38,.39]
    s_times = [0.32,0.33,0.34,0.35,0.36,0.37]
    
    transfer_time = random.choice(t_times)
    swing_time = random.choice(s_times)
    rospy.loginfo("Times picked: " + str(transfer_time) + ", " + str(swing_time))
    
    #transfer_time = .4
    #swing_time = .40
    BUTTON_POSITION = [-1.34, 0.73, 1.57, 0.0, 0.0, 0.0, 0.0] 
    
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
    rospy.sleep(0.01)  # Can this be smaller?
    
    #######################################
    # Walk through door
    
    step_size = .47
    swing_time = .55
    walk.addStepToWalkPlan(1.95*step_size, transfer_time, swing_time)  # Step over door jam
    swing_time = .40
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
    