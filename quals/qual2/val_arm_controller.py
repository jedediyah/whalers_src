#!/usr/bin/env python

import copy
import time
import rospy

from numpy import append

from std_msgs.msg import String
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import HandDesiredConfigurationRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

ZERO_VECTOR = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
HOME_POSITION = [-0.3, 1.3, 0.8, 1.3, 0.0, 0.0, 0.0]
#BUTTON_POSITION = [1.8, -0.8, 0.0, 0.0, 0.0, 0.0, 0.0]  # Change -.8 to -.9??
BUTTON_POSITION = [-1.34, 0.84, 1.55, 0.0, 0.0, 0.0, 0.0]
#BACK_POSITION = [-1.5, -1.25, -1.51, 1.0, 0.0, 0.0, 0.0]
BACK_POSITION = [-0.5, -1.15, -1.51, 1.0, 0.0, 0.0, 0.0]
DOWN_POSITION = [0.0, 1.3, 0.0, 0.0, 0.0, 0.0, 0.0]
IN_TIGHT = [-0.2, 1.5, 0.7, 2.0, 0.0, 0.0, 0.0]

ROBOT_NAME = None

def makeLeft(target):
    # Unfortunately, only some of the joints in the left arm are negative of 
    return [target[0],-target[1],target[2],-target[3],target[4],target[5],target[6]]
    
class ArmController:

    def __init__(self):
        try:
            #rospy.init_node('ihmc_arm_demo1')
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
            self.armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME), ArmTrajectoryRosMessage, queue_size=1)
            self.handDesiredConfigurationPublisher = rospy.Publisher("/ihmc_ros/{0}/control/hand_desired_configuration".format(ROBOT_NAME),HandDesiredConfigurationRosMessage, queue_size=1)
            self.rate = rospy.Rate(10) # 10hz
            #time.sleep(0.1)

            # make sure the simulation is running otherwise wait
            if self.armTrajectoryPublisher.get_num_connections() == 0:
                #rospy.loginfo('waiting for subscriber...')
                while self.armTrajectoryPublisher.get_num_connections() == 0:
                    self.rate.sleep()

            if not rospy.is_shutdown():
                pass
                #rospy.loginfo('Val arm controller ready.')
                #time.sleep(2)

        except rospy.ROSInterruptException:
            pass
         
    def publish(self,msg):
        # Important: arm joint commands will be lost if sent while Val is in motion. 
        #            So we wait until Val is done walking before sending joint commands. 
        while rospy.wait_for_message("/ihmc_ros/valkyrie/output/robot_motion_status", String).data != 'STANDING':
            self.rate.sleep() 
        #rospy.loginfo('Val is STANDING and ready for joint commands')
        self.armTrajectoryPublisher.publish(msg)
        
    def rightHandConfiguration(self, configuration):
        # "hand_configuration" enum values:
        # STOP=0 # stops the fingers at their current position
        # OPEN=1 # fully opens the fingers
        # CLOSE=2 # fully closes the fingers
        # CRUSH=3 # fully closes the fingers applying maximum force
        # HOOK=4 # closes all but one finger to create a hook
        # BASIC_GRIP=5 # sets gripper to use a standard grasp
        # PINCH_GRIP=6 # sets gripper to use a pinch grasp where the thumb and fingers come together when closed
        # WIDE_GRIP=7 # sets gripper to use a wide-spread finger grasp
        # SCISSOR_GRIP=8 # sets gripper to use a scissor grasp where the index and middle finger come together when closed
        # RESET=9 # sets all fingers to their zero position
        # OPEN_FINGERS=10 # fully open all fingers except the thumb
        # OPEN_THUMB=11 # fully open the thumb only
        # CLOSE_FINGERS=12 # fully close all fingers except the thumb
        # CLOSE_THUMB=13 # fully close the thumb only
        msg = HandDesiredConfigurationRosMessage()
        msg.robot_side = 1
        msg.hand_desired_configuration = configuration
        msg.unique_id = -1
        print "Sending hand configuration: " + str(msg.hand_desired_configuration)
        self.handDesiredConfigurationPublisher.publish(msg)
          
    def sendLeftArm(self, target):
        msg = ArmTrajectoryRosMessage()
        msg.robot_side = ArmTrajectoryRosMessage.LEFT
        if target == "home":
            msg = self.appendTrajectoryPoint(msg, 2.0, makeLeft(HOME_POSITION)) 
        elif target == "button":
            msg = self.appendTrajectoryPoint(msg, 2.0, makeLeft(BUTTON_POSITION)) 
        elif target == "down":
            msg = self.appendTrajectoryPoint(msg, 2.0, makeLeft(DOWN_POSITION)) # [0.0, -1.3, 0.0, 0.0, 0.0, 0.0, 0.0]) # DOWN_POSITION) 
        elif target == "extend":
            msg = self.appendTrajectoryPoint(msg, 2.0, ZERO_VECTOR) 
        elif target == "tight":
            msg = self.appendTrajectoryPoint(msg, 2.0, makeLeft(IN_TIGHT)) 
        else:
            #rospy.loginfo('Failed to send arm to ' + target + ' position')
            #return 1
            msg = self.appendTrajectoryPoint(msg, 2.0, makeLeft(target))
        msg.unique_id = -1
        if isinstance(target, basestring):
            strtarget = target
        else:
            strtarget = str(target)
        rospy.loginfo('sending left arm to ' + strtarget + ' position')
        self.publish(msg)
        return 0
            
    def sendRightArm(self, target):
        msg = ArmTrajectoryRosMessage()
        msg.robot_side = ArmTrajectoryRosMessage.RIGHT
        if target == "home":
            msg = self.appendTrajectoryPoint(msg, 2.0, HOME_POSITION) 
        elif target == "button":
            msg = self.appendTrajectoryPoint(msg, 2.0, BUTTON_POSITION) 
        elif target == "down":
            msg = self.appendTrajectoryPoint(msg, 2.0, DOWN_POSITION) 
        elif target == "extend":
            msg = self.appendTrajectoryPoint(msg, 2.0, ZERO_VECTOR) 
        elif target == "tight":
            msg = self.appendTrajectoryPoint(msg, 2.0, IN_TIGHT)
        elif target == "back":
            msg = self.appendTrajectoryPoint(msg, 2.0, BACK_POSITION)
        else:
            #rospy.loginfo('Failed to send arm to ' + target + ' position')
            #return 1
            msg = self.appendTrajectoryPoint(msg, 2.0, target) 
        msg.unique_id = -1
        if isinstance(target, basestring):
            strtarget = target
        else:
            strtarget = str(target)
        rospy.loginfo('sending right arm to ' + strtarget + ' position')
        self.publish(msg)
        return 0

#    def sendRightArmTrajectory(self):
#        msg = ArmTrajectoryRosMessage()
#        msg.robot_side = ArmTrajectoryRosMessage.RIGHT
#        msg = self.appendTrajectoryPoint(msg, 2.0, ZERO_VECTOR) # Trajectories to send
#        msg.unique_id = -1
#        rospy.loginfo('publishing right trajectory')
#        self.publish(msg)

    def appendTrajectoryPoint(self, arm_trajectory, time, positions):
        if not arm_trajectory.joint_trajectory_messages:
            arm_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
        for i, pos in enumerate(positions):
            point = TrajectoryPoint1DRosMessage()
            point.time = time
            point.position = pos
            point.velocity = 0
            arm_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
        return arm_trajectory
    