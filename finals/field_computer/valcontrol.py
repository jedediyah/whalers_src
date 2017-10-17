#!/usr/bin/env python

import rospy

from val_walk_controller import WalkController

from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage 
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

import copy
import time
import sys
import yaml

class ValController:
    
    ###
    def __init__(self):
        self.robot_name = rospy.get_param('/ihmc_ros/robot_name')
        self.neck_publisher = rospy.Publisher('/ihmc_ros/{0}/control/neck_trajectory'.format(self.robot_name), NeckTrajectoryRosMessage, queue_size=1)
        self.walk = WalkController()
        
    ###
    def sendNeck(self):
        msg = NeckTrajectoryRosMessage()
        msg.unique_id = -1
        msg.joint_trajectory_messages = []
        
        point0 = TrajectoryPoint1DRosMessage()
        point0.time = 1.0
        point0.position = -0.3
        point0.velocity = 0
        point1 = TrajectoryPoint1DRosMessage()
        point1.time = 1.0
        point1.position = -0.3
        point1.velocity = 0
        point2 = TrajectoryPoint1DRosMessage()
        point2.time = 1.0
        point2.position = -0.3
        point2.velocity = 0
        
        msg.joint_trajectory_messages.trajectory_points.append([point0,point1,point2])
        
        self.neck_publisher.publish(msg)
        
    ###    
    def sendTrajectory(self,joint_waypoints):
        msg = NeckTrajectoryRosMessage()
        msg.unique_id = -1
        # for each set of joint states
        for y in joint_waypoints:
            # first value is time duration
            time = float(y[0])
            # subsequent values are desired joint positions
            commandPosition = array([ float(x) for x in y[1].split() ])
            msg = appendTrajectoryPoint(msg, time, commandPosition)
        rospy.loginfo('publishing neck trajectory')
        neckTrajectoryPublisher.publish(msg)

    ###
    def appendTrajectoryPoint(self,neck_trajectory, time, positions):
        if not neck_trajectory.joint_trajectory_messages:
            neck_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
        for i, pos in enumerate(positions):
            point = TrajectoryPoint1DRosMessage()
            point.time = time
            point.position = pos
            point.velocity = 0
            neck_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
        return neck_trajectory

#rostopic pub /ihmc_ros/valkyrie/control/neck_trajectory ihmc_msgs/NeckTrajectoryRosMessage '{joint_trajectory_messages: [{trajectory_points: [time: 0.1, position: -0.05, velocity: 0.0, unique_id: 1]},{trajectory_points: [time: 0.1, position: 0.0, velocity: 0.0, unique_id: 1]},{trajectory_points: [time: 0.1, position: 0.0, velocity: 0.0, unique_id: 1]}], unique_id: 1}'

        
        
    def shuffleForward(self):
        self.walk.shuffleForward()
        
        
        
        
        
        
        
        
        
        
        
