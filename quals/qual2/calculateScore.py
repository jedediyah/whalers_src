#!/usr/bin/env python

from datetime import datetime
import rospy
from nav_msgs.msg import Odometry
#from rosgraph_msgs.msg import Clock

def run():

    startCrossed = False
    finishCrossed = False 
    startTime = 0.0
    finishTime = 0.0

    while True:
        pose = rospy.wait_for_message("/ihmc_ros/valkyrie/output/robot_pose", Odometry)
        
        x = pose.pose.pose.position.x
        
        if x < 0.3:
            startCrossed = False
        if x >= 0.50:
            if startCrossed == False:
                startTime = rospy.get_time() 
                print "New trial detected, 0.5 m crossed at time " + str(startTime)
                startCrossed = True
                rospy.sleep(5) 
            if x >= 4.5:
                if finishCrossed == False:
                    finishCrossed = True
                    stopTime = rospy.get_time() 
                    print "     *****######***** Trial finished!  Total time taken: " + str(stopTime-startTime)
                    print str(datetime.now())
                    with open("LOG.txt","a+") as myfile:
                        myfile.write(str(stopTime-startTime) + "  ")
                        myfile.write(str(datetime.now())+"\n")
        
            
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('score_node')
        rospy.loginfo('Starting walk score calculator...')
        run()
    except rospy.ROSInterruptException:
        rospy.loginfo('Failed to start walk_scorer')
        pass
    