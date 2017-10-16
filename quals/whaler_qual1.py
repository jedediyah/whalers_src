#!/usr/bin/env python

import rospy
import cv2 
import numpy as np 

from srcsim.msg import Console
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import PointCloud2
#from sensor_msgs.msg import point_cloud2

from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from math import sqrt, isnan

# Point cloud 
import point_cloud2 as pc2
import roslib.message
from stereoProcessor import stereoProcessor 

def readHeadCameras():
    # Read left and right head cameras
    # Return OpenCV Images 
    left_data = rospy.wait_for_message("/multisense/camera/left/image_rect_color", Image)
    right_data = rospy.wait_for_message("/multisense/camera/right/image_rect_color", Image)
    try: 
        bridge = CvBridge()
        # Convert to OpenCV and invert image (Valkerie's head is upside down)
        left_image = cv2.flip(cv2.flip( bridge.imgmsg_to_cv2(left_data, "bgr8") ,0),1)
        right_image = cv2.flip(cv2.flip( bridge.imgmsg_to_cv2(right_data, "bgr8") ,0),1)
        return left_image, right_image 
    except CvBridgeError as e:
        rospy.loginfo('Error converting ROS Image to OpenCV: ')
        rospy.loginfo(e)
    
def checkForColors(image):
    #print "Checking for colors..." 
    pixels = []
    colors = []
    dc=10 # Acceptable error in color detection (256 scale)
    boundaries = [  ([255-dc, 0, 0], [255, dc, dc]), 
	                ([0, 255-dc, 0], [dc, 255, dc]),
	                ([0, 0, 255-dc], [dc, dc, 255]) ] 
	# loop over the boundaries
    for (lower, upper) in boundaries:
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        # find the colors within the specified boundaries and apply the mask
        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)
        gray_image = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        (contours, _) = cv2.findContours(gray_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) < 45:     # Typical area is around 75 for small buttons 
                continue
            rectangle = cv2.minAreaRect(c)
            [rect_y,rect_x] = rectangle[0]  # Silly backwards graphics :(
            color = image[int(rect_x), int(rect_y)]
            colors.append(color)
            pixels.append([int(rect_x), int(rect_y)])
            highlight = [255-color[0], 255-color[1], 255-color[2]]
            box = cv2.cv.BoxPoints(rectangle)
            box = np.int0(box)
            cv2.drawContours(image,[box],0,highlight,1)
            
            #print "   Color " + str(image[int(rect_x), int(rect_y)])  + " found at pixel " + str([int(rect_x), int(rect_y)])
            cv2.imshow("color detection",image) 
            cv2.waitKey(3)
    return colors, pixels


############
# Test point cloud
def grayPointCloud(points):
    maximum = 0
    width = 1024
    height = 544
    depth_image = np.zeros((height,width,3), np.uint8)
    for p in points:
        if not isnan(p[0]) and not isnan(p[1]) and not isnan(p[2]):
            distance = sqrt(p[0]**2 + p[1]**2 + p[2]**2)
            if distance > maximum:
                maximum = distance
            depth_image[index%pc_rows,index/pc_rows] = min(255,distance)
            print "Maximum : " + str(maximum)
    cv2.imshow("depth data",depth_image) 
    cv2.waitKey(0)

def getPointCloud():
    # Not the typical data structure (generator) for a point cloud 
    pc_message = rospy.wait_for_message("/multisense/camera/points2", PointCloud2)
    pc = pc2.read_points(pc_message, field_names = ("x", "y", "z"), skip_nans=False)
    depth_image = np.zeros((544,1024,3), np.float32)
    row=0
    col=0
    for index in xrange(544*1024):
        point = pc.next()
        if not isnan(point[2]): 
            depth_image[row,col][0] = point[0]
            depth_image[row,col][1] = point[1]
            depth_image[row,col][2] = point[2]
        col += 1
        if col >= 1024:
            col = 0
            row += 1
    return cv2.flip(cv2.flip( depth_image,0),1)
    
    
def run():
    # Initialize first_frame for motion / color-change detection 
    rospy.loginfo('Running whaler_qual 1.')
    pub = rospy.Publisher('/srcsim/qual1/light',Console,queue_size=50)
    #stereo = stereoProcessor()
    
    lastPixel = [0,0]
    while True:
        [left_image, right_image] = readHeadCameras()
        [colors,pixels] = checkForColors(left_image)  
        if len(colors) > 0:
            for color,pixel in zip(colors,pixels):
                px = pixel[0]
                py = pixel[1]
                print "  BGR Color " + str(left_image[px,py])  + " found at pixel " + str([px,py])
                if (lastPixel[0]-px)**2 + (lastPixel[1]-py)**2 < 25:
                    print "Already published this light"
                    continue
                lastPixel = [px,py]
                
                # Get 3d coordinate
                pc = getPointCloud() 
                vector = pc[px,py]
                print vector 
                
                # Scale light color to [0,1]
                ledColor = [0.0, 0.0, 0.0]
                if abs(color[0])>128:
                    ledColor[2] = 1.0
                if abs(color[1])>128:
                    ledColor[1] = 1.0
                if abs(color[2])>128:
                    ledColor[0] = 1.0
                    
                # Publish Console message
                msg = Console()
                msg.x = vector[0]
                msg.y = vector[1]
                msg.z = vector[2]
                msg.r = ledColor[0]
                msg.g = ledColor[1]
                msg.b = ledColor[2]
                pub.publish(msg)

                                
        rospy.sleep(0.1)
    cv2.destroyAllWindows() 
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('whaler_qual1_run')
        rospy.loginfo('Starting whaler_qual_1_run...')
        run()
    except rospy.ROSInterruptException:
        rospy.loginfo('whaler_qual1_run stopped unexpectedly')
        cv2.destroyAllWindows() 
    
