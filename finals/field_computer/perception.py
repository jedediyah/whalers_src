#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
import point_cloud2 as pc2
import roslib.message
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage

from math import sqrt, isnan
from matplotlib import pyplot as plt

class PerceptionController:
    
    ###
    def __init__(self):
        self.left_camera = None
        self.right_camera = None
        self.point_cloud = None
        
    ###############
    ## Point Cloud
    
    ###
    def getPointCloud(self):
        print ('getting point cloud...')
        pc_message = rospy.wait_for_message("/multisense/camera/points2", PointCloud2)
        print ('got point cloud.') 
        self.point_cloud = pc2.read_points(pc_message, field_names = ("x", "y", "z"), skip_nans=True)
        return self.point_cloud
        
    ###
    def getPointCloudImage(self):
        # Not the typical data structure (generator) for a point cloud 
        pc_message = rospy.wait_for_message("/multisense/camera/points2", PointCloud2)
        pc = pc2.read_points(pc_message, field_names = ("x", "y", "z"), skip_nans=False)
        depth_image = np.zeros((544,1024,3), np.float32)
        row=0
        col=0
        for index in xrange(544*1024):
            point = pc.next()
            if not isnan(point[2]): 
                distance = sqrt(point[0]**2+point[1]**2+point[2]**2)
                depth_image[row,col][1] = 1-min(1.0,distance/15.0) # Green only
                #depth_image[row,col][0] = point[0]
                #depth_image[row,col][1] = point[1]
                #depth_image[row,col][2] = point[2]
            col += 1
            if col >= 1024:
                col = 0
                row += 1
        return cv2.flip(cv2.flip( depth_image,0),1)
    
    
    ######################
    ## Head Camera Images
    
    ###
    def readHeadCameras(self):
        # Read left and right head cameras
        # Return OpenCV Images 
        left_data = rospy.wait_for_message("/multisense/camera/left/image_rect_color", Image)
        right_data = rospy.wait_for_message("/multisense/camera/right/image_rect_color", Image)
        try: 
            bridge = CvBridge()
            # Convert to OpenCV and invert image (Valkerie's head is upside down)
            left_image = cv2.flip(cv2.flip( bridge.imgmsg_to_cv2(left_data, "bgr8") ,0),1)
            right_image = cv2.flip(cv2.flip( bridge.imgmsg_to_cv2(right_data, "bgr8") ,0),1)
            self.left_camera = left_image
            self.right_camera = right_image
            return left_image, right_image 
        except CvBridgeError as e:
            rospy.loginfo('Error converting ROS Image to OpenCV: ')
            rospy.loginfo(e)  
            
    #######################
    ### Image Processing
    
    ###
    def maskLeftCamera(self,image):
         # Given a numpy image from Valkyrie's LEFT head camera, masks the obscured portion on the right
         ri = 0
         rf = int(len(image)*115/544) 
         ci = int(len(image[0])*956/1024)
         cf = len(image[0])
         for row in xrange(ri,rf):
            for col in xrange(ci+int(.6*(row-ri)),cf):
                image[row][col][0:3] = 255
         ri = int(len(image)*220/544)
         rf = len(image) 
         ci = int(len(image[0])*998/1024)
         cf = len(image[0])
         for row in xrange(ri,rf):
            for col in xrange(cf-int(.1*(row-ri)),cf):
                image[row][col][0:3] = 255
         return image 
     
    ###
    def removeBackground(self,img):
        # Assumes non-background items are gray and assumes Mars is not gray
        # TODO: Improve efficiency... vectorize?  Not sure how.
        for row in xrange(len(img)):
            for col in xrange(len(img[0])):
                point = [int(img[row][col][0]),int(img[row][col][1]),int(img[row][col][2])]
                if abs(point[0]-point[1])>4 or abs(point[1]-point[2])>4 or abs(point[2]-point[0])>4:
                    img[row][col][0]=img[row][col][1]=img[row][col][2]=255
        return img
        
    ###
    def getEdges(self,img):
        # Given an RGB image, returns Canny edge detected image
        # No need to blur since we are dealing with Gazebo images, not real life
        return cv2.Canny( self.removeBackground(
                            self.maskLeftCamera(img.copy()) ), 100,100)
        
    ###
    def getBlobs(self,img):
        # Setup SimpleBlobDetector parameters.
        # TODO: Potentially useful for identifying the finish square (checker pattern)
        params = cv2.SimpleBlobDetector_Params()
         
        # Change thresholds
        #params.minThreshold = 10;
        #params.maxThreshold = 200;
         
        # Filter by Area.
        params.filterByArea = True
        #params.minArea = 1500
         
        # Filter by Circularity
        #params.filterByCircularity = True
        #params.minCircularity = 0.1
         
        # Filter by Convexity
        #params.filterByConvexity = True
        #params.minConvexity = 0.87
         
        # Filter by Inertia
        #params.filterByInertia = True
        #params.minInertiaRatio = 0.01
        detector = cv2.SimpleBlobDetector(params)
        keypoints = detector.detect(edges)
        im_with_keypoints = cv2.drawKeypoints(edges, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(0)
        
    ###
    def identifyDish(self,img_bgr):
        # Looks for the communications dish in the image
        img = cv2.cvtColor(self.maskLeftCamera(self.removeBackground(img_bgr.copy())), cv2.COLOR_BGR2GRAY)
        template = cv2.imread('dish_control.png',0)   # template image
        w, h = template.shape[::-1]
        res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        top_left1 = max_loc
        bottom_right1 = (top_left1[0] + w, top_left1[1] + h)
               
        #img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        template = cv2.imread('dish.png',0)   # template image
        w, h = template.shape[::-1]
        res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        top_left2 = max_loc
        bottom_right2 = (top_left2[0] + w, top_left2[1] + h)
        
        cv2.rectangle(img_bgr,top_left1, bottom_right1, 255, 2)
        cv2.rectangle(img_bgr,top_left2, bottom_right2, 255, 1)
        
        plt.imshow(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB),cmap = 'gray')
        plt.title('Communication Dish'), plt.xticks([]), plt.yticks([])
        plt.text(top_left1[0]+5,top_left1[1]+20,'Dish Controls',color='red')
        plt.text(top_left2[0]+5,top_left2[1]+20,'Communications Dish',color='red')

        plt.show()
        cv2.waitKey(0)
        
    ###
    def getPathContour(self,img_bgr):
        # Assumes the path is relatively dark
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        lower_threshold = np.array([0,0,0])
        upper_threshold = np.array([2,2,35])
        mask = cv2.inRange(hsv, lower_threshold, upper_threshold)
        
        # Only look at bottom 3rd of image
        for r in xrange(int(0.60*len(mask))):   
            mask[r][:] = 0
            
        # Apply mask
        res = cv2.bitwise_and(img_bgr,img_bgr, mask=mask)
        
        # Threshold
        ret,mask_threshold = cv2.threshold(res,5,255,cv2.THRESH_BINARY)
        
        # Blur
        kernel = np.ones((15,15),np.float32)/225
        mask_blurred = cv2.filter2D(mask_threshold,-1,kernel)
        
        # Threshold again
        ret2,mask_threshold2 = cv2.threshold(mask_blurred,5,255,cv2.THRESH_BINARY)

        # Look for contours         
        edges = self.getEdges(mask_threshold2)
        cv2.imshow('edges',edges)
        (contours, _) = cv2.findContours( edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours.sort(key=lambda x: cv2.arcLength(x,True), reverse=True)

        #cv2.fillPoly(img_bgr,contours,(200,0,0))
        #cv2.imshow("keypoints", im_with_keypoints)
        #cv2.imshow('path',res)
        #cv2.imshow('threshold',mask_threshold)
        #cv2.imshow('blurred threshold',mask_blurred)
        
        #cv2.drawContours(img_bgr, contours, 0, (0,255,0), -1)
        #cv2.imshow('Contours',img_bgr) 
        #cv2.waitKey(0) 
        if len(contours)>0:
            return contours[0]
        else:
            return []
        
        
        
        
        
        
        
        
        
        
        
