#!/usr/bin/env python

### Field Computer Control
# Controls Val's autonomy and listens for user commands from OCU

import rospy
import cv2
import numpy as np

from perception import PerceptionController
from valcontrol import ValController

import cPickle
import zlib


class FieldController:
    def __init__(self):
        self.thisnode = rospy.init_node('whaler_fc_control')
        self.perception = PerceptionController()
        self.val = ValController() 
    
    def run(self):
    
        self.task1()
        return 
        
        self.val.shuffleForward()
        
        cv2.imshow("pcl",self.perception.getPointCloudImage())
        #cv2.waitKey(0)
        
        [left,right] = self.perception.readHeadCameras()
        
        #left_small = cv2.resize(left,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
        
        self.perception.identifyPath(left)
        left_filtered = self.perception.removeBackground(self.perception.maskLeftCamera(left))
        
        #left_small = cv2.resize(left,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
        cv2.imshow('edges',self.perception.getEdges(left))
                   
        #cv2.imshow('left',left) 
        #cv2.imshow('left_small',left_small)
        left_filtered = self.perception.removeBackground(self.perception.maskLeftCamera(left))
        cv2.imshow("Masked",left_filtered)
        
        cv2.waitKey(0)
        
    def calculateWalkingDirectionFromContour(original_image, contour):
        rows, cols, _ = original_image.shape
        # Lots of issues here...
        
        
    def task1(self):
        # Automous task 1
        # Look around and find path, and possibly dish
        # TODO: look down (when the neck works)
        [left,_] = self.perception.readHeadCameras()
        left = cv2.resize(left,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
        path_contour = self.perception.getPathContour(left)
        cv2.drawContours(left, [path_contour], -1, (0,255,0), -1)
        cv2.imshow('Path',left)
        cv2.waitKey(0)
                
        # Move along path until positioned in front of dish controls
        
        # Reposition
        
        # Correct dish position 
    

if __name__== "__main__":
    fieldControl = FieldController()
    fieldControl.run()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
