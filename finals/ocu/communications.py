#!/usr/bin/env python

# This is the OCU side of the transceiver

import sys
import struct
import socket
import cPickle
import zlib

import cv2 
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import rospy
import roslib.message
import std_msgs.msg

import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

RVIZ_FRAME = 'pelvis' 

class MessageMaker:
    # Translates between TCP data ROS messages

    ###
    def __init__(self):
        #self.joint_state_publisher = rospy.Publisher('/ihmc_ros/valkyrie/output/joint_states',JointState,queue_size=10)
        self.image_publisher = rospy.Publisher('/multisense/camera/left/image_rect_color',Image,queue_size=10)
        self.points2_publisher = rospy.Publisher('/multisense/camera/points2',PointCloud2,queue_size=10)
        self.joints_publisher = rospy.Publisher('/joint_states',JointState,queue_size=10)
        self.tf_publisher = rospy.Publisher('/tf',TFMessage,queue_size=10)

    ###
    def makeROS(self, data):
        # Given a list of the form [{message type (str)}, {message data (dict)}] received via TCP,
        # parse and create a ROS message.  
        print "Data received (" + str(sys.getsizeof(data[1])) + "): " + data[0]
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = RVIZ_FRAME
        if data[0] == '/hardware_joint_states':
            pass
        elif data[0] == '/ihmc_ros/valkyrie/output/joint_states':
            msg = JointState()
            msg.header = header
            msg.name = data[1]['name']
            msg.position = data[1]['position']
            self.joints_publisher.publish(msg)
        elif data[0] == '/tf':
            msg = TFMessage()
            msg.transforms = data[1]['transforms']
            self.tf_publisher.publish(msg)
        elif data[0] == '/multisense/camera/points2':
            self.points2_publisher.publish(pc2.create_cloud_xyz32(header,data[1]))
        elif data[0] == '/multisense/camera/left/image_rect_color':
            msg = Image()
            msg.header = header 
            msg.height = data[1]['height']
            msg.width = data[1]['width']
            msg.encoding = data[1]['encoding']
            msg.is_bigendian = data[1]['is_bigendian']
            msg.step = data[1]['step']
            msg.data = data[1]['data']
            self.image_publisher.publish(msg)
            
            bridge = CvBridge()
            left_image = cv2.flip(cv2.flip( bridge.imgmsg_to_cv2(msg, "bgr8") ,0),1)
            cv2.imshow("left camera",left_image) 
            cv2.waitKey(3)
        else: 
            print ("WARNING!  Unrecognized data packet received.  Ignoring data")


class Transceiver:
    # A TCP transmitter and receiver
    
    ###
    def __init__(self):
        self.client = True 
        
    ###
    def startClient(self, ip_address, port):
        print ('Initializing network client...')
        self.ip_address = ip_address
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip_address, self.port))
        print ('Connected to ' + str(self.ip_address))
   
    ###
    def startServer(self,port):
        print ('Starting network server...')
        self.port = port 
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(('', self.port))
        sock.listen(1)
        self.sock, self.addr = sock.accept()    # Initialize connection 
        print ('Connected to ' + str(self.addr))
        
    ###
    def stop(self):
        self.sock.close() 
        
    ###
    def transmit(self, data):
        msg = zlib.compress(cPickle.dumps( data ))  # Serialize and compress
        # Prefix each message with a 4-byte length (network byte order)
        msg = struct.pack('>I', len(msg)) + msg
        self.sock.sendall(msg)

    ###
    def receive(self):
        # Read message length and unpack it into an integer
        raw_msglen = self.recvall(4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        # Read the message data
        return cPickle.loads(zlib.decompress( self.recvall(msglen) ))
    
    ###
    def recvall(self, n):
        # Helper function to recv n bytes or return None if EOF is hit
        data = ''
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data
        
        
