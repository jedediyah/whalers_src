#!/usr/bin/env python

### OCU Computer 

from communications import Transceiver
from communications import MessageMaker

import rospy
from std_msgs.msg import String

class WhalerOCUComms:
    def __init__(self):
        self.thisnode = rospy.init_node('whalers_ocu_comms')
        self.transmitter = Transceiver()
        self.receiver = Transceiver()
        self.msgs = MessageMaker() 
        self.receiver.startClient('10.201.140.155',5005) # Waits for connection with FC
        #self.transmitter.startServer(4004)                     # Waits for connection with FC
        
        #rospy.Subscriber('/whaler_command', , self.transmit_whaler_command)

    def run(self):
        # The main loop of Whaler OCU Comms
        while True:
            data = self.receiver.receive()   # Receive message from Field Computer
            self.msgs.makeROS(data)             # Convert FC message to ROS message
        self.transmitter.stop()
        self.receiver.stop() 
        
    def transmit_whaler_command(self,data):
        self.transmitter.transmit(['/whaler_command',{'data':data}])
    
    

if __name__== "__main__":
    whalers = WhalerOCUComms()
    whalers.run()
    
