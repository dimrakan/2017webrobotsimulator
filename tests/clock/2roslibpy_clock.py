#!/usr/bin/env python

from __future__ import print_function

from roslibpy import Message, Ros, Topic
import rospy
import logging
import atexit
import signal
from bond.msg import Status
from sensor_msgs.msg import Image,JointState,PointCloud2,CompressedImage,Imu
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from gazebo_msgs.msg import LinkStates,ModelStates


logging.basicConfig(level=logging.INFO)
global my
global my2
class ROSCloudSubBridge(object):

    def __init__(self, ip, port, local_topic_name, remote_topic_name, message_type):
        self._ip = ip
        self._port = port
        self._local_topic_name = local_topic_name     
        self._remote_topic_name = remote_topic_name
        self._message_type = message_type
        self._ros_client= Ros(self._ip,self._port) 
        self._rosPub = None  # Should be specified by the automation engine
        self._listener = None  # Should be specified by the automation engine
        self._2listener = None  # Should be specified by the automation engine

        self._logger = logging.getLogger(self.__class__.__name__)
        self._logger.setLevel(logging.DEBUG)

    def _run_topic_pubsub(self):
    	try:
            self._listener = Topic(self._ros_client, self._remote_topic_name, self._message_type)
#            self._2listener = Topic(self._ros_client, self._remote_topic_name, self._message_type)
            rospy.init_node('cloudsub', anonymous=True)
            global my
#            global my2
#            my2=rospy.get_time()    
            my=rospy.get_time()
            self._ros_client.on_ready(self._start_receiving, run_in_thread=True)
            self._ros_client.run_event_loop() 
        except:        
            self._logger.exception("Fatal error in constructor")
    

    def _receive_message(self,message):
        global my
        rospy.loginfo(rospy.get_caller_id() + " Message type %s ",self._message_type)
        rospy.loginfo(rospy.get_caller_id() + " Time from previous message %s ",(rospy.get_time()-my))
        my=rospy.get_time()
        try:
            msg=Clock()
            msg.clock.secs=message['clock']['secs']
            msg.clock.nsecs=message['clock']['nsecs']
            self._rosPub=rospy.Publisher(self._local_topic_name, Clock, queue_size=10) #message type is String instead of msg_stds/String
            self._rosPub.publish(msg)
        except:
           print('Error')        

#    def _2receive_message(self,message):
#        global my2
#        rospy.loginfo(rospy.get_caller_id() + "I heard a message %s",(rospy.get_time()-my2))
#        my2=rospy.get_time()
#        try:
#            msg=Clock()
#            msg.clock.secs=message['clock']['secs']
#            msg.clock.nsecs=message['clock']['nsecs']
#            self._rosPub=rospy.Publisher('/test/clock2', Clock, queue_size=10) #message type $
#            self._rosPub.publish(msg)
#        except:
#           print('Error')        


    def _start_receiving(self): 
        self._listener.subscribe(self._receive_message)
#        self._listener2.subscribe(self._2receive_message)


    def start(self):
        signal.signal(signal.SIGINT, lambda *x: self.stop())
        self._run_topic_pubsub()

    def stop(self):
        if self._ros_client is not None:
            self._ros_client.terminate()


if __name__ == '__main__':
    a=ROSCloudSubBridge(ip='83.212.96.15', port="8018", local_topic_name="/local/clock",remote_topic_name="/clock", message_type="rosgraph_msgs/Clock")
    a.start()

