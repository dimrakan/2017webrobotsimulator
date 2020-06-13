#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist # from std_msgs.msg import String  # This should be specified by the automation engine
from rosbridge_pyclient import *
import time
import atexit
import logging

logging.basicConfig(level=logging.INFO)


class ROSCloudPubBridge(object):

    def __init__(self, ip, port, local_topic_name, remote_topic_name, message_type):
        self._ip = ip
        self._port = port
        self._local_topic_name = local_topic_name     
        self._remote_topic_name = remote_topic_name
        self._message_type = message_type
        self._cloudExecutor = ExecutorThreaded(self._ip, self._port)
        self._rosSub = None  # Should be specified by the automation engine
        self._cloudPub = None  # Should be specified by the automation engine
        self._logger = logging.getLogger(self.__class__.__name__)
        self._logger.setLevel(logging.DEBUG)
        atexit.register(self._cleanup())

    @property
    def ip(self):
        return self._ip
        
    @property
    def port(self):
        return self._port

    @property
    def local_topic_name(self):
        return self._local_topic_name

    @property
    def remote_topic_name(self):
        return self._remote_topic_name

    @property
    def message_type(self):
        return self._message_type

    def start(self):
        try:
            # Handle ROS Node name. Should be unique for each instance. If not ROS will raise an error,
            # when trying to registed a Node with a name that already exists and is owned by another Node.
            rospy.init_node('cloudpub', anonymous=True)
            self._cloudExecutor.start()
            # Should be specified by the automation engine. Can be Publisher or Subscriber, etc
            self._cloudPub = Publisher(self._cloudExecutor, self._remote_topic_name, self._message_type)
            # Should be specified by the automation engine. Can be Publisher or Subscriber, etc
            self._rosSub = rospy.Subscriber(self._local_topic_name, Twist, self._callback) #self._rosSub = rospy.Subscriber(self._local_topic_name, String, self._callback)
            rospy.spin()
        except Exception as exc:
            self._logger.exception("Fatal error in constructor")

    def _callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s" , data)
        try:
            self._cloudPub.publish({"linear": {'x':data.linear.x,'y':data.linear.y,'z':data.linear.z},'angular':{'x':data.angular.x,'y':data.angular.y,'z':data.angular.z}}) #self._cloudPub.publish({"data": data.data})
            
        except:
            self._logger.exception("Exception occured in ROS Subscriber callback")

    def _cleanup(self):
        if self._cloudPub is not None:
            self._cloudPub.unregister()


if __name__ == '__main__':
    l = ROSCloudPubBridge(ip="83.212.96.15", port="8026", local_topic_name="/robot/mobile_base/commands/velocity",remote_topic_name="/mobile_base/commands/velocity", message_type="geometry_msgs/Twist") 
    l.start()
