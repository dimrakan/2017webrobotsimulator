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
from geometry_msgs.msg import TransformStamped
import tf2_ros

logging.basicConfig(level=logging.INFO)
global my
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

        self._logger = logging.getLogger(self.__class__.__name__)
        self._logger.setLevel(logging.DEBUG)
        self.br =tf2_ros.StaticTransformBroadcaster()
        self.t = []

    def _run_topic_pubsub(self):
        try:
            self._listener = Topic(self._ros_client, self._remote_topic_name, self._message_type)
            rospy.init_node('cloudsub', anonymous=True)
            global my
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
        for i in range(len(message['transforms'])):
            self.t.append(TransformStamped())
            self.t[-1].header.stamp = rospy.Time.now()
            self.t[-1].header.frame_id = message['transforms'][i]['header']['frame_id']
            self.t[-1].child_frame_id =  message['transforms'][i]['child_frame_id']
            self.t[-1].transform.translation.x = message['transforms'][i]['transform']['translation']['x']
            self.t[-1].transform.translation.y = message['transforms'][i]['transform']['translation']['y']
            self.t[-1].transform.translation.z = message['transforms'][i]['transform']['translation']['z']
            self.t[-1].transform.rotation.x = message['transforms'][i]['transform']['rotation']['x']
            self.t[-1].transform.rotation.y = message['transforms'][i]['transform']['rotation']['y']
            self.t[-1].transform.rotation.z = message['transforms'][i]['transform']['rotation']['z']
            self.t[-1].transform.rotation.w = message['transforms'][i]['transform']['rotation']['w']
        self.br.sendTransform(self.t)


    def _start_receiving(self): 
        self._listener.subscribe(self._receive_message)


    def start(self):
        signal.signal(signal.SIGINT, lambda *x: self.stop())
        self._run_topic_pubsub()

    def stop(self):
        if self._ros_client is not None:
            self._ros_client.terminate()


if __name__ == '__main__':
    a=ROSCloudSubBridge(ip='83.212.96.15', port="8003", local_topic_name="/tf_static",remote_topic_name="/tf_static", message_type="tf2_msgs/TFMessage")
    a.start()

