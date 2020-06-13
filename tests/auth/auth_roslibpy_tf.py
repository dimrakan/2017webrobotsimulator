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

logging.basicConfig(level=logging.INFO)
global my
class ROSCloudSubBridge(object):

    def __init__(self, ip, port, local_topic_name, remote_topic_name, message_type, file):
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
        self._file = file

    def _run_topic_pubsub(self):
    	try:
            self._listener = Topic(self._ros_client, self._remote_topic_name, self._message_type)
            rospy.init_node('cloudsub', anonymous=True)
            self._ros_client.send_on_ready(Message(self._file))
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
        try:
           msg=TFMessage()
           msg1=TransformStamped()
           list_msg=[]
           for i in range(len(message['transforms'])) :
            msg1.header.seq=message['transforms'][i]['header']['seq']
            msg1.header.stamp.secs=message['transforms'][i]['header']['stamp']['secs']
            msg1.header.stamp.nsecs=message['transforms'][i]['header']['stamp']['nsecs']
            msg1.header.frame_id=message['transforms'][i]['header']['frame_id']
            msg1.child_frame_id=message['transforms'][i]['child_frame_id']
            msg1.transform.translation.x=message['transforms'][i]['transform']['translation']['x']
            msg1.transform.translation.y=message['transforms'][i]['transform']['translation']['y']
            msg1.transform.translation.z=message['transforms'][i]['transform']['translation']['z']
            msg1.transform.rotation.x=message['transforms'][i]['transform']['rotation']['x']
            msg1.transform.rotation.y=message['transforms'][i]['transform']['rotation']['y']
            msg1.transform.rotation.z=message['transforms'][i]['transform']['rotation']['z']
            msg1.transform.rotation.w=message['transforms'][i]['transform']['rotation']['w']
            list_msg.append(msg1)
           msg=TFMessage(list_msg)
           self._rosPub=rospy.Publisher(self._local_topic_name, TFMessage, queue_size=10)
           self._rosPub.publish(msg)
        except:
           print('Error')        
 


    def _start_receiving(self): 
        self._listener.subscribe(self._receive_message)

    def start(self):
        signal.signal(signal.SIGINT, lambda *x: self.stop())
        self._run_topic_pubsub()

    def stop(self):
        if self._ros_client is not None:
            self._ros_client.terminate()


if __name__ == '__main__':
    file={"op":"auth","mac":"8580033a66cf2f34872e9525f4c38aa757fc5ccc24e1902fffac14394cb29f9dac9550554a5aaa7ce643d9f2982255db33a0536a97bf314e2292c68fd5104caa","client":"10.32.0.1","dest":"83.212.96.15","rand":"bbb1","t":0.0,"level":"admin","end":12000.0}
    a=ROSCloudSubBridge(ip='83.212.96.15', port="8063", local_topic_name="/tf",remote_topic_name="/tf", message_type="tf2_msgs/TFMessage",file=file)
    a.start()

