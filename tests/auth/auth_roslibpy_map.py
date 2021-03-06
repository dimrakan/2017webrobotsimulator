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
from nav_msgs.msg import OccupancyGrid
import time

logging.basicConfig(level=logging.INFO)
global my
class ROSCloudSubBridge(object):

    def __init__(self, ip, port, local_topic_name, remote_topic_name, message_type,file):
        self._ip = ip
        self._port = port
        self._local_topic_name = local_topic_name     
        self._remote_topic_name = remote_topic_name
        self._message_type = message_type
        self._ros_client= Ros(self._ip,self._port) 
        self._rosPub = None  # Should be specified by the automation engine
        self._listener = None  # Should be specified by the automation engine
        self._file = file
        self._logger = logging.getLogger(self.__class__.__name__)
        self._logger.setLevel(logging.DEBUG)

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
            msg=OccupancyGrid()
            msg.header.seq=message['header']['seq']
            msg.header.stamp=rospy.Time.now() #.secs=message['header']['stamp']['secs']
            msg.header.frame_id=message['header']['frame_id'] #type unicode
            msg.info.map_load_time.secs=message['info']['map_load_time']['secs']
            msg.info.map_load_time.nsecs=message['info']['map_load_time']['nsecs']
            msg.info.resolution=message['info']['resolution']
            msg.info.width=message['info']['width']
            msg.info.height=message['info']['height']
            msg.info.origin.position.x=message['info']['origin']['position']['x']
            msg.info.origin.position.y=message['info']['origin']['position']['y']
            msg.info.origin.position.z=message['info']['origin']['position']['z']
            msg.info.origin.orientation.x=message['info']['origin']['orientation']['x']
            msg.info.origin.orientation.y=message['info']['origin']['orientation']['y']
            msg.info.origin.orientation.z=message['info']['origin']['orientation']['z']
            msg.info.origin.orientation.w=message['info']['origin']['orientation']['w']
            msg.data=message['data']
            self._rosPub=rospy.Publisher(self._local_topic_name, OccupancyGrid, queue_size=10)
            self._rosPub.publish(msg)        
            time.sleep(3)
            self.start()
              
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
    a=ROSCloudSubBridge(ip='83.212.96.15', port="8063", local_topic_name="robot1/map",remote_topic_name="robot1/map", message_type="nav_msgs/OccupancyGrid",file=file)
    a.start()

