#!/usr/bin/env python

from __future__ import print_function

from roslibpy import Message, Ros, Topic
import rospy
import logging
import atexit
import signal
from bond.msg import Status
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

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
           try:
            msg=LaserScan()
            msg.header.seq=message['header']['seq']
            msg.header.stamp.secs=message['header']['stamp']['secs']
            msg.header.stamp.nsecs=message['header']['stamp']['nsecs']
            msg.header.frame_id=message['header']['frame_id']
            msg.angle_min=message['angle_min'] 
            msg.angle_max=message['angle_max'] 
            msg.angle_increment=message['angle_increment']
            msg.time_increment=message['time_increment'] 
            msg.scan_time=message['scan_time']
            msg.range_min=message['range_min']
            msg.range_max=message['range_max']
            ranges_list=[]
            intensities_list=[] 
            for i in range(len(message['ranges'])):
             ranges_list.append(float)
             if message['ranges'][i]==None:
              ranges_list[i]=float('NaN')
             else:
              ranges_list[i]=message['ranges'][i]
            for i in range(len(message['intensities'])):
             intensities_list.append(float)
             if message['intensities'][i]==None:
              intensities_list[i]=float('NaN')
             else:
              intensities_list[i]=message['intensities'][i]
            msg.ranges=ranges_list
            msg.intensities=intensities_list
            self._rosPub=rospy.Publisher(self._local_topic_name, LaserScan, queue_size=10)
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
    a=ROSCloudSubBridge(ip='83.212.96.15', port="8018", local_topic_name="/local/scan",remote_topic_name="/scan", message_type="sensor_msgs/LaserScan")
    a.start()


