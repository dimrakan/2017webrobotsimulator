from __future__ import print_function

from roslibpy import Message, Ros, Topic
import rospy
from sensor_msgs.msg import PointCloud2,PointField
import logging
import atexit
import signal

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
        rospy.loginfo(rospy.get_caller_id() + " I heard a message of %s",self._message_type)
        rospy.loginfo(rospy.get_caller_id()  + " Time from previous message %s",(rospy.get_time()-my)  )#edw mou leei unicode type
        my=rospy.get_time() 
        try:
            msg=PointCloud2()
            msg_list=[]
            for i in range (len(message['fields'])):
             msg_list.append(PointField())
            msg.header.seq=message['header']['seq']
            msg.header.stamp.secs=message['header']['stamp']['secs']
            msg.header.stamp.nsecs=message['header']['stamp']['nsecs'] 
            msg.header.frame_id='camera_depth_optical_frame' #message['header']['frame_id']
            msg.height=message['height']
            msg.width=message['width']
            msg.is_dense=message['is_dense']
            msg.is_bigendian=message['is_bigendian']
            msg.row_step=message['row_step']
            msg.point_step=message['point_step']
            msg.data=message['data'].decode('base64')
            for i in range (len(message['fields'])):
             msg_list[i].name=message['fields'][i]['name']
             msg_list[i].offset=message['fields'][i]['offset']
             msg_list[i].datatype=message['fields'][i]['datatype']
             msg_list[i].count=message['fields'][i]['count']
            msg.fields=msg_list
            self._rosPub=rospy.Publisher(self._local_topic_name, PointCloud2, queue_size=10) #message type is String instead of msg_stds/String
            self._rosPub.publish(msg)
        except:
           print('Error')        

    def _start_receiving(self):
        self._listener.subscribe(self._receive_message)
        print('start')  

    
    def start(self):
        signal.signal(signal.SIGINT, lambda *x: self.stop())
        self._run_topic_pubsub()

    def stop(self):
        if self._ros_client is not None:
            self._ros_client.terminate()



if __name__ == '__main__':
    a=ROSCloudSubBridge(ip='83.212.96.15', port="8018", local_topic_name="/local/robot/camera/depth/points",remote_topic_name="/camera/depth/points", message_type="sensor_msgs/PointCloud2")
    a.start()
