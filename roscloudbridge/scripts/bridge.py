#!/usr/bin/env python

from __future__ import print_function

from rosbridge_pyclient import Executor, ExecutorManager, Publisher, Subscriber, ROSApi ,Goal,ServiceClient,ActionClient
import rospy
import logging
import atexit
import signal
import time
from rosconversions import dict_to_ros_msg, get_message_class,ros_msg_to_dict, ros_srv_req_to_dict, get_service_class,dict_to_ros_srv_response,fill_ros_message
import tf2_ros
from geometry_msgs.msg import TransformStamped
import re
import actionlib

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
class Bridge(object):
    """Abstract Bridge class"""
    def __init__(self, ip, port, secret=None, secret_fpath=None):
        self._ip = ip
        self._port = port
        self._secret = secret
        self._secret_fpath = secret_fpath
        signal.signal(signal.SIGINT, lambda *x: self.stop())
        self._manager = ExecutorManager()
        self._exec = Executor(ip=ip, port=port)
        self._exec.connect()
        self._manager.add(self._exec)
        self._rosapi = ROSApi(executor=self._exec)
#        rospy.init_node(self.__class__.__name__, anonymous=True)

    @property
    def rosApi(self):
        """Returns an instance of the ROSApi"""
        return self._rosapi

    def start_executor(self):
        """Start executor"""
        self._manager.start()

    def auth(self):
        """Perform authentication to remote ros env through rosbridge"""
        self._exec.authenticate(self._secret, self._secret_fpath)
   
    def stop(self):
        """Stop executor"""
        logger.debug("Shutting down proxy")
        self._manager.kill()
        rospy.signal_shutdown("Interrupted by User")


    def terminated(self):
        """Check whether or not all websocket connections are closed"""
        for ws in self._manager.websockets.itervalues():
            if not ws.terminated:
                return False
        return True

    def run_forever(self):
        """Keep main thread running, via iterative sleep operations"""
        self.run()
        while True:
            if self.terminated():
                break
            time.sleep(3)

    def run(self):
        try:
            self.start_executor()
            self._start_remote_endpoint()
            self._start_local_endpoint()
        except KeyboardInterrupt:
            self.stop()
 
    def _start_local_endpoint(self):
        raise NotImplementedError("")

    def _start_remote_endpoint(self):
        raise NotImplementedError("")



class RemoteTfBridge(Bridge):
    def __init__(self, ip, port, secret, remote_topic=None,
                 local_topic=None, topic_type=None,secret_fpath=None):
        Bridge.__init__(self, ip, port, secret,secret_fpath)
        self._remote_topic = '/tf'
        self._remote_topic_2 = '/tf_static'
        self._local_topic = local_topic
        if self._local_topic == None:
         self._local_topic=self._remote_topic  
        self._topic_type = topic_type
        self._local_ros_pub = None
        self._remote_ros_sub = None
        self._remote_ros_sub_2 = None
        self.br =tf2_ros.StaticTransformBroadcaster()
        self.t = [] 

    def _receive_message(self,message):
#        rospy.loginfo("Received message")
        if self._local_ros_pub is not None:
            msg=dict_to_ros_msg(self._topic_type, message)
            self._local_ros_pub.publish(msg)


    def _receive_message_2(self,message):
#        rospy.loginfo("Receive message")
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



    def _start_remote_endpoint(self):
        self.auth()
        if self._topic_type is None:
         self._topic_type = self.rosApi.get_topic_type(self._remote_topic)[1]
        self._remote_ros_sub = Subscriber(self._exec, self._remote_topic,
                                          self._topic_type,
                                          self._receive_message)
        self._remote_ros_sub_2 = Subscriber(self._exec, self._remote_topic_2,
                                          self._topic_type,
                                          self._receive_message_2)

    def _start_local_endpoint(self):
        self._local_ros_pub = rospy.Publisher(
            self._local_topic,
            get_message_class(self._topic_type), queue_size=10)

class RemoteSubscriberBridge(Bridge):
    def __init__(self, ip, port, remote_topic,secret=None, local_topic=None, topic_type=None ,secret_fpath=None):
        Bridge.__init__(self, ip, port, secret ,secret_fpath)
        self._remote_topic = remote_topic
        self._local_topic = local_topic
        if self._local_topic == None:
         self._local_topic=self._remote_topic  
        self._topic_type = topic_type
        self._local_ros_pub = None
        self._remote_ros_sub = None

    def _receive_message(self,message):
#        rospy.loginfo("Received message ")
        if self._local_ros_pub is not None:
            msg=dict_to_ros_msg(self._topic_type, message)
            self._local_ros_pub.publish(msg)

    def _start_remote_endpoint(self):
        self.auth() 
        if self._topic_type is None:
           try:
            self._topic_type = self.rosApi.get_topic_type(self._remote_topic)[1]
            if self._topic_type == "":
             print('Error : Remote topic '+self._remote_topic+' does not exist')
             rospy.logerr('Remote topic '+self._remote_topic+' does not exist')
             return
           except:
            print('Remote topic '+self._remote_topic+' is not available')  
            rospy.logerr('Remote topic '+self._remote_topic+' does not exist')
            return 
        self._remote_ros_sub = Subscriber(self._exec, self._remote_topic,
                                          self._topic_type,
                                          self._receive_message)
  

    def _start_local_endpoint(self):
        try:
         msg_class=get_message_class(self._topic_type)
         if msg_class==None : 
          e='Error : message class '+self._topic_type+' is not available in local machine'   
          print(e)
          rospy.logerr('message class '+self._topic_type+' is not available in local machine')
          return
        except:
         e='Error : local topic type of '+self._local_topic+' is not available'   
         print(e)
         rospy.logerr('message class '+self._topic_type+' is not available in local machine')
         return
        self._local_ros_pub = rospy.Publisher(self._local_topic,msg_class, queue_size=10)


class RemotePublisherBridge(Bridge):
    def __init__(self, ip, port, 
                 local_topic, topic_type=None,secret=None,remote_topic=None,secret_fpath=None):
        Bridge.__init__(self, ip, port, secret,secret_fpath)
        self._remote_topic = remote_topic
        self._local_topic = local_topic
        if self._remote_topic == None:
         self._remote_topic=self._local_topic  
        self._topic_type = topic_type
        self._local_ros_sub = None
        self._remote_ros_pub = None

    def _receive_message(self,message):
        rospy.loginfo("Sending message")
        if self._local_ros_sub is not None:
            msg=ros_msg_to_dict(message)
            self._remote_ros_pub.publish(msg)

    def _start_remote_endpoint(self):
        self.auth()
        if self._topic_type is None:
           try:
            self._topic_type = self.rosApi.get_topic_type(self._remote_topic)[1]
            if self._topic_type == "":
             print('Error : Remote topic '+self._remote_topic+' does not exist')
             rospy.logerr('Remote topic '+self._remote_topic+' does not exist')
             return
           except:
            print('Remote topic '+self._remote_topic+' is not available')  
            rospy.logerr('Remote topic '+self._remote_topic+' does not exist')
            return 
        self._remote_ros_pub = Publisher(self._exec, self._remote_topic,
                                          self._topic_type)
          

    def _start_local_endpoint(self):
        try:
         msg_class=get_message_class(self._topic_type)
         if msg_class==None : 
          e='Error: messagee class '+self._topic_type+' is not available in local machine'   
          print(e)
          rospy.logerr('messagee class '+self._topic_type+' is not available in local machine')
          return
        except:
         e='Error: local topic type of '+self._local_topic+' is not available'   
         rospy.logerr('messagee class '+self._topic_type+' is not available in local machine')
         return
        self._local_ros_sub = rospy.Subscriber(self._local_topic,msg_class, self._receive_message)


class RemoteServiceBridge(Bridge):
    def __init__(self, ip, port, remote_topic,local_topic=None, topic_type=None ,secret_fpath=None,secret=None):
        Bridge.__init__(self, ip, port, secret,secret_fpath)
        self._remote_topic = remote_topic
        self._local_topic = local_topic
        if self._local_topic == None:
         self._local_topic=self._remote_topic  
        self._service_type = topic_type
        self._local_ros_srv = None
        self._remote_ros_sub = None
        self._flag=False

    def _receive_message(self,status,message):
#        rospy.loginfo("Received message %s",message)
        self._resp=(dict_to_ros_srv_response(self._service_type,message))
        self._flag=True

    def _start_remote_endpoint(self):
        self.auth()
        if self._service_type is None:
           try:
             self._service_type = self.rosApi.get_service_type(self._remote_topic)[1]
             if self._service_type == "":
              print('Error : Remote service '+self._remote_topic+'does not exist')
              rospy.logerr('Remote service '+self._remote_topic+'does not exist')
              return
           except:
            print('Remote service '+self._remote_topic+' is not available')  
            rospy.logerr('Remote service '+self._remote_topic+'does not exist')
            return 
        self._srv = ServiceClient(self._exec, self._remote_topic, self._service_type)


    def _start_local_endpoint(self):
       try: 
        srv_class=get_service_class(self._service_type)
        if srv_class==None : 
         e='Error: service class '+self._service_type+' is not available in local machine'   
         print(e)
         rospy.logerr('service class '+self._service_type+' is not available in local machine')
       except:
         e='Error: local service type of '+self._local_topic+' is not available'   
         print(e)
         rospy.logerr('service class '+self._service_type+' is not available in local machine')
         return
       self._local_ros_srv = rospy.Service(self._local_topic,srv_class, self._callback)


    def _callback(self,req):
        self._srv.call(ros_srv_req_to_dict(req),self._receive_message) 
        while self._flag==False :
         time.sleep(1)
        self._flag=False
        return self._resp 

class RemoteActionBridge(Bridge):
    def __init__(self, ip, port, remote_topic, local_topic=None ,secret=None, topic_type=None ,secret_fpath=None):
        Bridge.__init__(self, ip, port, secret,secret_fpath)
        self._remote_topic = remote_topic
        self._local_topic = local_topic
        if self._local_topic == None:
         self._local_topic=self._remote_topic  
        self._topic_type = topic_type
        self._local_ros_pub = None
        self._remote_ros_sub = None

    def _receive_message(self,message):
#        rospy.loginfo("Received message ")
        if self._local_ros_pub is not None:
            msg=dict_to_ros_msg(self._topic_type, message)
            self._local_ros_pub.publish(msg)

    def _start_remote_endpoint(self):
        self.auth()
        self._remote_action=self._remote_topic 
        self._remote_feedback=self._remote_action+'/feedback'
        if self._topic_type is None:
         try:
          self._topic_type = self.rosApi.get_topic_type(self._remote_feedback)[1]
          if self._topic_type == "":
             print('Error : Remote action server '+self._remote_action+'does not exist')
             rospy.logerr('Remote action server '+self._remote_action+'does not exist')
             return
         except:
            print('Remote action server '+self._remote_action+' is not available')  
            rospy.logerr('Remote action server '+self._remote_action+'does not exist')
            return 
        self._action_type=re.search(r'([\s\S]*?)(ActionFeedback)',self._topic_type).group(1)
        self.ac = ActionClient(self._exec, self._remote_action, self._action_type)

    def _start_local_endpoint(self):
       try: 
        msg_class=get_message_class(self._action_type+'Action')
        if msg_class==None : 
         e='Error: message class '+self._action_type+'Action is not available in local machine'   
         print(e)
         rospy.logerr('message class '+self._action_type+'Action is not available in local machine')
         return  
        self._msg_class_feedback=get_message_class(self._action_type+'ActionFeedback') #self._topic_type.split('Action$
        if self._msg_class_feedback==None : 
         e='Error: message class '+self._action_type+'ActionFeedback is not available in local machine'   
         print(e)
         rospy.logerr('message class '+self._action_type+'ActionFeedback is not available in local machine')
         return
        self._msg_class_status=get_message_class('actionlib_msgs/GoalStatusArray') #self._topic_type.split('Action$
        if self._msg_class_status==None : 
         e='Error: message class actionlib_msgs/GoalStatusArray is not available in local machine'   
         print(e)
         rospy.logerr('message class actionlib_msgs/GoalStatusArray is not available in local machine')
         return
        self._msg_class_result=get_message_class(self._action_type+'ActionResult') #self._topic_type.split('Action$
        if self._msg_class_result==None : 
         e='Error: message class '+self._action_type+'ActionResult is not available in local machine'   
         print(e)
         rospy.logerr('message class '+self._action_type+'ActionResult is not available in local machine')
         return
       except:
        print('Error with action server '+self._remote_action)
        rospy.logerr('Action server '+self._remote_action+' is not available in local machine')
        return  
       self._as = actionlib.SimpleActionServer(self._remote_action, msg_class, execute_cb=self.execute_cb, auto_start = False)
       self._as.start()
       data={}  

    def execute_cb(self,goal):
        self.goal = Goal(ros_msg_to_dict(goal), on_result=self._on_result,
                         on_feedback=self._on_feedback,
                         on_status=self._on_status)
        self.id=self.ac.send_goal(self.goal)
        self.success=False
        self.preempt=False
        self.abort=False
        while not self.success:
          if self._as.is_preempt_requested(): 
             self.ac.cancel_goal(self.id) 
          if self.preempt :
             rospy.loginfo('%s: Preempted' % self.id)
             self.ac.cancel_goal(self.id) 
             self._as.set_preempted(self._result,self._text)
             break
          if self.abort:
             self._as.set_aborted(self._result,self._text) 
             break 
          if self.success:
        #     print(self._result)
             self._as.set_succeeded(self._result,self._text)
             break
        print('End of goal execution')

    def _on_result(self, data, status ,header):
#        d={'header': header,'status': status,'result': data}
        self._result=fill_ros_message(get_message_class(self._action_type+'Result')(),data) #
        self._text=status['text']
        if status['status']==4:
           print('R!Abort')
           self.abort=True
        elif status['status']==3:
           print('R!Goal Reached')
           self.success=True
        elif status['status']==1: 
           print('R!Active') 
        elif status['status']==2: 
           print('R!Preempted') 
           self.preempt=True



    def _on_feedback(self, data, status ,header):
         d={'header': header,
            'status': status ,
            'feedback': data
         }
         feedback=fill_ros_message(get_message_class(self._action_type+'Feedback')(),data)
         self._as.publish_feedback(feedback) 

    def _on_status(self, data ,header):
        d={'header': header,
            'status_list': data
        }
#        status=fill_ros_message(get_message_class('actionlib_msgs/GoalStatusArray')(),d)

class ROSBridge(object):

  def __init__(self):
   self._ip = rospy.get_param('~ip', None)
   if self._ip is None:
     rospy.logerr('No ip given.')
     raise Exception('No ip given.')
   self._port = rospy.get_param('~port', None)
   self._secret = rospy.get_param('~secret', None)
   self._tf=  rospy.get_param('~tf', None)
   rospy.loginfo("Will connect to ROSBridge websocket: ws://{}:{}".format(self._ip, self._port))
   self._remote_topics = rospy.get_param('~sub_topics', [])
   rospy.loginfo("Remote topics: " + str(self._remote_topics))
   self._local_topics = rospy.get_param('~pub_topics', [])
   rospy.loginfo("Local topics: " + str(self._local_topics))
   self._remote_services = rospy.get_param('~remote_services', [])
   rospy.loginfo("Remote services: " + str(self._remote_services))
   self._remote_actions = rospy.get_param('~remote_actions', [])
   rospy.loginfo("Remote actions: " + str(self._remote_actions))
   all=[]
   if self._tf:
    all.append(RemoteTfBridge(ip=self._ip,port=self._port,secret=self._secret))
   for r_t in self._remote_topics:
    remote_topic=r_t[0]
    if len(r_t) == 1:
     all.append(RemoteSubscriberBridge(ip=self._ip,port=self._port,secret=self._secret,remote_topic=remote_topic))
    elif len(r_t) == 2:
     local_topic=r_t[1]
     all.append(RemoteSubscriberBridge(ip=self._ip,port=self._port,secret=self._secret,remote_topic=remote_topic,local_topic=local_topic))
   for l_t in self._local_topics:
    local_topic=l_t[0]
    if len(l_t) == 1:
     topic_type=None
     remote_topic=None
    elif len(l_t) == 2:  
     remote_topic=l_t[1]
     topic_type=None
    elif len(l_t) == 3:
     remote_topic=l_t[1]
     topic_type=l_t[2]
    all.append(RemotePublisherBridge(ip=self._ip,port=self._port,secret=self._secret,local_topic=local_topic,remote_topic=remote_topic,topic_type=topic_type))
   for r_s in self._remote_services:
    remote_service=r_s[0]
    all.append(RemoteServiceBridge(ip=self._ip,port=self._port,secret=self._secret,remote_topic=remote_service))
   for r_a in self._remote_actions:
    remote_action=r_a[0]
    all.append(RemoteActionBridge(ip=self._ip,port=self._port,secret=self._secret,remote_topic=remote_action))
   for a in all: 
     a.run()


if __name__=='__main__':
   rospy.init_node('try') 
   r=ROSBridge() 

