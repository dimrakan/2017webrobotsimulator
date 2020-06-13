#!/usr/bin/env python

from __future__ import print_function

from rosbridge_pyclient import Executor, ExecutorManager, ROSApi ,ActionClient, Goal 
import rospy
import logging
import atexit
import signal
import time
import actionlib
from rosconversions import dict_to_ros_msg, get_message_class , ros_msg_to_dict ,fill_ros_message
from .base import Base
import re

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class Bridge(Base):
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
        rospy.init_node(self.__class__.__name__, anonymous=True)

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
