#!/usr/bin/env python

from __future__ import print_function

from rosbridge_pyclient import Executor, ExecutorManager, Publisher, Subscriber, ROSApi ,ServiceClient,ActionClient
import rospy
import logging
import atexit
import signal
import time
from rosconversions import ros_srv_req_to_dict, get_service_class,dict_to_ros_srv_response
from .base import Base

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
        self._resp=None
        self._flag=False  
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


class RemotePublisherBridge(Bridge):
    def __init__(self, ip, port, secret, remote_topic,local_topic, topic_type=None ,secret_fpath=None):
        Bridge.__init__(self, ip, port, secret,secret_fpath)
        self._remote_topic = remote_topic
        self._local_topic = local_topic
        if self._local_topic == None:
         self._local_topic=self._remote_topic  
        self._service_type = topic_type
        self._local_ros_srv = None
        self._remote_ros_sub = None

    def _receive_message(self,status,message):
        rospy.loginfo("Received message %s",message)
        self._resp=(dict_to_ros_srv_response(self._service_type,message))
        self._flag=True

    def _start_remote_endpoint(self):
        self.auth()
        if self._service_type is None:
         self._service_type = self.rosApi.get_service_type(self._remote_topic)[1]
         self._srv = ServiceClient(self._exec, self._remote_topic, self._service_type)


    def _start_local_endpoint(self):
        srv_class=get_service_class(self._service_type)
        if srv_class==None : 
         e='Error: service class '+self._service_type+' is not available in local machine'   
         self.stop()
         print(e)
        else : 
         self._local_ros_srv = rospy.Service(self._local_topic,srv_class, self._callback)

    def _callback(self,req):
        self._srv.call(ros_srv_req_to_dict(req),self._receive_message) 
        while self._flag==False :
         time.sleep(1)
        self._flag=False
        return self._resp 
