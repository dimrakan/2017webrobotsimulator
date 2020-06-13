# 2017_web_robot_simulator

Purpose
=====
This is a web_robot_simulator platform running in the cloud for running robot simulations in the cloud. Users are able to develop robotics application by using cloud simulation's data.


Install
-----

After a Kubernetes Cluster is ready, by running the following commands the system is ready and running in the cloud.

   For the disk space the system requires:
    
     $ kubelet apply -f role.yaml
     $ kubelet apply -f pv.yaml

   For the flask server:
   
     $ kubelet apply -f flask.yaml
   
   For the mongoDB:
   
     $ kubelet apply -f mongo.yaml
     
     
