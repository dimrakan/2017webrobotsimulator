Purpose
======
Roscloudbridge is a application to connect ROS on two remote machines through Rosbridge. 


Install
------

This packages must be installed in the local system.( rosbridge_pyclient (https://github.com/robotics-4-all/rosbridge_pyclient) && rosconversions_py (https://github.com/robotics-4-all/rosconversions_py) )

Download this folder and run the following command in a catkin workspace

    $ catkin build

Usage
---------------

Fill the following param file (config/try.yaml) with the appropriate values and run

    $ roslaunch roscloudbrigde bridge.launch
   

```yaml
# ROSbridge websocket server info
ip: 83.212.96.15
port: 8115
secret: ldsUQPz9Fnih0Kib
tf: False
# Topics to subscribe from remote ['remote_topic_type'].
# If you want to change the name in the local one ['remote_topic_name','local_topic_name']
sub_topics: [
                    ['/robot1/scan'],
                    ['/robot1/joint_states','/local/joint_states']
                    ]
# Topics to publish form local to cloud ['local_topic_name','remote_topic_name','topic_type'].
# If the remote_topic_name already exists in the cloud leave topic_type empty.
# If you want to keep the same topic_name in both remote and local leave local_topic_name empty.
pub_topics: [
                    ['/robot1/cmd_vel','/robot1/test','std_msgs/String'],
                    ['/robot1/mobile_base/commands/velocity'],
                    ['/local/rere','/robot1/cmd_vel_mux/input/teleop']  
                    ]
# Services to get in local.
remote_services: [
                    ['/rosapi/topics']
                    ]
# Action Servers to get in local.
remote_actions: [
                    ['/robot1/move_base']
                    ]
```         
