if __name__ == '__main__':
    import logging
    from roslibpy import *
    from roslibpy.tf import TFClient
    import rospy
    import tf
    import tf2_ros
    from geometry_msgs.msg import TransformStamped 

    FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
    logging.basicConfig(level=logging.DEBUG, format=FORMAT)

    ros_client = Ros('83.212.96.15', 8018)
    rospy.init_node('turtle_tf_broadcaster')

    def run_tf_example():
        tfclient = TFClient(ros_client, fixed_frame='camera_depth_frame',
                            angular_threshold=0.00, translation_threshold=0.00)

        tfclient.subscribe('camera_depth_optical_frame', callback)

    def callback(msg):
          br = tf2_ros.TransformBroadcaster()
          t = TransformStamped()
          t.header.stamp = rospy.Time.now()
          t.header.frame_id = "camera_depth_frame"
          t.child_frame_id = 'camera_depth_optical_frame'
          t.transform.translation.x = msg['translation']['x']
          t.transform.translation.y = msg['translation']['y']
          t.transform.translation.z = msg['translation']['z']
          t.transform.rotation.x = msg['rotation']['x']
          t.transform.rotation.y = msg['rotation']['y']
          t.transform.rotation.z = msg['rotation']['z']
          t.transform.rotation.w = msg['rotation']['w']
          br.sendTransform(t)
          print("send")

    run_tf_example()
    ros_client.run_event_loop()
