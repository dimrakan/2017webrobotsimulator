from flask import Flask, render_template, request, redirect ,make_response ,url_for ,g ,jsonify ,json,send_file
import datetime
import logging
import random,string
import connexion
from connexion import NoContent
from passlib.apps import custom_app_context as pwd_context
import jwt
from functools import wraps
from urllib.parse import urlparse
import time
import atexit
from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.triggers.interval import IntervalTrigger
from werkzeug import secure_filename
import os
import glob
from kubernetes import client as kube_client
from kubernetes import config
from kubernetes.client import configuration
from kubernetes.client.rest import ApiException
from pymongo import MongoClient,ASCENDING, DESCENDING
from passlib.apps import custom_app_context as pwd_context
import threading
import bson.binary
from io import StringIO,BytesIO
import gridfs

client = MongoClient('mongodb://mongodb-service/')
db = client.flask
mydb = db.flask
configuration.assert_hostname = False
config.load_incluster_config()
api = kube_client.CoreV1Api()
db1 = client.gridfs_example
fs = gridfs.GridFS(db1)




def hash_password(password):
        return pwd_context.encrypt(password)

def verify_password(password1,password2):
        return pwd_context.verify(password1,password2)

def check_pods(username) -> str:  
   try:
    status1=api.read_namespaced_pod(username+'pod1','default').status.phase
   except:
    status1="Not found"
   try:
    status2=api.read_namespaced_pod(username+'pod2','default').status.phase
   except:
    status2='Not found'
   try:
    status3=api.read_namespaced_pod(username+'pod3','default').status.phase
   except:
    status3='Not found'  
   return status1,status2,status3

def is_inactive():
    q=mydb.users.find({'login':{'$ne':None}})
    for i in range(q.count()):
     print (q[i]["username"])
     secs=(datetime.datetime.utcnow()-q[i]["login"]).total_seconds()
     print (secs)
     print ("!~~~~~~~~~~~~~~~!")  
     if secs > 3000:
          body=kube_client.V1DeleteOptions()
          po1=None
          try:
           po1=api.read_namespaced_pod(q[i]['username']+'pod1','default')
          except ApiException as e:
           if e.status != 404:
            print("Unknown error: %s" % e)
            exit(1)
          if po1 :
            api.delete_namespaced_pod(q[i]['username']+'pod1','default',body)
          po2=None
          try:
           po2=api.read_namespaced_pod(q[i]['username']+'pod2','default')
          except ApiException as e:
            if e.status != 404:
              print("Unknown error: %s" % e)
              exit(1)
          if po2 :
            api.delete_namespaced_pod(q[i]['username']+'pod2','default',body)
          po3=None
          try:
           po3=api.read_namespaced_pod(q[i]['username']+'pod3','default')
          except ApiException as e:
            if e.status != 404:
              print("Unknown error: %s" % e)
              exit(1)
          if po3 :
            api.delete_namespaced_pod(q[i]['username']+'pod3','default',body)
          svc1=None
          try:
           svc1 = api.read_namespaced_service(q[i]['username']+'pod1','default')
          except ApiException as e:
           if e.status != 404:
             print("Unknown error: %s" % e)
             exit(1)
          if svc1 :
             api.delete_namespaced_service(q[i]['username']+'pod1','default')
          svc2=None
          try:
            svc2 = api.read_namespaced_service(q[i]['username']+'pod2','default')
          except ApiException as e:
           if e.status != 404:
            print("Unknown error: %s" % e)
            exit(1)
          if svc2 :
            api.delete_namespaced_service(q[i]['username']+'pod2','default')
          svc3=None
          try:
              svc3 = api.read_namespaced_service(q[i]['username']+'pod3','default')
          except ApiException as e:
            if e.status != 404:
             print("Unknown error: %s" % e)
             exit(1)
          if svc3 :
           api.delete_namespaced_service(q[i]['username']+'pod3','default')
          filelist=glob.glob("upload/"+q[i]["username"]+"/*")
          for file in filelist:
           os.remove(file)
          mydb.users.update_one({'username':q[i]["username"]},{'$set':{"login":None}})
          mydb.users.update_one({'username':q[i]["username"]},{'$set':{"filename":None}})
          mydb.users.update_one({'username':username},{'$set':{username+'pod1out':None}})
          mydb.users.update_one({'username':username},{'$set':{username+'pod2out':None}})
          mydb.users.update_one({'username':username},{'$set':{username+'pod3out':None}})
          mydb.users.update_one({'username':username},{'$set':{username+'pod1list':None}})
          mydb.users.update_one({'username':username},{'$set':{username+'pod2list':None}})
          mydb.users.update_one({'username':username},{'$set':{username+'pod3list':None}})

          print (q[i]["login"]) 
     else:
      print ('keep going')
    return ('Done')

scheduler = BackgroundScheduler()
scheduler.start()
scheduler.add_job(
func=is_inactive,
trigger=IntervalTrigger(seconds=60),
id='is_inactive_job',
name='Check if any gzweb is running on background',
replace_existing=True)
atexit.register(lambda: scheduler.shutdown())



def token_required(f):
   @wraps(f)
   def decorated(*args,**kwargs):
      token=request.args.get('token')
      if not token:
         return jsonify({'message':'Token is missing'}),403
      try:
        data = jwt.decode(token,"sadfaf") 
        if data['user'] != kwargs["username"]: 
          return jsonify({'message':'Wrong user'})
      except:
        return jsonify({'message':'Token is invalid'}),403
      return f(*args,**kwargs)
   return decorated


@token_required
def upload(username) -> str:
    q=mydb.users.find_one({"username":username})
    f = request.files['file']
    if (f.filename==''):
      msg='No file selected'
      #data=list_files(username)
      return make_response(render_template("upload.html",username=username,msg=msg))
    try: 
     f.save(os.path.join('upload/'+username+"/"+secure_filename(f.filename)))
     mydb.users.update_one({'username':username},{'$set':{'filename':secure_filename(f.filename).split('.')[0]}})    
     msg="File successfully uploaded"
     return make_response(render_template("upload.html",username=username,msg=msg))
    except:
     mydb.users.update_one({'username':username},{'$set':{'filename':None}})
     msg="File have not been uploaded,try again" 
     return make_response(render_template("upload.html",username=username,msg=msg))
    return make_response(render_template("upload.html",username=username,msg=msg,Lfiles=data))
    #msg="File successfully uploaded"
    #data=list_files(username)
    #a=mydb.users.find_one({'username':username,'pod1list':{'$ne':None}})
    #if a :
    # tlist=a['pod1list']######automate
    #else:
    # tlist=[]
    #return make_response(render_template("upload.html",username=username,Lfiles=data,Kfiles=tlist,msg=msg))

 

def download(username,filename) -> str:
        q=mydb.users.find_one({"username":username})
        cursor1 = fs.find_one({'username':username,'filename':filename})
        if cursor1==None :
          return 'File not available',(404)
        cursor=fs.get(cursor1._id)
        outputdata =cursor.read()
        outfilename=cursor1.filename
        output= open(outfilename,"wb")
        output.write(outputdata)
        #fs.delete(cursor1._id) 
        return send_file('/usr/src/app/'+outfilename,as_attachment='True')


@token_required
def remove_upload(username) -> str:
   filelist=glob.glob("upload/"+username+"/*")
   for file in filelist:
     os.remove(file)  
   return "Done"
   
@token_required
def build_package(username,pod) -> str: #add filename###############
    q=mydb.users.find_one({"username":username})
    out=""
    Lfiles=[] 
    status1,status2,status3=check_pods(username)
    if pod=='1' :
     name='pod1'
     podpsw=q["pass1"]
     if status1!='Running':
       return 'Simulation Environment 1 is not running',(403)
    elif pod=='2' :
     name='pod2'
     podpsw=q['pass2']
     if status2!='Running':
       return 'Simulation Environment 2 is not running',(403)
    elif pod=='3' :
     name='pod3'
     podpsw=q['pass3']
     if status3!='Running':
       return 'Simulation Environment 3 is not running',(403)
    else :
       return "Simulation Environment not valid",(403)
    try:
     filename1=os.listdir('upload/'+username)[0]
    except:
     msg="Upload a zip file first pls" 
     msg="Package has not been build check console for info"
     Lfiles.append(msg)
     Lfiles.append(out)
     print('No file uploaded')
     return Lfiles
    filename=os.path.splitext(filename1)[0] 
    cmd_mkdir='mkdir -p /root/'+filename+'/src'
    command=['bin/bash','-c',cmd_mkdir]
    api_response = api.connect_get_namespaced_pod_exec(username+name,'default',command=command,stderr='True')
    cmd_unzip='cd /home/'+username+' && unzip '+filename1+' -d /root/'+filename+'/src/'
    command=['bin/bash','-c',cmd_unzip]
    api_response = api.connect_get_namespaced_pod_exec(username+name,'default',command=command,stderr='True')
    cmd_build='source /opt/ros/kinetic/setup.bash && cd /root/'+filename+' && catkin build'
    command=['bin/bash','-c',cmd_build]
    api_response = api.connect_get_namespaced_pod_exec(username+name,'default',command=command,stderr='True',stdout='True')
    out=api_response
    mydb.users.update_one({'username':username},{'$set':{username+name+'out':out}})
    cmd_lfiles='source /root/'+filename+'/devel/setup.bash && roscd '+filename+'_gazebo/launch && ls *.launch'
    command=['bin/bash','-c',cmd_lfiles]
    api_response = api.connect_get_namespaced_pod_exec(username+name,'default',command=command,stderr='True',stdout='True')
    print(api_response)
    Lfiles=api_response.split()
    if Lfiles[0]=='Error' or Lfiles[0]=='bash:':
      Lfiles=[]
    mydb.users.update_one({'username':username},{'$set':{username+name+'list':Lfiles}})     
    cmd_assets='mkdir /root/gzweb/http/client/assets/'+filename+'_description && source /root/'+filename+'/devel/setup.bash && roscd '+filename+'_description && cp -R /meshes /root/gzweb/http/client/assets/'+filename+'_description/'
    command=['bin/bash','-c',cmd_assets]
    api_response = api.connect_get_namespaced_pod_exec(username+name,'default',command=command,stderr='True',stdout='True')
    cmd_gazmodels='source /root/'+filename+'/devel/setup.bash && roscd '+filename+'_description && cd .. && cp -R '+filename+'_description ~/.gazebo/models/'
    command=['bin/bash','-c',cmd_gazmodels]
    try:
     api_response = api.connect_get_namespaced_pod_exec(name,username,command=command,stderr='True',stdout='True')
     print(api_response) 
    except:
     print ('error gazebo models')
    msg="File has successfully builded"
    Lfiles.append(msg)
    Lfiles.append(out)
    filelist=glob.glob("upload/"+username+"/*")
    for file in filelist:
     os.remove(file)
    return Lfiles
    
    
    
@token_required
def launch_gazebo(username,pod,robot) -> str:
    q=mydb.users.find_one({"username":username})
    o=urlparse(request.host_url).hostname
    status1,status2,status3=check_pods(username)
    print(pod)
    if pod=='1' :
     name='pod1'
     podpsw=q["pass1"]
     if status1!='Running':
       return 'Simulation Environment 1 is not running',(403) 
    elif pod=='2' :
     name='pod2'
     podpsw=q['pass2'] 
     if status2!='Running':
       return 'Simulation Environment 2 is not running',(403) 
    elif pod=='3' :
     name='pod3'
     podpsw=q['pass3']
     if status3!='Running':
      return 'Simulation Environment 3 is not running',(403) 
    else :
       return "Simulation Environment not valid",(403)     
    secret=''.join([random.choice(string.ascii_letters + string.digits) for n in range(16)])
    print(secret)
    cmd_sec="echo '"+secret+"' > /root/rosauth/src/rosauth/test/valid.secret "
    command=['bin/bash','-c',cmd_sec]
    api_response = api.connect_get_namespaced_pod_exec(username+name,'default',command=command,stderr='True')
    cmd_display="export DISPLAY=:1.0"
    cmd_gzweb=cmd_display+' && ./root/gzweb/start_gzweb.sh '+username+' '+podpsw
    command_gzweb=['bin/bash','-c',cmd_gzweb]
    cmd_kinetic="source /opt/ros/kinetic/setup.bash"
    if robot["map"]!="-1" : 
     number=robot["number"]
     map=robot["map"] 
     robotname=[]
     base=[]
     a3d_sensor=[]
     lidar_sensor=[]
     pose_x=[]
     pose_y=[]
     pose_z=[]
     pose=[]
     for i in range(int(number)) :   
        robotname.append(robot["robots"][i]["robotname"])
        base.append(robot["robots"][i]["base"])
        a3d_sensor.append(robot["robots"][i]["a3d_sensor"])
        lidar_sensor.append(robot["robots"][i]["lidar_sensor"])
        pose_x.append(robot["robots"][i]["pose_x"])
        pose_y.append(robot["robots"][i]["pose_y"])
        pose_z.append(robot["robots"][i]["pose_z"])
        pose_str="-x "+pose_x[i]+" -y "+pose_y[i]+" -z "+pose_z[i]
        pose.append(pose_str)
     cmd_world="export TURTLEBOT_GAZEBO_WORLD_FILE=/opt/ros/kinetic/share/turtlebot_gazebo/worlds/"#/usr/share/gazebo-7/worlds/"
     cmd_launch_multi="roslaunch turtlebot_gazebo new_turtlebot_multi_world.launch"
     cmd_gmapping="roslaunch --wait turtlebot_gazebo new_gmapping.launch"
     cmd_form=""
     cmd_new_multi="<launch>"
     cmd_new_gmap='<launch> <arg name="custom_gmapping_launch_file" default="$(find turtlebot_navigation)/launch/includes/gmapping/new_gmapping.launch.xml"/>'
     gmap='False'
     for i in range(int(number)): 
      cmd_robotname="export ROBOT"+str(i)+"_NAME="
      if a3d_sensor[i]=='none' and lidar_sensor[i]=='none':
        cmd_sensor_type="export ROBOT"+str(i)+"_SENSOR_TYPE=no_sensor"
        cmd_sensor="export ROBOT"+str(i)+"_SENSOR=none"   ##OR TYPE_SENSOR
      elif a3d_sensor[i]=='none' :
        cmd_sensor_type="export ROBOT"+str(i)+"_SENSOR_TYPE=lidar_sensor"  ##OR TYPE_SENSOR
        cmd_sensor="export ROBOT"+str(i)+"_LIDAR_SENSOR="+lidar_sensor[i]
        cmd_new_gmap=cmd_new_gmap+'  <include file=\"$(arg custom_gmapping_launch_file)\">   <arg name=\"base_frame\" value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')_tf/base_footprint\"/>   <arg name=\"odom_frame\" value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')_tf/odom\"/>   <arg name=\"scan_topic\" value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')/scan\"/>   <arg name=\"node_name\" value=\"gmap_$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')\"/>   <arg name=\"map_frame\" value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')_tf/map\"/>   <arg name=\"model\" value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')\"/>  </include>'
        gmap='True'
      elif lidar_sensor[i]=='none' :
        cmd_sensor_type="export ROBOT"+str(i)+"_SENSOR_TYPE=3d_sensor"  ##OR TYPE_SENSOR
        cmd_sensor="export ROBOT"+str(i)+"_3D_SENSOR="+a3d_sensor[i]
      else :
        cmd_sensor_type="export ROBOT"+str(i)+"_SENSOR_TYPE=multi_sensor"  ##OR TYPE_SENSOR
        cmd_sensor="export ROBOT"+str(i)+"_3D_SENSOR="+a3d_sensor[i]
        cmd_sensor="export ROBOT"+str(i)+"_LIDAR_SENSOR="+lidar_sensor[i]
        cmd_new_gmap=cmd_new_gmap+'  <include file=\"$(arg custom_gmapping_launch_file)\">   <arg name=\"base_frame\" value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')_tf/base_footprint\"/>   <arg name=\"odom_frame\" value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')_tf/odom\"/>   <arg name=\"scan_topic\" value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')/scan\"/>   <arg name=\"node_name\" value=\"gmap_$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')\"/>   <arg name=\"map_frame\" value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')_tf/map\"/>   <arg name=\"model\" value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')\"/>  </include>'
        gmap='True'
      cmd_base="export ROBOT"+str(i)+"_BASE="
      if base[i]=="kobuki" :
        cmd_stacks="export ROBOT"+str(i)+"_STACKS=hexagons"
      elif base[i]=="create" :
        cmd_stacks="export ROBOT"+str(i)+"_STACKS=circles"
      else :
        print ("Error,not supported base")
        return "Error,not supported base" ,(403)
      cmd_pose="export ROBOT"+str(i)+"_POSE="
      cmd_form=cmd_form+" && "+cmd_robotname+robotname[i]+" && "+cmd_sensor_type+" && "+cmd_sensor+" && "+cmd_base+base[i]+" && "+cmd_stacks+" && "+ cmd_pose+"'"+pose[i]+"'"
      cmd_new_multi=cmd_new_multi+' <group ns=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')\"> <param name=\"tf_prefix\" value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')_tf\" /> <include file=\"$(find turtlebot_gazebo)/launch/$(optenv ROBOT'+str(i)+'_BASE kobuki)_$(optenv ROBOT'+str(i)+'_SENSOR_TYPE 3d_sensor)_single_robot.launch\" > <arg name=\"init_pose\" value=\"$(optenv ROBOT'+str(i)+'_POSE -x 4 -y -1 -z 0 -Y 3.14)\" /> <arg name=\"robot_name\"  value=\"$(optenv ROBOT'+str(i)+'_NAME Robot'+str(i)+')\" /> <arg name=\"base\" value=\"$(optenv ROBOT'+str(i)+'_BASE kobuki)\"/>  <arg name=\"stacks\" value=\"$(optenv ROBOT'+str(i)+'_STACKS hexagons)\"/> <arg name=\"3d_sensor\" value=\"$(optenv ROBOT'+str(i)+'_3D_SENSOR kinect)\"/> <arg name=\"lidar_sensor\" value=\"$(optenv ROBOT'+str(i)+'_LIDAR_SENSOR hokuyo)\"/>  </include> </group> '
     cmd_edit="echo '"+cmd_new_multi+"</launch>' > /opt/ros/kinetic/share/turtlebot_gazebo/launch/new_multi_robot.launch "
     command=['bin/bash','-c',cmd_edit]
     api_response = api.connect_get_namespaced_pod_exec(username+name,'default',command=command,stderr='True')
     print('edit'+api_response)
     cmd=cmd_kinetic+' && '+cmd_display+cmd_form+' && '+cmd_world+map+'.world && '+cmd_launch_multi
     command=['bin/bash','-c',cmd]
     threading.Thread(target=thread_run,args=(username+name,'default',command)).start()
     print('launch') 
     if gmap=='True':
      cmd_edit="echo '"+cmd_new_gmap+"</launch>' > /opt/ros/kinetic/share/turtlebot_gazebo/launch/new_gmapping.launch "
      command=['bin/bash','-c',cmd_edit]
      api_response = api.connect_get_namespaced_pod_exec(username+name,'default',command=command,stderr='True')
      print('edit'+api_response)
      cmd=cmd_kinetic+' && '+cmd_display+cmd_form+' && '+cmd_gmapping
      command=['bin/bash','-c',cmd]
      threading.Thread(target=thread_run,args=(username+name,'default',command)).start()
      print('gmap')
    elif robot["map"]=="-1" : 
     print("YOURFILE")
     filename=q['filename']
     print(filename)
     cmd=cmd_kinetic+' && source /root/'+filename+'/devel/setup.bash && export DISPLAY=:1.0  &&  roslaunch '+filename+'_gazebo  '+robot["number"]
     command=['bin/bash','-c',cmd]
     threading.Thread(target=thread_run,args=(username+name,'default',command)).start()
    command_rosbridge=cmd_kinetic+' && '+cmd_display+' && source /root/rosauth/devel/setup.bash && roslaunch --wait rosbridge_server rosbridge_websocket.launch authenticate:=true'
    command=['bin/bash','-c',command_rosbridge]
    threading.Thread(target=thread_run,args=(username+name,'default',command)).start()
    print ('rosbridge')
    command_web_video_server=cmd_kinetic+' && '+cmd_display+' && rosrun web_video_server web_video_server  _port:=8081 '
    command=['bin/bash','-c',command_web_video_server]
    threading.Thread(target=thread_run,args=(username+name,'default',command)).start()
    o=urlparse(request.host_url).hostname
    command=command_gzweb
    threading.Thread(target=thread_run,args=(username+name,'default',command)).start()
    print('gzweb')
    token=request.args.get('token')
    if pod=='1' :
     pod_port=str(q['port_gzclient'])
     name='pod1'
    elif pod=='2' :
     pod_port=str(q['port_gzclient']+4)
     name='pod2'
    elif pod=='3' :
     pod_port=str(q['port_gzclient']+8)
     name='pod3'
    else :
     return ('Invalid Option')
    gzurl="http://"+o+":"+pod_port+"?username="+username+"&pod="+pod+"&token="+token
    return gzurl   


def thread_run(name,username,command) -> str:
     api_repsonse=api.connect_get_namespaced_pod_exec(name,username,command=command,stderr='True')
     return ('done')    


def run_gzweb(username,pod) -> str:
    q=mydb.users.find_one({"username":username}) 
    o=urlparse(request.host_url).hostname
    if pod=='1' :
     pod_port=str(q['port_gzclient'])
     name='pod1'
     podpsw=q['pass1']
    elif pod=='2' :
     pod_port=str(q['port_gzclient']+4)
     name='pod2'
     podpsw=q['pass2']
    elif pod=='3' :
     pod_port=str(q['port_gzclient']+8)
     name='pod3'
     podpsw=q['pass3']
    else :
     return ('Invalid Option'),(403)
    cmd_display="export DISPLAY=:1.0"
    cmd_gzweb=cmd_display+' && ./root/gzweb/start_gzweb.sh '+username+' '+podpsw
    command_gzweb=['bin/bash','-c',cmd_gzweb]
    command=command_gzweb
    threading.Thread(target=thread_run,args=(username+name,'default',command)).start()
    token=request.args.get('token')
    gzurl="http://"+o+":"+pod_port+"?username="+username+"&pod="+pod+"&token="+token
    return gzurl 


@token_required
def logout_user(username) -> str:
    body=kube_client.V1DeleteOptions()
    po1=None
    try:
     po1=api.read_namespaced_pod(username+'pod1','default')
    except ApiException as e:
       if e.status != 404:
           print("Unknown error: %s" % e)
           exit(1)
    if po1 :
      api.delete_namespaced_pod(username+'pod1','default',body)
    po2=None
    try:
     po2=api.read_namespaced_pod(username+'pod2','default')
    except ApiException as e:
       if e.status != 404:
           print("Unknown error: %s" % e)
           exit(1)
    if po2 :
      api.delete_namespaced_pod(username+'pod2','default',body)
    po3=None
    try:
     po3=api.read_namespaced_pod(username+'pod3','default')
    except ApiException as e:
       if e.status != 404:
           print("Unknown error: %s" % e)
           exit(1)
    if po3 :
      api.delete_namespaced_pod(username+'pod3','default',body)
    svc1=None
    try:
       svc1 = api.read_namespaced_service(username+'pod1','default')
    except ApiException as e:
       if e.status != 404:
           print("Unknown error: %s" % e)
           exit(1)
    if svc1 :
        api.delete_namespaced_service(username+'pod1','default')
    svc2=None
    try:
       svc2 = api.read_namespaced_service(username+'pod2','default')
    except ApiException as e:
       if e.status != 404:
           print("Unknown error: %s" % e)
           exit(1)
    if svc2 :
     api.delete_namespaced_service(username+'pod2','default')
    svc3=None
    try:
       svc3 = api.read_namespaced_service(username+'pod3','default')
    except ApiException as e:
       if e.status != 404:
           print("Unknown error: %s" % e)
           exit(1)
    if svc3 :
       api.delete_namespaced_service(username+'pod3','default')
    filelist=glob.glob("upload/"+username+"/*")
    for file in filelist:
     os.remove(file)
    o=urlparse(request.host_url).hostname
    q=mydb.users.find_one({"username":username})
    mydb.users.update_one({'username':username},{'$set':{"login":None,'pass1':None,'pass2':None,'pass3':None}})
    mydb.users.update_one({'username':username},{'$set':{username+'pod1out':None}})
    mydb.users.update_one({'username':username},{'$set':{username+'pod2out':None}})
    mydb.users.update_one({'username':username},{'$set':{username+'pod3out':None}})
    mydb.users.update_one({'username':username},{'$set':{username+'pod1list':None}})
    mydb.users.update_one({'username':username},{'$set':{username+'pod2list':None}})
    mydb.users.update_one({'username':username},{'$set':{username+'pod3list':None}})
    return make_response(render_template("login.html"))




@token_required
def update_time(username) -> str:
    k=mydb.users.find_one({"username":username}) 
    secs=datetime.datetime.utcnow()-k["login"]
    print("updated")
    mydb.users.update_one({'username':username},{'$set':{"login":datetime.datetime.utcnow()}}) 
    return 'updated'

@token_required
def get_user_custom(username) -> str:
   return make_response(render_template("custom.html",username=username))

@token_required
def get_files(username) -> str:#remove
   data=list_files(username)
   return make_response(render_template("files.html",username=username,Lfiles=data))

def list_files(username) -> str:#remove
  data=[]
  for grid_out in fs.find({"username": username},no_cursor_timeout=True):
    data.append(grid_out.filename)
  return data


@token_required
def get_user_packages(username) -> str:
   data=list_files(username)
   a=mydb.users.find_one({'username':username})
   list1=a[username+'pod1list']
   list2=a[username+'pod2list']
   list3=a[username+'pod3list']
   out1=a[username+'pod1out']
   out2=a[username+'pod2out']
   out3=a[username+'pod3out']
   print(list1)
   print(out1)
   return make_response(render_template("upload.html",username=username))
    


@token_required
def get_user_pods(username) -> str:
   k=mydb.users.find_one({"username":username})
   pass1=k["pass1"]
   pass2=k["pass2"]
   pass3=k["pass3"]
   try:
    status1=api.read_namespaced_pod(username+'pod1','default').status.phase
   except:
    status1="Not found"
   try:
    status2=api.read_namespaced_pod(username+'pod2','default').status.phase
   except:
    status2='Not found'
   try:
    status3=api.read_namespaced_pod(username+'pod3','default').status.phase
   except:
    status3='Not found'  
   return make_response(render_template("pods.html",username=username,status1=status1,status2=status2,status3=status3,pass1=pass1,pass2=pass2,pass3=pass3))



@token_required
def remove_pod(username,pod) -> str:
   q=mydb.users.find_one({"username":username}) 
   if pod=="1" :
     pod=username+'pod1'
     mydb.users.update_one({"username":username},{"$set":{'pass1':None}}) 
   elif pod=="2" :
     pod=username+'pod2'
     mydb.users.update_one({"username":username},{"$set":{'pass2':None}}) 
   elif pod=="3" :
     pod=username+'pod3'
     mydb.users.update_one({"username":username},{"$set":{'pass3':None}}) 
   else :
     return "Pod not exists"   
   body=kube_client.V1DeleteOptions()
   po=None
   try:
     po=api.read_namespaced_pod(pod,'default')
   except ApiException as e:
       if e.status != 404:
           print("Unknown error: %s" % e)
           exit(1)
   if po :
      api.delete_namespaced_pod(pod,'default',body)
   svc=None
   try:
       svc = api.read_namespaced_service(name=pod,namespace='default')
   except ApiException as e:
       if e.status != 404:
           print("Unknown error: %s" % e)
           exit(1)
   if svc :
        api.delete_namespaced_service(pod,'default')
   mydb.users.update_one({'username':username},{'$set':{pod+'list':None}})
   mydb.users.update_one({'username':username},{'$set':{pod+'out':None}})
   mydb.users.update_one({'username':username},{'$set':{'filename':None}})
   return "Pod deleted"

@token_required
def start_pod(username,pod,password) -> str:
   q=mydb.users.find_one({"username":username})
   if (q==None) :
     return "User does not exist"  
   o=urlparse(request.host_url).hostname
   print(o+"--------------")   #???#
   Lfiles=["",""]
   resp = None
   status="Not Ready"
   print(password['password'])
   if pod=='1' :
    name=username+'pod1'
    mydb.users.update_one({"username":username},{"$set":{'pass1':password['password']}}) 
    port_gzclient=q['port_gzclient']
   elif pod=='2' :
    name=username+'pod2'
    mydb.users.update_one({"username":username},{"$set":{'pass2':password['password']}}) 
    port_gzclient=q['port_gzclient']+4
   elif pod=='3' :
    name=username+'pod3'
    mydb.users.update_one({"username":username},{"$set":{'pass3':password['password']}}) 
    port_gzclient=q['port_gzclient']+8
   else :
    print('Already 3 pods running')
    return "Error"
   try: 
     resp = api.read_namespaced_pod(name=name,namespace='default')
       
   except ApiException as e:
       if e.status != 404:
           print("Unknown error: %s" % e)
           exit(1)

   if not resp:
       print("Pod %s does not exits. Creating it..." % name)
       pod_manifest = {
           'apiVersion': 'v1',
           'kind': 'Pod',
           'metadata': {
               'name': name ,
               'labels': {'name':name}
           },
           'spec': {
               'containers': [{
                   'image': 'dimrak/2018_web_simulator:rosauth_v2',
                   'name': name,
                   'command': ["/bin/bash"],
                   'args': ["-c", "Xvfb :1 -screen 0 1600x1200x16 "],
                   'volumeMounts': [{
                    'mountPath': '/home/',
                    'name': 'myupload',
                    'subPath': 'upload',
                    }]
                }],
                'volumes': [{
                   'name': 'myupload',
                   'persistentVolumeClaim': {'claimName':'claimdata'}
                 }]
           }

       }
       try:
         resp = api.create_namespaced_pod(body=pod_manifest,namespace='default')
       except ApiException as e:
          if e.status != 404:
           print("Unknown error: %s" % e)
           exit(1)
   resp1 = None
   try:
       resp1 = api.read_namespaced_service(name=name,namespace='default')
   except ApiException as e:
       if e.status != 404:
           print("Unknown error: %s" % e)
           exit(1)


   if not resp1:
        print("Service %s does not exits. Creating it..." % name)
        svc_manifest = {
           'apiVersion': 'v1',
           'kind': 'Service',
           'metadata': {
               'name': name 
           },
           'spec': {
             'selector':{
                'name':name
              },
             'ports':[
               {
                'name':'gzclient',
                'protocol':'TCP',
                'port':int(port_gzclient),
                'targetPort':8080
               },
               {
                'name':'gzwebsocket',
                'protocol':'TCP',
                'port':int(port_gzclient+1),
                'targetPort':7681
               },    
               {
                'name':'rosbridge',
                'protocol':'TCP',
                'port':int(port_gzclient+2),
                'targetPort':9090
               },
               {
                'name':'web-video',
                'protocol':'TCP',
                'port':int(port_gzclient+3),
                'targetPort':8081
               }],
                'externalIPs': [
                  o
                   ]
              

              }
        }
        resp1 = api.create_namespaced_service(body=svc_manifest,namespace='default')   
   r = make_response(render_template('pods.html',username=username))
   return "Done"   

def register() -> str :
 return make_response(render_template('register.html'))

def login_user(body) -> str:
    q=mydb.users.find_one({"username":body["username"]})
    if q!=None:
      if pwd_context.verify(body["password"],q["password"]): 
        token= jwt.encode({'user': body["username"],'exp':datetime.datetime.utcnow() + datetime.timedelta(minutes=60)},'sadfaf')
        response=make_response(url_for('.controllers_user_controller_get_user_by_name',username=body["username"],token=token))
        return response 
      else:
        return 'Password is invalid',(401)
    else:
      return 'Username does not exist',(401)



@token_required
def get_user_by_name(username) -> str:
   q=mydb.users.find_one({"username":username}) 
   if (q==None) :
     return "User does not exist" 
   mydb.users.update_one({'username':username},{'$set':{"login":datetime.datetime.utcnow()}})
   o=urlparse(request.host_url).hostname
   Lfiles=["",""]
   return make_response(render_template('home.html',username=username))

def create_user(body) -> str:
      port0=8001
      while (mydb.users.find_one({"port_gzclient":port0})):
          port0=port0+12
      if " " in body["username"] or len(body["username"])<3:
        return "Invalid username"  
      if mydb.users.find_one({"username":body["username"]}):
        return "User already exists"
      if mydb.users.find_one({"email":body["email"]}) or " " in body["email"] :
        return "Email already exists"
      if " " in body["password"] or len(body["password"])<2 :
        return "Invalid password" 
      password=hash_password(body["password"])       
      username=body["username"]
      user={"username":body["username"],"firstname":body["firstname"],"lastname":body["lastname"],"email":body["email"],"password":password,"port_gzclient":port0,"filename":None,"pass1":None,"pass2":None,"pass3":None,"login":None,(username+'pod1list'):None,username+'pod2list':None,username+'pod3list':None,username+'pod1out':None,username+'pod2out':None,username+'pod3out':None} 
      mydb.users.insert(user)
      if not os.path.exists('upload/'+body["username"]):
       os.makedirs('upload/'+body["username"]) 
      return "User created successfully"

@token_required
def get_deployment(username) -> str:
    q=mydb.users.find_one({"username":username})
    gz_port=q['port_gzclient']
    gz_port=gz_port+2  
    return make_response(render_template("deploys.html",username=username,port=gz_port))

def get_deployment_topics(username,pod) -> str :
    a=mydb.users.find_one({"username":username})
    status1,status2,status3=check_pods(username)
    if pod=='1' :
     name='pod1'
     if status1!='Running':
       return 'Simulation Environment 1 is not running',(403)
    elif pod=='2' :
     name='pod2'
     if status2!='Running':
       return 'Simulation Environment 2 is not running',(403)
    elif pod=='3' :
     name='pod3'
     if status3!='Running':
       return 'Simulation Environment 3 is not running',(403)
    else :
       return "Simulation Environment not valid",(403)     
    list=a[username+name+'list']
    out=a[username+name+'out']
    if list==None :
     list=[None] 
    Lfiles=list
    Lfiles.append(out)
    print(Lfiles)
    return Lfiles
  






@token_required
def get_mac(username,pod) -> str :
   q=mydb.users.find_one({"username":username})
   status1,status2,status3=check_pods(username)
   print(pod)
   gz_port=q['port_gzclient']
   if pod=='1' :
    name='pod1'
    if status1!='Running':
      return 'Simulation Environment 1 is not running',(403)
   elif pod=='2' :
    name='pod2'
    gz_port=gz_port+4
    if status2!='Running':
      return 'Simulation Environment 2 is not running',(403)
   elif pod=='3' :
    name='pod3'
    gz_port=gz_port+4
    if status3!='Running':
      return 'Simulation Environment 3 is not running',(403)
   else :
      return "Simulation Environment not valid",(403)
   cmd_mac='source /opt/ros/kinetic/setup.bash && source /root/rosauth/devel/setup.bash && rosparam set client_ip '+request.remote_addr+' && rosparam set rand '+username+pod+'  && rosrun rosauth ros_mac_generator'
   print(cmd_mac)
   command=['bin/bash','-c',cmd_mac]
   api_response = api.connect_get_namespaced_pod_exec(username+name,'default',command=command,stdout='True')
   cmd_get_mac='cat /root/mac.txt'
   command=['bin/bash','-c',cmd_get_mac]
   api_response = api.connect_get_namespaced_pod_exec(username+name,'default',command=command,stderr='True',stdout='True')
   mac='{'+api_response.split("{")[1].split("}")[0]+'}'
   mac=str(mac).replace("'", '"')
   return mac


@token_required
def get_secret(username,pod) -> str :
   q=mydb.users.find_one({"username":username})
   status1,status2,status3=check_pods(username)
   print(pod)
   gz_port=q['port_gzclient']
   if pod=='1' :
    name='pod1'
    if status1!='Running':
      return 'Simulation Environment 1 is not running',(403)
   elif pod=='2' :
    name='pod2'
    gz_port=gz_port+4
    if status2!='Running':
      return 'Simulation Environment 2 is not running',(403)
   elif pod=='3' :
    name='pod3'
    gz_port=gz_port+4
    if status3!='Running':
      return 'Simulation Environment 3 is not running',(403)
   else :
      return "Simulation Environment not valid",(403)
   cmd_secret='cat /root/rosauth/src/rosauth/test/valid.secret'
   print(cmd_secret)
   command=['bin/bash','-c',cmd_secret]
   api_response = api.connect_get_namespaced_pod_exec(username+name,'default',command=command,stderr='True',stdout='True')
   secret=api_response
   return secret

