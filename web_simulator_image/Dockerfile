FROM ros:kinetic 
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
# setup sources.list
RUN . /etc/os-release \
    && . /etc/lsb-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/gazebo-latest.list
# install gazebo packages
RUN apt-get update && apt-get install -q -y \
    gazebo7 \
    libgazebo7-dev \
    ros-kinetic-gazebo-ros-pkgs \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-rosbridge-suite \
    ros-kinetic-web-video-server \
    unzip \
    unrar \
    python-catkin-tools \
    ros-kinetic-turtlebot-* \
    ros-kinetic-*-controller* \
    && rm -rf /var/lib/apt/lists/*
#install packages for gzweb
RUN apt-get update && apt-get install -q -y \
    build-essential \
    cmake \
    imagemagick \
    libboost-all-dev \
    libgts-dev \
    libjansson-dev \
    libtinyxml-dev \
    mercurial \
    nodejs \
    nodejs-legacy \
    npm \
    pkg-config \
    psmisc \
    xvfb\
    && rm -rf /var/lib/apt/lists/*
#install gzweb
RUN hg clone https://bitbucket.org/osrf/gzweb ~/gzweb
#deploy gzweb
RUN /bin/bash -c "cd ~/gzweb \
    && hg up gzweb_2.0.0 \
    && source /usr/share/gazebo-7/setup.sh \
    && xvfb-run -s '-screen 0 1280x1024x24' ./deploy.sh -m -t"
#change gzweb_files
#ADD rosauth /root/rosauth

RUN  cd /root/gzweb && npm install http-server-with-auth 

#ADD crsm-slam-ros-pkg /root/crsm-slam-ros-pkg
ADD gzweb/ /root/gzweb/
RUN chmod +x /root/gzweb/start_gzweb.sh
ADD opt/ros/kinetic/share/turtlebot_gazebo/ /opt/ros/kinetic/share/turtlebot_gazebo/
ADD opt/ros/kinetic/share/turtlebot_description/ /opt/ros/kinetic/share/turtlebot_description/
ADD opt/ros/kinetic/share/rosbridge_server/launch/ /opt/ros/kinetic/share/rosbridge_server/launch/

RUN mkdir -p /root/rosauth/src && cd /root/rosauth/src && git clone https://github.com/GT-RAIL/rosauth.git
ADD rosauth/src/rosauth/ /root/rosauth/src/rosauth/
RUN cd /root/rosauth && /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make "
#ADD rosauth/src/rosauth/ /root/rosauth/src/rosauth/
#RUN chmod +x /root/rosauth/src/rosauth/src/ros_mac_generator.cpp

#ADD client/* /root/gzweb/http/client/ ADD images/stop.png /root/gzweb/http/client/style/images/ 
#ADD includes/* /opt/ros/kinetic/share/turtlebot_gazebo/launch/includes/ 
#ADD robots/* /opt/ros/kinetic/share/turtlebot_description/robots/ 
#COPY models/turtlebot_description/meshes /root/gzweb/http/client/assets/turtlebot_description/meshes 
#COPY models/kobuki_description/meshes /root/gzweb/http/client/assets/kobuki_description/meshes 
#COPY models/create_description/meshes /root/gzweb/http/client/assets/create_description/meshes 
#ADD multi-turtle/* /opt/ros/kinetic/share/turtlebot_gazebo/launch/ 
#ADD models/turtlebot_description /root/.gazebo/models/turtlebot_description 
#ADD models/kobuki_description /root/.gazebo/models/kobuki_description 
#ADD models/create_description /root/.gazebo/models/create_description
#need to change launch files in turtlebot too,to spawn the robot model as sdf changes too to 
#/opt/ros/kinetic/share/turtlebot_gazebo/worlds/playground.world(cube_20k.stl -> cube_20k.dae) changes too to 
#/root/gzweb/http/client/assets/cube_20k/model.sdf(cube_20k.stl -> cube_20k.dae) add to /root/gzweb/http/client/assets/cube_20k/meshes/ (cube_20k.dae) changes 
#too to /root/.gazebo/models/cube_20k/model.sdf(cube_20k.stl -> cube_20k.dae)
#add to /root/.gazebo/models/cube_20k/meshes/ (cube_20k.dae)
