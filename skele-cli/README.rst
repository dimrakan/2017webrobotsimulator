skele-cli
=========

*A command line program in Python to connect ROS through Rosbridge.*


Purpose
-------

This is a application which connects ROS through Rosbridge.It has been developed to connect ROS, which is running in a Pod(Docker Container) from Kubernetes Cluster, to every Personal Computer is running ROS.

Usage
-----
Download this folder and run the following command in your folder.(This packages must be installed in the local system.
rosbridge_pyclient(https://github.com/robotics-4-all/rosbridge_pyclient) && rosconversions_py(https://github.com/robotics-4-all/rosconversions_py))

    $ pip install -e .[test]


After installing use command line commands.


  skele tf --ip=<ip> --secret=<secret>

  skele tf --ip=<ip> --secret_fpath=<secet_fpath>

  skele sub --remote=<remote> --local=<local> --ip=<ip> --secret=<secret> 

  skele sub --remote=<remote> --local=<local> --ip=<ip> --secret=<secret_fpath> 

  skele sub --remote=<remote>  --ip=<ip> --secret=<secret>

  skele sub --remote=<remote>  --ip=<ip> --secret_fpath=<secret_fpath>

  skele pub --remote=<remote> --local=<local> --ip=<ip> --secret=<secret>

  skele pub --remote=<remote> --local=<local> --ip=<ip> --secret_fpath=<secret_fpath>

  skele pub --remote=<remote> --local=<local> --ip=<ip> --secret=<secret> --type=<type>

  skele pub --remote=<remote> --local=<local> --ip=<ip> --secret_fpath=<secret_fpath> --type=<type>

  skele pub --remote=<remote>  --ip=<ip> --secret=<secret>

  skele pub --remote=<remote>  --ip=<ip> --secret_fpath=<secret_fpath>

  skele srv --ip=<ip> --secret=<secret> --remote=<remote> 

  skele srv --ip=<ip> --secret_fpath=<secret_fpath> --remote=<remote> 

  skele action --ip=<ip> --secret=<secret> --remote=<remote> 

  skele action --ip=<ip> --secret_fpath=<secret_fpath> --remote=<remote> 


