# Botec code modified by ZJUNlict

* tested in Ubuntu 20.04
* install
    * install **CoppeliaSim_Edu**(latest : 4.2.0)
    * clone whole project into **catkin_ws/src**
    * download **vrep ttt** model files into **Botec_Roban_Sim/vrep/\*** or any directory if you like
* Ubuntu 18.04
    * catkin_make find_package error
        * sudo apt-get install ros-melodic-joy*
        * sudo apt-get install ros-melodic-camera-info-manager
    * python3 
        * pip3 install rospkg
        * pip3 install netifaces
        * pip3 install defusedxml
* run
    * compile **catkin_ws/src**
    * `roscore`
    * `coppeliaSim.sh ***.ttt`
    * `rosrun ik_module ik_module_node`
    * `roslaunch bodyhub bodyhub.launch sim:=true`
    * `python3 Botec_Roban_Sim/player_scripts/demo.py`
