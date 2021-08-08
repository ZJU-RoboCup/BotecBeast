#! /usr/bin/python
import os
import signal
import rospy
from lejulib import *


def terminate(data):
    """terminate current process
    """
    rospy.loginfo(data.data)
    os.system("ps -aux | grep \"roslaunch ros_evs_node\" | awk '{print($2}' | xargs kill"))
    pid = os.getpid()
    os.kill(pid, signal.SIGKILL)


def main():
    try:
        rospy.init_node("act_runner", anonymous=True)
        rospy.sleep(0.2)
        rospy.Subscriber('terminate_current_process', String, terminate)
        rospy.loginfo("action node running")
        os.system("roslaunch ros_evs_node start.launch")
    except Exception as err:
        serror(err)
    finally:
        finishsend()

if __name__ == '__main__':
    main()
