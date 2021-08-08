#!/usr/bin/python
import rospy
import sys, select, termios, tty
import time
from std_msgs.msg import String

keyboardPub = rospy.Publisher('teleop_keyboard/keyboard_control', String, queue_size=2)

msg = """
    Reading from the keyboard ...
    ---------------------------
    Begin walking: r
    Moving around:
            w    
       a    s    d
    Stop walking: q
    CTRL_C to exit.
    ---------------------------
    """


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    return key


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('keyboard_control')
    time.sleep(0.2)
    try:
        print(msg)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            key = getKey()
            keyboardPub.publish(key)
            rate.sleep()
            if key == '\x03':
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
