#!/usr/bin/env python

import rospy
import os
import json
import threading
import socket

from ros_msg_node.srv import *

PUBLIC_HOST = '0.0.0.0'
PUBLIC_PORT = 5006
os.system('kill -9 $(fuser {}/tcp 2>/dev/null)'.format(PUBLIC_PORT))

socket_conn = None


def _dict_to_bytes(dict_msg):
    return bytes(json.dumps(dict_msg))


def send_debug_message(req):
    if not socket_conn is None:
        msg = {
            "cmd": req.msgType,
            "content": req.processMsg
        }
        socket_conn.send(_dict_to_bytes(msg))
        return ProcessMsgResponse(1)
    else:
        rospy.logwarn("No socket connect")
        return ProcessMsgResponse(0)


def send_finish_message(req):
    if not socket_conn is None:
        msg = {
            "cmd": "end_message",
            "content": "Running finish"
        }
        socket_conn.send(_dict_to_bytes(msg))
        rospy.loginfo("send finish message success")
        return RunningFinishResponse()
    else:
        rospy.logwarn("No socket connect")
        return RunningFinishResponse()


def thread_handle_connect(conn):
    """wait for socket client bytes buffer data,
    try to reconnect when timeout
    :param conn:
    :return:
    """
    global socket_conn
    while not rospy.is_shutdown():
        try:
            data = conn.recv(1024)
            if not data:
                rospy.logwarn("message socket disconnected")
                conn.close()
                socket_conn = None
                break

        except Exception as e:
            err = e.args[0]
            if err == 'timed out':
                continue
            else:
            	rospy.logerr(e)
                conn.close()
                socket_conn = None
                break



def main():
    # initial node
    rospy.init_node("ros_msg_node", anonymous=True)
    rospy.sleep(2.0)

    #initial service
    error_msg = rospy.Service('/ros_msg_node/process_msg_process', ProcessMsg, send_debug_message)
    finish_msg = rospy.Service('/ros_msg_node/finish_msg_process', RunningFinish, send_finish_message)

    # initial socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((PUBLIC_HOST, PUBLIC_PORT))
    server.listen(1)
    server.settimeout(None)

    global socket_conn
    while not rospy.is_shutdown():
        rospy.loginfo("message node waiting for connection")
        conn, addr = server.accept()
        socket_conn = conn
        rospy.loginfo("connected by %s" % (addr,))
        threading.Thread(target=thread_handle_connect, args=(conn,)).start()


if __name__ == '__main__':
	main()
