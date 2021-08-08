#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import os
import sys
import json
import threading
import socket
import time
import array
import logging

from ZeroServoControl import ZeroServoTransfer
from bodyhub.msg import JointControlPoint
from bodyhub.srv import *  # for SrvState.srv
from std_msgs.msg import UInt16
from std_msgs.msg import String
from ros_actions_node.msg import DeviceList
from lejufunc.bezier import get_custom_points, get_bezier_frames
from nodes import Node
from lejufunc.ServoJointTrajectory import ServoJointTrajectoryC
from sensor_msgs.msg import Joy
import yaml
import Queue   
from motion.motionControl import *

logging.basicConfig(level=logging.DEBUG, format=' %(asctime)s - %(levelname)s- %(message)s')

HOST = '0.0.0.0'
PORT = 12000
os.system('kill -9 $(fuser {}/tcp 2>/dev/null)'.format(PORT))

ROS_NODE_NAME = 'SocketNode'
rospy.loginfo("Starting {}".format(ROS_NODE_NAME))
rospy.init_node(ROS_NODE_NAME, anonymous=True, log_level=rospy.INFO)
rospy.sleep(2.0)  # The necessary delay for init

# TODO: immature code inside
SERVO = ServoJointTrajectoryC()
NODE = Node()
lock = threading.Lock()
run_lock = threading.Lock()
exec_lock = threading.Lock()

actQueue = Queue.Queue()
current_sock = None
isStopLast = False
EXE_RUN_TIME_OUT = 20


def _list_to_array(act_list):
    return array.array("d", act_list)


def _dict_to_bytes(dict_msg):
    return bytes(json.dumps(dict_msg))


def load_config(configfile):
    """reading config file

    :return: offset list or InitPose list
    """
    filePath = rospy.get_param(configfile)
    with open(filePath, "r") as file:
        file_body = yaml.load(file.read())
    if configfile == "poseOffsetPath":
        filecon = file_body["offset"]
    elif configfile == "poseInitPath":
        filecon = file_body["InitPose"]

    values = []
    for i in range(len(filecon)):
        keyvalue = "ID" + str(i + 1)
        values.append(filecon[keyvalue])
    return values


def async_do_job(func, args=None):
    """unblock do func task

    :param func:
    :param args:
    :return:
    """
    threading.Thread(target=func, args=args).start()


# TODO: NOT WORKING
def wait_move_done():
    """waiting for servo move done

    :return:
    """
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_message('MediumSize/BodyHub/Status', UInt16, 0.1)
            break
        except Exception as err:
            print(err)
            break


def act_execution_state():
    """enter action execution mode

    :return:
    """
    lock.acquire()
    NODE.masterID_guarantee()
    current_state = NODE.get_current_status()
    if current_state == "error":
        rospy.logerr("bodyhub state error")
        raise
    if current_state == "directOperate":
        NODE.state_jump("reset")
        NODE.state_jump("setStatus")
    elif current_state == "preReady":
        NODE.state_jump("setStatus")
    elif current_state == "walking":
        NODE.state_jump("stop")
        NODE.wait_for_ready()
    lock.release()


def act_directoperation_state():
    """enter direct method mode

    :return:
    """
    lock.acquire()
    NODE.masterID_guarantee()
    current_state = NODE.get_current_status()
    if current_state == "error":
        rospy.logerr("bodyhub state error")
        raise
    if current_state != "directOperate":
        if current_state in ["running", "pause", "walking"]:
            NODE.state_jump("stop")
        elif current_state == "preReady":
            NODE.state_jump("setStatus")
        NODE.wait_for_ready()
        NODE.state_jump("derectOperate")
    lock.release()


def walking_state():
    """enter walking mode

    :return:
    """
    NODE.masterID_guarantee()
    current_state = NODE.get_current_status()
    if current_state == "error":
        rospy.logerr("bodyhub state error")
        raise
    if current_state != "walking":
        if current_state == "preReady":
            NODE.state_jump("setStatus")
        elif current_state in ["running", "pause"]:
            NODE.state_jump("stop")
        elif current_state == "directOperate":
            NODE.state_jump("reset")
            NODE.state_jump("setStatus")
        NODE.wait_for_ready()
        NODE.state_jump("walking")



def act_execution(path):
    print(path)
    NODE.node_run(path)


def reset_walking_state():
    lock.acquire()
    NODE.walk_node_stop()
    while GetBodyhubStatus().data == "walking":
        time.sleep(0.01)
    NODE.walk_node_reset()
    NODE.walkflag = False
    lock.release()


def set_head_servo(angles, time, sck_conn):
    """set servos [21, 22] angles

    :param angles:[ang1,ang2]
    :return:
    """

    try:
        act_execution_state()
        SERVO.HeadJointTransfer(angles, time)
        SERVO.MotoWait()

        msg = {'cmd': 'set_head_servo',
               'state': 'ok'}
    except rospy.ServiceException as e:
        msg = {'cmd': 'set_head_servo',
               'state': 'error'}
    finally:
        sck_conn.send(_dict_to_bytes(msg))

is_stop = False
def set_all_servo(ids, angles, act_time, sck_conn, sendback=None):
    """set servos [1, 22] angles

    :param ids:
    :param angles:
    :return:
    """
    global is_stop
    msg = {
        'cmd': 'set_all_servo',
        'state': 'error'
    }
    is_first = True
    is_stop = False
    run_lock.acquire()
    try:
        rospy.wait_for_service('/MediumSize/BodyHub/GetJointAngle', 2)
        servo_client = rospy.ServiceProxy('/MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
        id_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        act_execution_state()
        result = servo_client(id_list, len(id_list))
        p0 = result.getData 
        move_frames = get_bezier_frames(p0, angles, None, act_time)[1:]
        for frame in move_frames:
            if is_stop:
                return
            SendJointCommand(2, id_list, frame)
            time.sleep(0 if is_first else 0.009)
            is_first = False
        WaitTrajectoryExecute(True)
        msg = {
            'cmd': 'set_all_servo',
            'state': 'ok'
        }
    finally:
        run_lock.release()
        if sendback == "TRUE":
            sck_conn.send(_dict_to_bytes(msg))


def set_multiple_all_servo(sck_conn, act_frames, start_index, stop_index):
    global is_stop
    act_execution_state()
    last_point = None
    is_first = True
    is_stop = False
    act_frames = { int(key): act_frames[key] for key in act_frames }
    joint_ids = act_frames.keys()
    joint_ids.sort()
    run_lock.acquire()
    num = 0
    for point in get_custom_points(act_frames):
        if is_stop:
            break       
        if num >= start_index and num <= stop_index:  
            SendJointCommand(2, joint_ids, point)
            time.sleep(0 if is_first else 0.009) 
            is_first = False 
        num = num + 1
    WaitTrajectoryExecute(True)
    run_lock.release()
    msg = {
        'cmd': 'set_multiple_all_servo',
        'state': 'ok'
    }
    sck_conn.send(_dict_to_bytes(msg))

def get_all_servo(id_list, sck_conn):
    """get servos [1, 22] angles

    :param id_list:
    :param sck_conn:
    :return:
    """

    try:
        rospy.wait_for_service('/MediumSize/BodyHub/GetJointAngle', 2)
        servo_client = rospy.ServiceProxy('/MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
        result = servo_client(id_list, len(id_list))
        resp_value = result.getData

        msg = {
            'cmd': 'get_position',
            'state': 'ok',
            'pos': resp_value
        }
    except rospy.ServiceException as e:
        msg = {
            'cmd': 'get_position',
            'state': 'error'
        }
    finally:
        sck_conn.send(_dict_to_bytes(msg))


def view_action(act_frames, sck_conn):
    """ View action

    :param act_frames:
    :param sck_conn:
    :return:
    """
    try:
        lock.acquire()
        actQueue.put(act_frames)
        lock.release()
        msg = {'cmd': 'view_action',
               'state': 'ok'}
    except Exception as e:
        msg = {'cmd': 'view_action',
               'state': 'error'}
    finally:
        sck_conn.send(_dict_to_bytes(msg))


def stop_action(sck_conn, id):
    """stop action view

    :param sck_conn:
    :return:
    """
    
    global is_stop
    #if is_stop:
    is_stop = True
    msg = {
        'cmd': 'stop_action',
        'state': 'error',
        'id': id
    }
    try:
        rospy.wait_for_service('/MediumSize/BodyHub/GetJointAngle', 2)
        servo_client = rospy.ServiceProxy('/MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
        id_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        result = servo_client(id_list, len(id_list))
        resp_value = result.getData

        msg = {
            'cmd': 'stop_action',
            'state': 'ok',
            'last_angle': resp_value,
            'id': id
        }
    except rospy.ServiceException as e:
        pass
    finally:
        sck_conn.send(_dict_to_bytes(msg))


imuDate = ()
def read_imuDateCallback(msg):
    imuDate = tuple(msg.data)
    rospy.loginfo("gyro:[%6.2f,%6.2f,%6.2f]",imuDate[0], imuDate[1], imuDate[2])
    rospy.loginfo("acc: [%6.2f,%6.2f,%6.2f]",imuDate[3], imuDate[4], imuDate[5])


def read_imuDate(sck_conn):
    rospy.Subscriber("/imu/Torso", Float64MultiArray, read_imuDateCallback)
    msg = {
        'cmd'  : 'read_imuDate',
        'state': 'imu_on',
        'data' : imuDate
    }
    sck_conn.send(_dict_to_bytes(msg))


def set_imuState(sck_conn, imu_state):
    rospy.wait_for_service("imuState")
    try:
        imuState_client = rospy.ServiceProxy("imuState",SrvimuState)
        resp = imuState_client.call(imu_state)
        rospy.loginfo("%s" %resp.freeback)
    except rospy.ServiceException, err:
        rospy.logwarn("Service call failed: %s" %err)
    sck_conn.send(_dict_to_bytes(resp.freeback))


def actmoto():
    """ moto queue to act

    :return:
    """

    while not rospy.is_shutdown():
        if actQueue.empty():
            time.sleep(0.01)
        else:
            lock.acquire()
            frame = actQueue.get()
            lock.release()
            servo_value = frame[0]
            acttime = frame[1]
            interval = frame[2] / 1000.0
            set_all_servo(None, servo_value, acttime, None)
            time.sleep(interval)


def get_act_nodes(sck_conn):
    """list ros node demo file under dir

    :param sck_conn:
    :return:
    """
    app_list = NODE.list_app()
    msg = {
        "cmd": "ls_node",
        "nodes": app_list
    }
    sck_conn.send(_dict_to_bytes(msg))


def get_login_account(sck_conn):
    """return ssh username and password

    :param sck_conn:
    :return:
    """
    msg = {
        "cmd": "get_login_account",
        "user": "lemon",
        "password": "softdev"
    }
    sck_conn.send(_dict_to_bytes(msg))

def exec_download_file(path):
    """executr download file

    :param path:
    :return:
    """
    if NODE.runningflag:
        reset_body_state()
        time.sleep(2)
    if isStopLast:
        NODE.runningflag = False
        return

    exec_lock.acquire()
    NODE.runningflag = True
    act_execution_state()
    act_execution(path)
    exec_lock.release()


def wait_terminate_sub(time_out):
    for _ in range(time_out):
        if NODE.terminate_pub.get_num_connections() != 0:
            return True
        time.sleep(1)
    return False

def reset_bodyhub(sck_conn):
    
    ResetBodyhub()
    msg = {
        "cmd": "reset_bodyhub",
        "state": "ok"
    }
    sck_conn.send(_dict_to_bytes(msg))


def reset_body_state():
    """reset bodyhub and actExec node state
       robot standby.
    :return:
    """
    while exec_lock.locked():
        time.sleep(0.1)
    if not NODE.runningflag:
        return

    exec_lock.acquire()
    # publish to terminate current python process
    terminal_str = "Terminate current python process"
    if not wait_terminate_sub(EXE_RUN_TIME_OUT):
        print("脚本启动异常或未监听停止指令")
    NODE.terminate_pub.publish(terminal_str)
    NODE.finish_pub.publish("finish")
    NODE.runningflag = False
    while not rospy.is_shutdown():
        try:
            if GetBodyhubStatus().data == "preReady":
               break 
            else:
                ResetBodyhub()
        except Exception as e:
            print("Error:", e)
        time.sleep(1)
    exec_lock.release() 

def query_node_state(sck_conn):
    """check demo node is running

    :return:
    """
    state = NODE.get_current_status()
    msg = {
        "cmd": "query_node_state",
        "state": state
    }
    sck_conn.send(_dict_to_bytes(msg))

con_is_reset = False
def handle_controller_cmd(sck_conn, key_value):
    """handle controller command

    :param key_value: [x1,y1,x2,y2]
    :return:
    """
    global con_is_reset
    try:
        con_is_reset = False
        while lock.locked():
            if con_is_reset:
                time.sleep(0.01)
                break
        lock.acquire()
        if not NODE.walkflag:
            walking_state()
            NODE.walkflag = True
        lock.release()
        axes = [0, 0, 0, 0, 0, 0, 0, 0]  # 0-7
        buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # 0-10
        axes[0] = -1 * key_value[0]
        axes[1] = key_value[1]
        axes[3] = -1 * key_value[2]
        axes[4] = key_value[3]
        joyPub = rospy.Publisher('/joy', Joy, queue_size=2)
        joyPub.publish(axes=axes, buttons=buttons)

        msg = {
            "cmd": "receive_controller_cmd",
            "state": 'ok'
        }
    except rospy.ServiceException as e:
        msg = {
            'cmd': 'receive_controller_cmd',
            'state': 'error'
        }
    except Exception as e:
        print(e)
        msg = {
            'cmd': 'receive_controller_cmd',
            'state': 'error'
        }
    finally:
        sck_conn.send(_dict_to_bytes(msg))


def handle_controller_reset(sck_conn):
    """handle controller reset

    :param :
    :return:
    """
    global con_is_reset
    con_is_reset = True
    try:
        reset_walking_state()
        msg = {
            "cmd": "receive_controller_reset",
            "state": 'ok'
        }
    except rospy.ServiceException as e:
        msg = {
            'cmd': 'receive_controller_reset',
            'state': 'error'
        }
    except Exception as e:
        print(e)
        msg = {
            'cmd': 'receive_controller_cmd',
            'state': 'error'
        }
    finally:
        sck_conn.send(_dict_to_bytes(msg))


def get_zero_value(sck_conn):
    """get servos [1, 22] zero position offset

    :param sck_conn:
    :return:
    """
    zero_value = load_config("poseOffsetPath")
    msg = {
        "cmd": "get_zero_value",
        "zero_value": zero_value
    }
    sck_conn.send(_dict_to_bytes(msg))


def set_zero_servo(sck_conn, sendback=None, slowspeed=None, ids=None, zerovalue=None):
    """ set servos [1, 22] zero position

    :param sck_conn:
    :param ids: None or [1-22]
    :param zerovalue: None or [value1-value22]
    :return:
    """
    try:
        act_directoperation_state()
        if ids == None or zerovalue == None:
            cmd = 'view_zero_position'
            ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
            angles = load_config("poseOffsetPath")
            slowspeed = "TRUE"
        else:
            cmd = 'set_zero_servo'
            ids = ids
            angles = zerovalue
        angles = [i + 2048.0 for i in angles]
        if slowspeed == "TRUE":
            ZeroServoTransfer(angles)
        else:
            rospy.wait_for_service("/MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll", 2)
            servomoto = rospy.ServiceProxy("/MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll",
                                           SrvServoAllWrite)
            servomoto(ids, len(angles), angles)

        msg = {'cmd': cmd,
               'state': 'ok'}
    except rospy.ServiceException as err:
        msg = {'cmd': cmd,
               'state': 'error'}
    finally:
        if sendback == "TRUE":
            sck_conn.send(_dict_to_bytes(msg))


def virtual_control_panel(sck_conn, key):
    """

    :param sck_conn:
    :param key: 'A' 'B' 'X' 'Y' 'LEFT' 'RIGHT' 'UP' 'DOWN' 'STOP'
    :return:
    """
    try:
        NODE.buttonPub.publish(data=key)
        msg = {'cmd': 'virtual_control_panel',
               'state': 'ok'}
    except rospy.ROSException as err:
        msg = {'cmd': 'virtual_control_panel',
               'state': 'error'}

    finally:
        sck_conn.send(_dict_to_bytes(msg))

def get_init_value(sck_conn):
    """get servos [1, 22] init position value

    :param sck_conn:
    :return:
    """
    init_value = load_config("poseInitPath")
    msg = {
        "cmd": "get_init_value",
        "init_value": init_value
    }
    sck_conn.send(_dict_to_bytes(msg))


def getGroups(s):
    groups = []
    count = 0  
    current = ""
    for c in s:
        if c == '{':
            count += 1
        elif c == '}':
            count -=1 
        current += c
        if count == 0:
            groups.append(json.loads(current))
            current = ""

    return groups, current

pre_data = ""

def handle_sck_data(_datas, conn):
    """process socket json cmd

    :param data:
    :param conn:
    :return:
    """
    try:
        global pre_data
        datas, pre_data = getGroups(pre_data + _datas.decode("utf-8"))
        for data in datas:
            print(data)
            cmd = data['cmd']
            if cmd == 'set_all_servo':
                async_do_job(set_all_servo, args=(data['id'], data['angle'], data['time'], conn, data['sendback']))
            elif cmd == 'set_head_servo':
                async_do_job(set_head_servo, args=(data['angle'], data['time'], conn,))
            elif cmd == 'get_all_servo':
                async_do_job(get_all_servo, args=(data['id'], conn,))
            elif cmd == 'set_multiple_all_servo':
                async_do_job(set_multiple_all_servo, args=(conn, data['act_frames'],data['start'],data['stop']))
            elif cmd == 'set_body_state':
                async_do_job(NODE.set_act_state, args=(data['state'],))
            elif cmd == 'ls_node':
                async_do_job(get_act_nodes, args=(conn,))
            elif cmd == 'run_node':
                async_do_job(exec_download_file, args=(data['path'],))
            elif cmd == 'stop_node':
                async_do_job(reset_body_state, args=())
            elif cmd == 'query_node_state':
                async_do_job(query_node_state, args=(conn,))
            elif cmd == 'get_login_account':
                async_do_job(get_login_account, args=(conn,))
            elif cmd == 'send_controller_cmd':
                async_do_job(handle_controller_cmd, args=(conn, data['key'],))
            elif cmd == 'send_controller_reset':
                async_do_job(handle_controller_reset, args=(conn,))
            elif cmd == 'get_zero_value':
                async_do_job(get_zero_value, args=(conn,))
            elif cmd == 'view_zero_position':
                async_do_job(set_zero_servo, args=(conn,))
            elif cmd == 'set_zero_servo':
                async_do_job(set_zero_servo,
                            args=(conn, data['sendback'], data['slowspeed'], data['id'], data['zerovalue']))
            elif cmd == 'virtual_control_panel':
                async_do_job(virtual_control_panel, args=(conn, data['key'],))
            elif cmd == 'view_action':
                async_do_job(view_action, args=(data['frame'], conn))
            elif cmd == 'stop_action':
                async_do_job(stop_action, args=(conn, data["id"]))
            elif cmd == 'get_init_value':
                async_do_job(get_init_value, args=(conn,))
            elif cmd == 'reset_bodyhub':
                async_do_job(reset_bodyhub, args=(conn,)) 
            elif cmd == 'set_imuState':
                async_do_job(set_imuState, args=(conn, data["imu_state"]))
            elif cmd == 'read_imuDate':
                async_do_job(read_imuDate, args=(conn,))
            else:
                print("[WARN] WIP...")

    except Exception as err:
        logging.error(err)


# send device list in current demo
def send_device_list(data):
    rospy.loginfo("camera status: %d", data.camera_status)
    rospy.loginfo("controller status: %d", data.controller_status)
    if not current_sock is None:
        msg = {
            'cmd': 'demo_device_list',
            'camera_status': data.camera_status,
            'controller_status': data.controller_status
        }
        current_sock.send(_dict_to_bytes(msg))


def thread_handle_connect(conn):
    """wait for socket client bytes buffer data,
    try to reconnect when timeout
    :param conn:
    :return:
    """
    global current_sock
    global con_is_reset
    while not rospy.is_shutdown():
        try:
            data = conn.recv(8192 * 3)
            if not data:
                print('[WARN] lost socket connection ...')
                conn.close()
                current_sock = None
                NODE.walkflag = False
                con_is_reset = True
                break
            # async run
            handle_sck_data(data, conn)

        except Exception as e:
            err = e.args[0]
            if err == 'timed out':
                continue
            else:
                conn.close()
                current_sock = None
                con_is_reset = True
                NODE.walkflag = False
                break

def get_socket():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    server.setsockopt(socket.SOL_TCP, socket.TCP_KEEPIDLE, 5)
    server.setsockopt(socket.SOL_TCP, socket.TCP_KEEPINTVL, 3)
    server.setsockopt(socket.SOL_TCP, socket.TCP_KEEPCNT, 3)
    server.bind((HOST, PORT))
    server.listen(1)
    server.settimeout(None)
    return server

def main():
    """main entry
    """

    server = None
    rospy.Subscriber("/ActRunner/DeviceList", DeviceList, send_device_list)
    threading.Thread(target=actmoto).start()

    global current_sock
    while not rospy.is_shutdown():
        if current_sock:
            time.sleep(1)
            continue
        else:
            print('listening...')
            server =  get_socket()
            rospy.set_param("socket_is_connected", False)
            conn, addr = server.accept()
            rospy.set_param("socket_is_connected", True)
            server.close()
        current_sock = conn
        print('handling connection from %s' % (addr,))
        threading.Thread(target=thread_handle_connect, args=(conn,)).start()


if __name__ == '__main__':
    main()
