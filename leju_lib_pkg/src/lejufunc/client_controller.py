#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from bodyhub.msg import SensorControl
from sensor_msgs.msg import ChannelFloat32
from bodyhub.srv import SrvInstWrite, SrvTLSstring
from ros_msg_node.srv import *
import json
from std_srvs.srv import *
controller_publisher = rospy.Publisher("/BodyHub/SensorControl",SensorControl,queue_size = 2)


def regist_sensor(sensor_name, reg_addr, data_len):
    # regist the sensor
    rospy.wait_for_service("/MediumSize/BodyHub/RegistSensor", 2)
    sensor_register = rospy.ServiceProxy('/MediumSize/BodyHub/RegistSensor', SrvInstWrite)
    sensor_register(sensor_name, reg_addr, data_len)


def delete_sensor(sensor_name):
    # delete the sensor
    rospy.wait_for_service("/MediumSize/BodyHub/DeleteSensor", 2)
    sensor_deleter = rospy.ServiceProxy('/MediumSize/BodyHub/DeleteSensor', SrvTLSstring)
    sensor_deleter(sensor_name)


def get_led_status():
    # read current led status from sensor
    sensor_name = "baseBoard"
    reg_addr = 24
    data_len = 56
    regist_sensor(sensor_name, reg_addr, data_len)

    data = rospy.wait_for_message("MediumSize/SensorHub/sensor_CF1", ChannelFloat32, 2)
    led_status = int(data.values[0]) >> 8

    delete_sensor(sensor_name)
    return led_status


def update_led_status(id, action, cur_status):
    new_status = cur_status
    if action == 1:
        # 开灯操作，待操作灯置为 1，其他置为 0，与当前状态执行或操作
        new_status = (1 << (id - 1)) | cur_status
    else:
        # 关灯操作，待操作灯置为 0，其他置为 1，与当前状态执行与操作
        next_status = [0x1E, 0x1D, 0x1B, 0x17, 0x0F]
        new_status = next_status[id-1] & cur_status

    return new_status


def led_controller(id, action):
    """
    Params:
      id: led light id, 1 ~ 5
      action: led action, 0 -- close, 1 -- open
    """
    led_status = get_led_status()
    new_status = update_led_status(id, action, led_status)

    sensor_name = "baseBoard"
    reg_addr = 26
    param = [new_status, 0x00]
    controller_publisher.publish(sensor_name, reg_addr, param)


def fan_controller(num):
    """

    :param para: 0:stop, 1:speed 1, 2:speed 2, 3:speed 3, 4:speed 4

    :return:
    """
    params = [[0x00],[0x11],[0x12],[0x13],[0x14]]
    para = params[num]
    sensor_name = "fan"
    reg_addr = 26
    controller_publisher.publish(SensorName = sensor_name,SetAddr = reg_addr,ParamList = para)


def send_video_status(is_on, url, width=640, height=480):
    rospy.wait_for_service("/ros_msg_node/process_msg_process", 2)
    set_status = rospy.ServiceProxy(
        '/ros_msg_node/process_msg_process', ProcessMsg)
    set_status("video_stream", json.dumps({"status": is_on, "url": url, "width":width, "height": height}))

def send_label_on(on):
    srv_name = "/switch_label"
    rospy.wait_for_service(srv_name, 1)
    client = rospy.ServiceProxy(srv_name, SetBool)
    client(on)

def camera_switch(on, type_):
    if type_.startswith("chin"):
        if type_ == "chin_rgb":
            url = "/chin_camera/" + ("start_capture" if on else "stop_capture")         
            rospy.wait_for_service(url, 1)
            client = rospy.ServiceProxy(url, Empty)
            client()
            return
    elif type_.startswith("realsense"):
        # TODO 之前测试的方式在程序运行阶段不可用，只能启动程序的时候生效。
        pass

    rospy.logerr("'{}' camera type does not exist!".format(type_))



if __name__ == '__main__':
    rospy.init_node("contorller_test", anonymous=True)
    rospy.sleep(3)

    try:
        send_label_on(False)
    except Exception as err:
        rospy.logerr(err)

