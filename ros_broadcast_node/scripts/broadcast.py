#!/usr/bin/env python
import rospy
import os
import time
import json
import getpass
import socket
import re

ROS_SOCKET_PORT = 12000
BRD_SOCKET_PORT = 5005


def get_username():
    return getpass.getuser()


def find_ip(value):
    ip_pattern = r'((25[0-5]\.|2[0-4]\d\.|1\d{2}\.|[1-9]?\d\.){3}(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d))'
    ip = re.findall(ip_pattern, value)
    return (ip[0][0], ip[1][0])


def find_mac(value):
    mac_pattern = r'([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}'
    mac = re.search(mac_pattern, value)
    return mac.group()


def get_net_name():
    cmd = r"ifconfig | grep  'Link encap' | awk '{print($1}'")
    names = os.popen(cmd).readlines()
    return [name.rstrip() for name in names if name != 'lo\n']


def find_all():
    result = []
    for name in get_net_name():
        try:
            val = "".join(os.popen(
                "ifconfig | grep -A2 {}".format(name)).readlines())
            ip = find_ip(val)
            mac = find_mac(val)
            result.append((ip, mac))
        except:
            continue
    return result


def send_broadcast(msgs):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    for msg in msgs:
        s.sendto(json.dumps(msg[1]).encode('utf-8'), (msg[0], BRD_SOCKET_PORT))
    s.close()


def main():
    ros_node_name = 'ros_broadcast_node'
    rospy.init_node(ros_node_name, anonymous=True)

    while not rospy.is_shutdown():
        rospy.sleep(3)
        try:
            datas = find_all()
            msgs = []
            for data in datas:
                ip = data[0][0]
                username = get_username()
                mac = data[1]
                msgs.append([])
                msgs[-1].append(data[0][1])
                msgs[-1].append({
                    "ip": ip,
                    "username": username,
                    "ros_port": ROS_SOCKET_PORT,
                    "mac": mac
                })
            if not rospy.get_param("socket_is_connected", False):
                send_broadcast(msgs)
        except Exception as error:
            rospy.loginfo(error)


if __name__ == '__main__':
    main()
