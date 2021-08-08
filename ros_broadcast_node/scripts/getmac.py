#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import re
import shlex
import socket
import struct
from subprocess import check_output
import rospy

DEVNULL = open(os.devnull, "wb")
PATH = os.environ.get("PATH", os.defpath).split(os.pathsep)
PATH.extend(("/sbin", "/usr/sbin"))
ENV = dict(os.environ)
ENV["LC_ALL"] = "C"  # Ensure ASCII output so we parse correctly
MAC_RE_COLON = r"([0-9a-fA-F]{2}(?::[0-9a-fA-F]{2}){5})"


def _read_file(filepath):
    try:
        with open(filepath) as f:
            return f.read()
    except (OSError, IOError):
        rospy.logdebug("Could not find file: '%s'", filepath)
        return None


def _search(regex, text, group_index=0):
    match = re.search(regex, text)
    if match:
        return match.groups()[group_index]
    return None


def _popen(command, args):
    for directory in PATH:
        executable = os.path.join(directory, command)
        if (
                os.path.exists(executable)
                and os.access(executable, os.F_OK | os.X_OK)
                and not os.path.isdir(executable)
        ):
            break
    else:
        executable = command
    cmd = [executable] + shlex.split(args)
    output = check_output(cmd, stderr=DEVNULL, env=ENV)
    return str(output)


def _try_methods(methods, to_find=None):
    found = None
    for m in methods:
        try:
            if isinstance(m, tuple):
                for arg in m[3]:
                    found = _search(m[0], _popen(m[2], arg), m[1])
            elif callable(m):
                if to_find is None:
                    found = m()
                else:
                    found = m(to_find)
        except Exception as ex:
            rospy.loginfo("exception:%s", str(ex))
            continue
        if found:
            break
    return found


def _get_default_iface_linux():
    data = _read_file("/proc/net/route")
    if data is not None and len(data) > 1:
        for line in data.split("\n")[1:-1]:
            iface_name, dest = line.split("\t")[:2]
            if dest == "00000000":
                return iface_name
    return None


def _hunt_linux_default_iface():
    methods = [
        _get_default_iface_linux,
        lambda: _popen("route", "-n")
            .partition("0.0.0.0")[2]
            .partition("\n")[0]
            .split()[-1],
        lambda: _popen("ip", "route list 0/0")
            .partition("dev")[2]
            .partition("proto")[0]
            .strip(),
    ]
    return _try_methods(methods)


def _hunt_for_mac(to_find):
    if to_find is None:
        rospy.logwarn("_hunt_for_mac() failed: to_find is None")
        return None
    methods = [
        _read_sys_iface_file,
        _fcntl_iface,
        (r"ether " + MAC_RE_COLON, 0, "ifconfig", [to_find]),
        (r"HWaddr " + MAC_RE_COLON, 0, "ifconfig", [to_find]),
        (
            to_find + r".*\n.*link/ether " + MAC_RE_COLON,
            0,
            "ip",
            ["link %s" % to_find, "link"],
        ),
        (to_find + r".*HWaddr " + MAC_RE_COLON, 0, "netstat", ["-iae"]),
        (to_find + r".*ether " + MAC_RE_COLON, 0, "ifconfig", [""]),
        (to_find + r".*HWaddr " + MAC_RE_COLON, 0, "ifconfig", ["", "-a", "-v"]),
        (to_find + r".*Ether " + MAC_RE_COLON, 0, "ifconfig", ["-av"]),
    ]
    return _try_methods(methods, to_find)


def _read_sys_iface_file(iface):
    data = _read_file("/sys/class/net/" + iface + "/address")
    return None if data is not None and len(data) < 17 else data


def _fcntl_iface(iface):
    import fcntl
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    info = fcntl.ioctl(s.fileno(), 0x8927, struct.pack("256s", iface[:15]))  # 0x8927 = SIOCGIFADDR
    return ":".join(["%02x" % ord(char) for char in info[18:24]])


def get_mac_address():
    to_find = _hunt_linux_default_iface()
    mac = _hunt_for_mac(to_find)
    return mac
