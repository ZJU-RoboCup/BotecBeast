#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from sys import path
import os
import yaml

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Range
from sensor_msgs.msg import ChannelFloat32
from sensor_msgs.msg import Illuminance
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity

from bodyhub.msg import SensorRawData

SensorNameIDFilePath = os.path.abspath(os.path.join(os.path.dirname(__file__), "../config/sensorNameID.yaml"))
SensorIdNameDict = {}
SensorList = []

ImuPub = rospy.Publisher('MediumSize/SensorHub/Imu', Imu, queue_size=1)
MagneticFieldPub = rospy.Publisher('MediumSize/SensorHub/MagneticField', MagneticField, queue_size=1)
BatteryStatePub = rospy.Publisher('MediumSize/SensorHub/BatteryState', BatteryState, queue_size=1)
RangePub = rospy.Publisher('MediumSize/SensorHub/Range', Range, queue_size=1)
ChannelFloat32Pub = rospy.Publisher('MediumSize/SensorHub/sensor_CF1', ChannelFloat32, queue_size=1)
IlluminancePub = rospy.Publisher('MediumSize/SensorHub/Illuminance', Illuminance, queue_size=1)
TemperaturePub = rospy.Publisher('MediumSize/SensorHub/Temperature', Temperature, queue_size=1)
HumidityPub = rospy.Publisher('MediumSize/SensorHub/Humidity', RelativeHumidity, queue_size=1)


class Sensor:
    def __init__(self, name, id, addr, length, data):
        self.name = name
        self.id = id
        self.dataAddr = addr
        self.dataLength = length
        self.rawData = data

    def getData(self):
        return self.rawData


def UintToInt(num, bit):
    bit -= 1
    if (num & (1 << bit)):
        temp = ~(num - 1)
        return -(temp & ((1 << bit) - 1))
    return num


# 板载imu
def ImuDataProcess(rawData):

    grayCoefficient = 2000.0 / 32768
    accCoefficient = 16.0 / 32768
    imuData = Imu()

    imuData.angular_velocity.x = UintToInt((rawData[1] << 8) + rawData[0], 16)
    imuData.angular_velocity.y = UintToInt((rawData[3] << 8) + rawData[2], 16)
    imuData.angular_velocity.z = UintToInt((rawData[5] << 8) + rawData[4], 16)

    imuData.angular_velocity.x *= grayCoefficient
    imuData.angular_velocity.y *= grayCoefficient
    imuData.angular_velocity.z *= grayCoefficient

    imuData.linear_acceleration.x = UintToInt((rawData[7] << 8) + rawData[6], 16)
    imuData.linear_acceleration.y = UintToInt((rawData[9] << 8) + rawData[8], 16)
    imuData.linear_acceleration.z = UintToInt((rawData[11] << 8) + rawData[10], 16)

    imuData.linear_acceleration.x *= accCoefficient
    imuData.linear_acceleration.y *= accCoefficient
    imuData.linear_acceleration.z *= accCoefficient

    ImuPub.publish(imuData)


# 板载磁场
def MagneticDataProcess(rawData):

    magCoefficient = 50.0 / 32768
    magData = MagneticField()

    magData.magnetic_field.x = UintToInt((rawData[1] << 8) + rawData[0], 16)
    magData.magnetic_field.y = UintToInt((rawData[3] << 8) + rawData[2], 16)
    magData.magnetic_field.z = UintToInt((rawData[5] << 8) + rawData[4], 16)

    magData.magnetic_field.x *= magCoefficient
    magData.magnetic_field.y *= magCoefficient
    magData.magnetic_field.z *= magCoefficient

    MagneticFieldPub.publish(magData)


# 板载adc
def AdcDataProcess(rawData):
    battData = BatteryState()
    battData.voltage = rawData[0] / 10.0
    battData.present = True
    BatteryStatePub.publish(battData)


# 板载按键
def KeyDataProcess(rawData):
    status = 0
    for i in range(5):
        if rawData[i]:
            status |= (1 << i)
    if status > 0:
        ChannelFloat32Pub.publish(name="KeyStatus", values=[status])


# 板载led
def LedDataProcess(rawData):
    ChannelFloat32Pub.publish(name="LedStatus", values=[(rawData[0] << 8) + rawData[1]])


# 板载测距
def RangeDataProcess(rawData):
    RangePub.publish(range=(rawData[1] << 8) + rawData[0])


# 板载传感器
def BaseBoardDataProcess(rawData):
    ledStatus = []
    keyStatus = []
    imuData = []
    MagneticData = []
    adcData = []
    rangeData = []
    startAddr = 24

    offset, lenght = 26, 2
    for i in range(offset, offset + lenght):
        ledStatus.append(rawData[i - startAddr])
    LedDataProcess(ledStatus)

    offset, lenght = 30, 5
    for i in range(offset, offset + lenght):
        keyStatus.append(rawData[i - startAddr])
    KeyDataProcess(keyStatus)

    offset, lenght = 38, 12
    for i in range(offset, offset + lenght):
        imuData.append(rawData[i - startAddr])
    ImuDataProcess(imuData)

    offset, lenght = 50, 1
    for i in range(offset, offset + lenght):
        adcData.append(rawData[i - startAddr])
    AdcDataProcess(adcData)

    offset, lenght = 51, 6
    for i in range(offset, offset + lenght):
        MagneticData.append(rawData[i - startAddr])
    MagneticDataProcess(MagneticData)

    offset, lenght = 59, 2
    for i in range(offset, offset + lenght):
        rangeData.append(rawData[i - startAddr])
    RangeDataProcess(rangeData)


# 光照度传感器
def IlluminanceDataProcess(rawData):
    IlluminancePub.publish(illuminance=rawData[0], variance=0)


# 温湿度传感器
def HumitureDataProcess(rawData):
    temperature = rawData[0] + rawData[1]
    humidity = rawData[2] + rawData[3]
    TemperaturePub.publish(temperature=temperature, variance=0)
    HumidityPub.publish(relative_humidity=humidity, variance=0)


# 人体红外传感器
def InfraredDataProcess(rawData):
    ChannelFloat32Pub.publish(name="InfraredStatus", values=[rawData[0]])


# 颜色传感器
def ColorDataProcess(rawData):
    ChannelFloat32Pub.publish(name="ColorValue", values=[rawData[0]])


# 火焰传感器
def FireDataProcess(rawData):
    ChannelFloat32Pub.publish(name="FireStatus", values=[rawData[0]])


# 气体传感器
def GasDataProcess(rawData):
    ChannelFloat32Pub.publish(name="GasStatus", values=[rawData[0]])


# 触摸传感器
def TouchDataProcess(rawData):
    ChannelFloat32Pub.publish(name="TouchStatus", values=[rawData[0]])


def SensorRawDataProcess():
    while (len(SensorList) > 0):
        sensor = SensorList.pop(0)
        if SensorIdNameDict[sensor.id] == 'baseBoard':
            BaseBoardDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'illuminance':
            IlluminanceDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'humiture':
            HumitureDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'infrared':
            InfraredDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'color':
            ColorDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'fire':
            FireDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'gas':
            GasDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'touch':
            TouchDataProcess(sensor.getData())
        else:
            pass


# ----------------------------------------------------------------------------------------


def LoadYamlFile(filePath):
    yamlFile = open(filePath)
    sensorNameIdDoc = yaml.load(yamlFile)

    for name in sensorNameIdDoc["sensorNameID"]:
        SensorIdNameDict[sensorNameIdDoc["sensorNameID"][name]] = name
    rospy.loginfo(SensorIdNameDict)


def SensorRawDataCallback(rawData):
    # rospy.loginfo(rawData.sensorData)
    sensorCount = rawData.sensorCount
    # dataOverallLength = rawData.dataLength

    dataIndex = 0
    sensorIdList = list(rawData.sensorReadID)
    for i in range(sensorCount):
        sensorIdList[i] = ord(sensorIdList[i])
        if sensorIdList[i] in SensorIdNameDict:
            SensorList.append(
                Sensor(SensorIdNameDict[sensorIdList[i]], sensorIdList[i], rawData.sensorStartAddress[i], rawData.sensorReadLength[i],
                       rawData.sensorData[dataIndex:(dataIndex + rawData.sensorReadLength[i])]))
        else:
            rospy.loginfo('Undefined id')
        dataIndex += rawData.sensorReadLength[i]

    SensorRawDataProcess()


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node('SensorHubNode', anonymous=True, log_level=rospy.INFO)  # DEBUG INFO ERROR WARN
        print('Starting SensorHubNode node')

        LoadYamlFile(SensorNameIDFilePath)

        rospy.Subscriber("MediumSize/BodyHub/SensorRaw", SensorRawData, SensorRawDataCallback)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down SensorHub node.")
