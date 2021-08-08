#!/usr/bin/env python

import rospy

from sensor_msgs.msg import ChannelFloat32, MagneticField
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity, Range, Illuminance
from bodyhub.srv import SrvInstWrite, SrvTLSstring


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


def acceleration_sensor(axis):
	"""
	Get data from acceleration sensor

	Args:
	  axis: x, y or z

	Returns:
	  acceleration value of axis
	"""

	sensor_name = "baseBoard"
	reg_addr = 24
	data_len = 56
	regist_sensor(sensor_name, reg_addr, data_len)

	#get sensor data
	data = rospy.wait_for_message('MediumSize/SensorHub/Imu', Imu, 2)
	acceleration = data.linear_acceleration
	if axis == "x":
		result = acceleration.x
	elif axis == "y":
		result = acceleration.y
	else:
		result = acceleration.z

	delete_sensor(sensor_name)
	return result

def gyroscope_sensor(axis):
	"""
	Get data from gyroscope sensor

	Args:
	  axis: x, y or z

	Returns:
	  gyroscope value of axis
	"""

	sensor_name = "baseBoard"
	reg_addr = 24
	data_len = 56
	regist_sensor(sensor_name, reg_addr, data_len)

	#get sensor data
	data = rospy.wait_for_message('MediumSize/SensorHub/Imu', Imu, 2)
	angular_velocity = data.angular_velocity
	if axis == "x":
		result = angular_velocity.x
	elif axis == "y":
		result = angular_velocity.y
	else:
		result = angular_velocity.z

	delete_sensor(sensor_name)
	return result



def posture_sensor(axis):
	"""
	Get data from posture sensor

	Args:
	  axis: x, y or z

	Returns:
	  posture value of axis
	"""

	return 0.0


def distance_sensor(unit):
	"""
	Get data from distance sensor

	Args:
	  unit: cm or mm

	Returns:
	  distance value in cm / mm
	"""

	sensor_name = "baseBoard"
	reg_addr = 24
	data_len = 56
	regist_sensor(sensor_name, reg_addr, data_len)

	data = rospy.wait_for_message("MediumSize/SensorHub/Range", Range, 2)
	distance = data.range
	# transfer sensor data to target unit
	if unit == "cm":
		result = distance / 10.0
	else:
		result = distance

	delete_sensor(sensor_name)
	return result


def pressure_sensor(foot):
	"""
	Get data from pressure sensor

	Args:
	  foot: left or right

	Returns:
	  pressure value on left / right foot
	"""

	sensor_name = "mpu6050"
	reg_addr = 38
	data_len = 16
	regist_sensor(sensor_name, reg_addr, data_len)

	data = rospy.wait_for_message("MediumSize/SensorHub/FootPressure", ChannelFloat32, 2)
	pressure = data.values
	if foot == "left":
		result = pressure[0]
	else:
		result = pressure[1]

	delete_sensor(sensor_name)
	return result


def temperature_sensor():
	"""
	Get data from temperature sensor

	Returns:
	  temperature data
	"""

	sensor_name = "humiture"
	reg_addr = 26
	data_len = 4
	regist_sensor(sensor_name, reg_addr, data_len)

	# get sensor data
	data = rospy.wait_for_message('MediumSize/SensorHub/Temperature', Temperature, 2)
	temperature = data.temperature

	delete_sensor(sensor_name)
	return temperature


def humidity_sensor():
	"""
	Get data from humidity sensor

	Returns:
	  humidity data
	"""

	sensor_name = "humiture"
	reg_addr = 26
	data_len = 4
	regist_sensor(sensor_name, reg_addr, data_len)

	# get sensor data
	data = rospy.wait_for_message('MediumSize/SensorHub/Humidity', RelativeHumidity, 2)
	humidity = data.relative_humidity

	delete_sensor(sensor_name)
	return humidity


def geomagnetic_sensor(axis):
	"""
	Get data from geomagnetic sensor

	Returns:
	  geomagnetic data
	"""
	sensor_name = "baseBoard"
	reg_addr = 24
	data_len = 56
	regist_sensor(sensor_name, reg_addr, data_len)

	#get sensor data
	data = rospy.wait_for_message('MediumSize/SensorHub/MagneticField', MagneticField, 2)
	magnetic_field = data.magnetic_field
	if axis == "x":
		result = magnetic_field.x
	elif axis == "y":
		result = magnetic_field.y
	else:
		result = magnetic_field.z

	delete_sensor(sensor_name)
	return result

def fire_sensor():
	"""
	Fire detect sensor

	Returns:
	  true / false
	"""

	sensor_name = "mpu6050"
	reg_addr = 38
	data_len = 16
	regist_sensor(sensor_name, reg_addr, data_len)

	data = rospy.wait_for_message("MediumSize/SensorHub/sensor_Fire", ChannelFloat32, 2)
	res = data.values
	result = False
	if res[0]:
		result = True

	delete_sensor(sensor_name)
	return result


def human_sensor():
	"""
	Human detect sensor

	Returns:
	  true / false
	"""

	sensor_name = "mpu6050"
	reg_addr = 38
	data_len = 16
	regist_sensor(sensor_name, reg_addr, data_len)

	data = rospy.wait_for_message("MediumSize/SensorHub/sensor_PIR", ChannelFloat32, 2)
	res = data.values
	result = False
	if res[0]:
		result = True

	delete_sensor(sensor_name)
	return result


def touch_sensor():
	"""
	Touch detect sensor

	Returns:
	  true / false
	"""

	sensor_name = "touch"
	reg_addr = 26
	data_len = 1
	regist_sensor(sensor_name, reg_addr, data_len)

	data = rospy.wait_for_message("MediumSize/SensorHub/sensor_Touch", ChannelFloat32, 2)
	res = data.values
	result = False
	if res[0]:
		result = True

	delete_sensor(sensor_name)
	return result

def illuminance_sensor():
	"""
    Get data from illuminance sensor

    Returns:
      illuminance data
    """

	sensor_name = "illuminance"
	reg_addr = 26
	data_len = 1
	regist_sensor(sensor_name, reg_addr, data_len)

	data = rospy.wait_for_message("MediumSize/SensorHub/Illuminance", Illuminance, 2)
	result = data.illuminance

	delete_sensor(sensor_name)
	return result

if __name__ == '__main__':
	rospy.init_node("sensor_test", anonymous=True)

	try:
		data = acceleration_sensor("y")
		print(data)
	except Exception as err:
		rospy.logerr(err)