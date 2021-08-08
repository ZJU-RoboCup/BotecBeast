# sensorhub Package

## 概述
用于将bodyhub包发布的外接传感器的原始数据，转换为ROS标准数据格式。
## 如何运行
```
rosrun sensorhub SensorHubNode.py
```
## 订阅的话题
- /MediumSize/BodyHub/SensorRaw (bodyhub::SensorRawData)  
bodyhub节点发布的外接传感器的原始数据。消息定义
  ```
  uint8[]   sensorReadID
  uint16[]  sensorStartAddress
  uint16[]  sensorReadLength
  int32[]   sensorData
  uint8     sensorCount
  uint8     dataLength
  ```
  sensorReadID外接传感器id列表，sensorStartAddress外接传感器起始地址列表，sensorReadLength外接传感器数据长度列表，sensorData外接传感器原始数据列表，sensorCount外接传感器个数，dataLength数据总长。
## 发布的话题
- /MediumSize/SensorHub/Imu ([Imu][sensor_msgs/Imu])  
imu传感器姿态数据，包含三轴角速度、三轴加速度的物理数值。
- /MediumSize/SensorHub/MagneticField ([MagneticField][sensor_msgs/MagneticField])  
磁场传感器测得的磁场强度的物理值，包含在xyz三个方向的分量。
- /MediumSize/SensorHub/BatteryState ([BatteryState][sensor_msgs/BatteryState])  
机器人电源状态，详细参考[sensor_msgs/BatteryState][sensor_msgs/BatteryState]。
- /MediumSize/SensorHub/Range ([Range][sensor_msgs/Range])  
测距传感器测量的物理数值。
- /MediumSize/SensorHub/sensor_CF1 ([ChannelFloat32][sensor_msgs/ChannelFloat32])  
带通道标识的32位浮点型数据，用来表示部分传感器测量的状态。
- /MediumSize/SensorHub/Illuminance ([Illuminance][sensor_msgs/Illuminance])  
光照度传感器测量的环境光强的物理数值。
- /MediumSize/SensorHub/Temperature ([Temperature][sensor_msgs/Temperature])  
温度传感器测量的环境温度的物理数值。
- /MediumSize/SensorHub/Humidity ([RelativeHumidity][sensor_msgs/RelativeHumidity])  
湿度传感器测量的环境湿度的百分比。
## 服务
无
## 参数
无
## 附录
[sensor_msgs/Imu]: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
[sensor_msgs/MagneticField]: http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html
[sensor_msgs/BatteryState]: http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html
[sensor_msgs/Range]: http://docs.ros.org/api/sensor_msgs/html/msg/Range.html
[sensor_msgs/ChannelFloat32]: http://docs.ros.org/api/sensor_msgs/html/msg/ChannelFloat32.html
[sensor_msgs/Illuminance]: http://docs.ros.org/api/sensor_msgs/html/msg/Illuminance.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
[sensor_msgs/RelativeHumidity]: http://docs.ros.org/api/sensor_msgs/html/msg/RelativeHumidity.html