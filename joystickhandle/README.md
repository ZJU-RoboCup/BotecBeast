# joystick_handle Package

## 概述
解析手柄数据，转换为机器人步态控制接口数据，控制机器人行走。
## 如何运行
下载安装游戏手柄驱动库
```
sudo apt-get install ros-kinetic-joystick-drivers
```
运行节点
```
roslaunch joystick_handle joystickHandleLaunch.launch
```
## 订阅的话题
- /requestGaitCommand ([std_msgs/Bool][std_msgs/Bool])  
步态节点发布的请求步态命令。步态节点通过`gaitCommand`消息接受步态命令，当接收到的步态命令队列执行后剩余或者小于2个时，会发送此话题请求新的步态。
- /Joy ([sensor_msgs/Joy][sensor_msgs/Joy])  
遥控节点发布的遥控数据。操纵杆或遥控的每个按钮和轴的当前状态，参考通用Linux游戏杆的ROS驱动程序[joystick_drivers][joystick_drivers]。
## 发布的话题
- /gaitCommand ([std_msgs/Float64MultiArray][std_msgs/Float64MultiArray])  
步态控制。

  **机器人行走基本原理**

  机器人行走最小动作可看作为任意脚的一个跨步，向前行走时，从开始到结束可简化为三个阶段
  - 抬脚阶段，从站立到跨步（计算步数）
  - 跨步阶段，执行行走步态（计算步数）
  - 收脚阶段，从跨步到站立（不计算步数，自动生成）

  左右行走和转弯时慢，跨步的偶数步为收脚步态，可简化为以下两个阶段的循环
  - 抬脚阶段，从站立到跨步（计算步数）
  - 收脚阶段，从跨步到站立（计算步数）

  只考虑机器人行走的情况，将地面看作一个平面，以机器人前方为x轴正方向，机器人右边为y轴正方向创建直角坐标系，这样要控制机器人的运动方向只需要给对应的(x,y)坐标即可。若需要机器人转动，以机器人为原点，从上往下看逆时针为正做极坐标系，这样要控制机器人转动只需要给对应的角度即可。

  **步态接口数据含义**

  步态定义格式定义如下
  ```
  std_msgs::Float64MultiArra GaitCommandMsg;
  GaitCommandMsg.data.resize(3);
  ```
  GaitCommandMsg是一个float64类型数组，长度初始化为3，三个参数分别代表
  - 跨步时向直角坐标系x轴方向移动的距离
  - 跨步时向直角坐标系y轴方向移动的距离
  - 跨步时在极坐标转动的角度

  每一组参数对应一个跨步，机器人每跨一步就需要一组参数，应当控制参数不能超过实际跨步范围。 
## 服务
无
## 参数
无
## 附录
[std_msgs/Bool]: http://docs.ros.org/api/std_msgs/html/msg/Bool.html
[sensor_msgs/Joy]: http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html
[std_msgs/Float64MultiArray]: http://docs.ros.org/api/std_msgs/html/msg/Float64MultiArray.html
[joystick_drivers]: http://wiki.ros.org/joy