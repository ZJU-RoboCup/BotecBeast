# ActExecPackege
## 概述
动作执行功能包，可执行机器人自定义动作。
## 如何运行
在终端输入以下命令，启动节点
```
rosrun actexecpackage ActExecPackageNode.py
```
使用上级节点通过`/MediumSize/ActPackageExec/StateJump`服务占用启动的节点，如下所示
``` 
rosservice call /MediumSize/ActPackageExec/StateJump "masterID: 1
stateReq: 'setStatus'"
```
返回`stateRes: 22`占用成功，否则占用失败。  
上级节点通过`/MediumSize/ActPackageExec/actNameString`服务，运行机器人自定义动作，如下所示（运行前，需要将trajectoryPlan_node.py中的DIRECTLY_RUN变量改为False）
```
rosservice call /MediumSize/ActPackageExec/actNameString "actNameReq: 'rosrun actexecpackage trajectoryPlan_node.py'"
```
动作执行完成，使用以下面命令释放动作执行节点
```
rosservice call /MediumSize/ActPackageExec/StateJump "masterID: 1
stateReq: 'reset'"
```
## 发布的话题
- /MediumSize/ActPackageExec/Status ([std_msgs/UInt16][std_msgs/UInt16])  
动作执行节点当前状态的数值。状态定义
    ```
    self.StateStyle = {
        'init' : 20,
        'preReady' : 21,
        'ready' : 22,
        'running' : 23,
        'pause' : 24,
        'stoping' : 25,
        'error' : 26,
    }
    ```
## 提供的服务
- /MediumSize/ActPackageExec/GetStatus (SrvString)  
返回节点状态机的状态；返回关节指令队列的长度。服务定义
  ```
  string str
  ---
  string data
  uint32 poseQueueSize
  ```
  str请求字符串，可任意设置，不能为空，data状态的字符串，poseQueueSize关节指令度列的长度。
- /MediumSize/ActPackageExec/StateJump (SrvState)  
节点状态机跳转。服务定义
  ```
  uint8 masterID
  string stateReq
  ---
  int16 stateRes
  ```
  masterID占用节点的id，stateReq状态跳转的控制字符串，stateRes操作返回的状态数值。
- /MediumSize/ActPackageExec/actNameString (SrvActScript)  
需要执行的动作指令字符。服务定义
    ```
    string actNameReq
    ---
    string actResultRes
    ```
    actNameReq指令名称，actResultRes执行结果。
## TrajectoryPlanning类
`TrajectoryPlan.py`文件为使用自动贝塞尔做动作轨迹规划的代码。`trajectoryPlan_node.py`文件为示例程序。  
TrajectoryPlanning类中，若i为任意一个目标值，则(i-1)为初始值，(i+1)为下一个目标值，(i-1)和i值之间的轨迹为第i段轨迹。生成第i段轨迹时，需要i-1、i、i+1三个目标值。
#### 接口简介
- `def __init__(self, numberOfTra=22, daltaX=10.0)`  
类构造函数  
参数@numberOfTra::规划的轨迹组数（关节数）  
参数@daltaX:轨迹插值的时间间隔，单位ms
- `def setDaltaX(self, daltaX)`  
设置轨迹插值的间隔  
参数@daltaX:间隔时间，单位ms
- `def setInterval(self, v)`  
设置目标值之间的间隔  
参数@v:间隔时间，单位ms
- `def planningBegin(self, firstGroupValue, secondGroupValue)`  
开始轨迹生成的准备，传入初始值和第一组目标值  
参数@firstGroupValue:初始值  
参数@secondGroupValue:第一组目标值
- `def planning(self, nextGroupValue)`  
传入下一组目标值（i+1），并返回上两组目标值之间（i-1和i）的第i段轨迹  
参数@nextGroupValue:下一组目标值  
返回值:list类型，上两组目标值生成的轨迹
- `def planningEnd(self)`  
生成最后一段轨迹（i和i+1之间）  
返回值:list类型，生成的最后一段轨迹列表
#### 示例代码
```
# 创建一个实例，规划22段曲线，轨迹点插值间隔为10ms
tpObject = TrajectoryPlanning(22,10.0)

# 设置目标点之间的时间间隔为1000ms
tpObject.setInterval(1000.0)
# 传入初始值和第一组目标值
tpObject.planningBegin(poseList[0], poseList[1])

# 修改目标点之间的时间间隔为1500ms
tpObject.setInterval(1500.0) 
# 传入下一组目标值，并返回上两组目标值之间的轨迹
trajectoryPoint = tpObject.planning(poseList[2])

# 修改目标点之间的时间间隔为2000ms，若不调用此函数，目标点的间隔保持为1500ms
tpObject.setInterval(2000.0) 
# 传入下一组目标值，并返回上两组目标值之间的轨迹
trajectoryPoint = tpObject.planning(poseList[3])

...

# 返回最后一段轨迹
trajectoryPoint = tpObject.planningEnd()
```
## 附录
[std_msgs/UInt16]: http://docs.ros.org/api/std_msgs/html/msg/UInt16.html