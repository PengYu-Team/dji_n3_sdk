# dji_n3_sdk

### 快速上手

- 下载代码

  ```shell
  git clone https://gitee.com/SYSU-Unmanned-System-Group/dji_n3_sdk.git
  ```
  
- 安装Onboard-SDK

  ```shell
  cd dji_n3_sdk/Modules/Onboard-SDK
  mkdir build
  cd build
  cmake ..
  sudo make -j7 install
  ```
  
- 可能需要安装依赖项

  ```shell
  sudo apt-get install ros-melodic-vrpn
  ```
  
- 编译整个项目

  ```shell
  cd dji_n3_sdk/
  catkin_make
  ```

- 使用

  ```shell
  ## 使用SSH或者远程桌面连接机载电脑
  ## 若机载电脑用户名有变动，请修改对应.sh文件
  窗口1：
  source ./dji_n3_sdk/Scripts/vicon_all.sh
  窗口2：
  source ./dji_n3_sdk/Scripts/terminal_control.sh
  遥控器操作：
  切A档，然后切脚架，飞机将起飞至固定高度（默认为0.5米）
  可通过窗口2进行简易的位置控制
  ```

### N3飞控使用说明

- **N3飞控官方网址：**https://www.dji.com/cn/n3/info#specs

- **DJI 开发者账户注册与绑定**

  - 注册：https://developer.dji.com/cn/

  - 注册登陆后，在个人中心找到自己专属的app_id和enc_key

    <img src="https://gitee.com/potato77/pic/raw/master/image-20210517200443584.png" alt="image-20210517200443584" style="zoom:67%;" />

  - 将生成的app_id和enc_key，加入djiros.launch文件中

- **DJI ASSITANT 2进行初始设置**

  - DJI ASSITANT 2 for N3下载地址：https://www.dji.com/cn/n3/info#downloads
  - 注意启动时，请勾选如下数据授权

  <img src="https://gitee.com/potato77/pic/raw/master/image-20210517200148444.png" alt="image-20210517200148444" style="zoom: 67%;" />

  - 刷写N3飞控版本：v1.7.6.0
  - SDK设置：启动API控制
  - 无人机姿态环抖动，调大动力带宽
  - 检查电调、电机转向

- **遥控器设置**

  - 需使用双回中遥控器
  - U，设置为REV，设置为模式切换，即 P\S\A （P为定点，A模式下才进入sdk控制）
  - 起落架通道，不REV，设置为自动轨迹trigger
  - 具体看截图

- **N3与机载电脑连接**

  - API接口，从左至右，空着、GND、RX、TX

  <img src="https://gitee.com/potato77/pic/raw/master/connection.png" alt="connection" style="zoom: 50%;" />

- **SDK设置**

  <img src="https://gitee.com/potato77/pic/raw/master/configuration.png" alt="configuration" style="zoom:67%;" />

- **使用机载电脑进行初次激活**

  - 需连接电脑激活，启动DJI ASSITANT 2，登录相应账号

    ```
    ## 启动如下文件两次
    roslaunch djiros djiros.launch
    ## 知道echo相应话题能够收到N3飞控回传的数据信息，代表激活成功
    rostopic echo /djiros/imu
    ```

### 代码说明

- Onboard-SDK是DJI官方提供的3.7版本
- controller/djiros是港科大改动过的Onboard-SDK-ROS，部分控制话题被修改
- vrpn_clinet_ros是官方源码编译，注意修改launch文件中的动捕频率

##### controller/n3ctrl说明

- controller/n3ctrl是控制状态机+位置环控制器代码

- 项目入口：n3ctrl_node.cpp，订阅发布话题如下

  ```
  ## 订阅以下话题
   ~imu  : [sensor_msgs/IMU]               IMU message from djiros.
   ~odom  : [nav_msgs/Odometry]              里程计消息
   ~joy   : [sensor_msgs/Joy]              RC message form djiros
   ~cmd : [quadrotor_msgs/PositionCommand] Position command message from the planner.
  ## 发布以下话题
   ~traj_start_trigger  : [geometry_msgs/PoseStamped]        A trigger message is sent when enter command mode.
   ~desire_pose : [geometry_msgs/PoseStamped]     The desired pose of the controller.
   ~ctrl : [sensor_msgs/Joy] The output of the control signal. The axes are the roll, pitch, thrust, yaw or yaw rate, thrust mode (if thrust mode > 0, thrust = 0~100%; if mode < 0, thrust = -? m/s ~ +? m/s), yaw mode (if yaw_mode > 0, axes[3] = yaw; if yaw_mode < 0, axes[3] = yaw_rate) respectively. The roll, pitch and yaw or yaw rate are in FRD frame. 
  ```

  - 新增加了一个订阅话题 ~/point_cmd

- n3ctrl中的状态切换

  - 相应文件:
    - N3CtrlFSM.cpp
    - N3CtrlFSM_state.cpp
    - N3CtrlFSM_control.cpp
  - 可参考下图弄清楚n3ctrl的**状态切换逻辑**

![image-20210517202212708](https://gitee.com/potato77/pic/raw/master/image-20210517202212708.png)

- 位置控制的核心的代码在controller.cpp
  - Controller::update() 更新控制量
  - Controller::publish_ctrl() 发布控制量
- 控制参数文件：$(find n3ctrl)/config/ctrl_param_fpv.yaml
  - 悬停油门建议选择小一些（质量和悬停油门可能要根据飞机进行调整）
  - 其他参数建议不改变
- 位置控制交互节点
  - terminal_control.cpp
  - 依次输入x,y,z三点位置，用于位置控制，单位:米

**controller/n3ctrl_sysu说明**

- vicon_odom.cpp可直接订阅动捕的位置计算速度，然后发布odom信息至N3控制器
- vicon_odom_without_vel.cpp可根据动捕的位置计算速度，然后发布odom信息至N3控制器

### 更多参考资料

https://github.com/dji-sdk/Onboard-SDK/tree/3.7

https://github.com/dji-sdk/Onboard-SDK-ROS/tree/3.7

http://wiki.ros.org/dji_sdk