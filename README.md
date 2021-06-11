# dji_n3_sdk

### 快速上手

- 下载代码

  ```c
  git clone https://gitee.com/SYSU-Unmanned-System-Group/dji_n3_sdk.git
  ```
  
- 安装Onboard-SDK

  ```c
  cd dji_n3_sdk/src/Onboard-SDK-3.8
  mkdir build
  cd build
  cmake ..
  sudo make -j7 install
  ```
  
- 可能需要安装依赖项

  ```c
  sudo apt-get install ros-melodic-vrpn
  ```
  
- 编译整个项目

  ```
  cd dji_n3_sdk/
  catkin_make
  ```

- 使用

  ```
  ./all.sh
  ```

### 使用说明
- N3飞控版本：v1.7.6.0

- 遥控器设置
  - U，设置为REV，设置为模式切换，即 P\S\A （P为定点，A模式下才进入sdk控制）
  - 起落架通道，不REV，设置为自动轨迹trigger
  - 具体看截图
- N3与机载电脑连接
  - API接口，从左至右，空着、GND、RX、TX
- SDK设置：启动API控制
- 无人机姿态环抖动，调大动力带宽
- 使用N3初始设置绑定的DJI账号中生成app_id和enc_key，加入djiros.launch文件中
  - 首次启动需连接电脑激活（启动launch文件2次）

### 参考资料

https://github.com/dji-sdk/Onboard-SDK/tree/3.8

https://github.com/dji-sdk/Onboard-SDK-ROS/tree/3.8

http://wiki.ros.org/dji_sdk