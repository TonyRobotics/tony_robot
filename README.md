本驱动包支持Tony Robotics公司轮式底盘，如TR10、TR30。

可到[cn.robostore.me](http://cn.robostore.me)网上商城进行选购[轮式底盘TR10](http://cn.robostore.me/goods-details/104601512459932935)，[电机驱动TRD](http://cn.robostore.me/goods-details/103801510130627468)，[激光雷达](http://cn.robostore.me)，[深度相机](http://cn.robostore.me/goods-details/104301510191123102)等。

安装步骤为:

### 1. 准备工作

创建ROS工作区，下载源代码，并进行编译。

```
$ mkdir -p ~/ros_ws/src
$ cd ~/ros_ws/src
$ git clone https://github.com/tonyrobotics/tony_robot
$ cd ~/ros_ws
$ catkin build #或执行 catkin_make
$ echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### 2. 电机驱动测试

测试型号为[Tony Robotics TRD](http://cn.robostore.me/goods-details/103801510130627468)。

#### 2.1 添加设备udev规则

```
$ cd ~/ros_ws/src/tony_robot/trd_diff_controller
$ sudo cp motor_trd.rules /etc/udev/rules.d/
$ sudo service udev reload
$ sudo service udev restart
```

#### 2.2 启动及测试

将TRD连接至USB端口，执行以下指令:

```
$ roslaunch trd_diff_controller trd_diff_controller.launch
```

打开新窗口，启动控制脚本:

```
$ cd ~/ros_ws/src/tony_robot/trd_diff_controller/scripts
$ python keyboard_teleop.py
```

#### 2.3 控制

此时，可通过“i”、“,”、“j”、“l”、“k”键分别控制底盘前进、后退、左转、右转和停止。

#### 3. 激光雷达测试

测试型号为[镭神LS01D](http://cn.robostore.me/)。

#### 3.1 添加设备udev规则

```
$ cd ~/ros_ws/src/tony_robot/lslaser
$ sudo cp leishen.rules /etc/udev/rules.d/
$ sudo service udev reload
$ sudo service udev restart
```

#### 3.2 启动及测试

将激光雷达连接至USB端口，执行以下指令:

```
$ roslaunch ls01c leishen.launch
```

打开新窗口，打印激光数据:

```
$ rostopic echo /scan
```

若看到命令行输出激光数据，则证明工作正常。

### 4. 深度相机测试

测试型号为[奥比中光Orbbec Astra](http://cn.robostore.me/goods-details/104301510191123102)。

#### 4.1 打开终端，执行以下命令安装ROS驱动包

```
$ sudo apt-get install ros-$ROS_DISTRO-astra-camera ros-$ROS_DISTRO-astra-launch
```

#### 4.2 添加设备udev规则

设置前，需要将深度相机USB接口拔掉。然后，在命令行依次执行以下命令。

```
$ cd /etc/udev/rules.d
$ sudo wget https://raw.githubusercontent.com/orbbec/ros_astra_camera/master/56-orbbec-usb.rules
$ sudo service udev reload
$ sudo service udev restart
```

#### 4.3 启动

将设备连接至USB端口，执行以下指令。

```
$ roslaunch astra_launch astra.launch
```

或在自己的launch文件中引用。在launch文件中可以下列方式引用。

```xml
<launch>
  <include file="$(find astra_launch)/launch/astra.launch">
  </include>
</launch>
```

Launch启动后，可在命令行启动rqt_image_view浏览图像。在命令行执行：

$ rqt_image_view

