# 快速开始

## 1 下载编译
~~~
下载
cd ~/ 
git clone https://gitee.com/lulese/uavoffbard-frame_ws.git
cd UAVOffbardFrame_ws/

//编译
catkin_make
//更新环境变量，可以把刚才编译的功能包路径添加到环境中，那么本终端就可以运行本工作空间的功能包，也可以在~/.bashrc里面添加source语句，那么所有终端都可以使用本工作空间功能包
source ./devel/setup.bash
~~~
## 2 运行

### 2.1 启动环境（开一个终端）

单机：
~~~
roslaunch px4 mavros_posix_sitl.launch 
~~~
多机：
~~~
roslaunch px4 multi_uav_mavros_sitl.launch 
~~~

如果想加载一个室内环境在gazebo上：
~~~
cp ~/UAVOffbardFrame_ws/src/world/room.world  ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
cp ~/UAVOffbardFrame_ws/src/world/mavros_posix_sitl_room.launch ~/PX4-Autopilot/launch/
~~~

~~~
roslaunch px4 mavros_posix_sitl_room.launch 
~~~
![输入图片说明](src/world/room.png)

### 2.2 程控（两个launch二选一）（另开一个终端）
单机：
~~~
//运行点控程序(需要自己解锁、切到offboard)
cd ~/UAVOffbardFrame_ws/ && source ./devel/setup.bash
roslaunch offboard position_control.launch

//跑正方形或者圆形(不需要自己解锁、切到offboard，注意实物飞行慎用这个程序，防止意外发生，还是写成手动解锁好一点)
cd ~/UAVOffbardFrame_ws/ && source ./devel/setup.bash
roslaunch offboard autoarming_control.launch 
~~~

多机：
~~~
roslaunch offboard autoarming_Mult.launch
~~~

