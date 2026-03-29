# 1. 编译

~~~
mkdir -p workspace/src
git clone https://gitee.com/lulese/my_utils.git
cd workspace/ 
catkin_make
source ./devel/setup.bash
~~~

# 2. 运行

## 2.1 驱动mjepg的摄像头：

~~~
cd ~/workspace/src/topic_tf_tran/script/
python3 mjpeg_pub.py
~~~
