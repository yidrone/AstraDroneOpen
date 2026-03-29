# orin nano / nx 打开csi摄像头流程
## 启动配置CSI Connector界面
~~~
sudo /opt/nvidia/jetson-io/jetson-io.py
~~~

## 选择配置Jetson 24pin CSI Connector
选择：
~~~
Configure Jetson 24pin CSI Connecton
~~~

## 确认当前CSI Connector IO配置
下一步就行

## 选择满足需求的CSI Connector IO配置
~~~
选择对应的摄像头型号，Camera IMX219 Dual
~~~

## 保存CSI Connector IO配置
保存即可

## 确认保存，并重启生效CSI Connector IO配置
保存，并重启

## 任意键执行重启，一般回车
回车即可重启

## 测试
~~~
nvgstcapture
~~~

## RTSP 推流
在 `launch/csi_camera.launch` 中设置：
~~~xml
<param name="enable_rtsp" value="true"/>
<param name="rtsp_port" value="8554"/>
<param name="rtsp_mount" value="/csi"/>
<param name="rtsp_bitrate" value="2000"/>
<param name="rtsp_use_hardware_encoder" value="true"/>
~~~

启动后可在局域网内访问：
~~~
rtsp://<Jetson_IP>:8554/csi
~~~
