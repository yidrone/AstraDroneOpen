
# SIYISDK

针对 SIYI A8mini 云台相机开发的一款 Python SDK，方便调用云台功能。

## Usage
### 依赖项：pynput ，用于监测键盘输入
```bash
pip install pynput
```

### 1. 直接使用源代码

将整个项目 clone 到本地，进入 `test.py` 文件，更改对应的相机 IP 地址和端口，然后运行：

```bash
python test.py
```

### 2. 使用 pip 安装包（推荐）

从 [release](https://github.com/your-repo/releases) 中下载 `siyiA8mini-0.1.0-py3-none-any.whl` 到本地，然后运行：

```bash
pip install siyiA8mini-0.1.0-py3-none-any.whl
```

然后新建 `test.py`，写入示例程序（Example），运行：

## Example（test.py）

```python
# 引入包
from siyiA8mini import siyisdk

# 实例化
siyi_controler = siyisdk.SIYISDK("192.168.1.25", 37260, 1024)

# 启动键盘控制功能
siyi_controler.keep_turn()

# 结束控制
siyi_controler.close()
```

## Function

| 函数名                        | 参数                            | 解释                                                                                                                                       |
|-------------------------------|---------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| `one_click_down()`             | 无                              | 一键朝下，相机垂直转动 90°。                                                                                                               |
| `get_device_hardwareID()`      | 无                              | 获取设备硬件信息，通过控制台打印。                                                                                                          |
| `get_device_workmode()`        | 无                              | 获取设备运行状态，通过控制台打印。                                                                                                          |
| `keep_turn()`                  | 无                              | 进入键盘控制相机转动模式，`↑↓←→` 控制摄像头移动，`WSAD` 控制转动速度，`ESC` 退出控制模式。                                                |
| `one_click_back()`             | 无                              | 一键回中，将相机朝向归位。                                                                                                                  |
| `get_position()`               | 无                              | 获取相机当前姿态，通过控制台打印。                                                                                                          |
| `turn_to(yaw, pitch)`          | `yaw`: 横向转动角度；`pitch`: 纵向转动角度 | 将相机转动一定角度，`yaw`: -135.0 至 135.0；`pitch`: -90.0 至 25.0。                                                                         |
| `single_turn_to(angle, direction)` | `angle`: 转动角度；`direction`: 方向 | 控制相机单方向转动，`angle` 确定角度，`direction` 确定方向。                                                                                 |
| `get_config_info()`            | 无                              | 获取云台配置信息，通过控制台打印。                                                                                                          |
| `get_encode_info()`            | 无                              | 获取相机编码信息，通过控制台打印。                                                                                                          |
| `format_SDcard()`              | 无                              | 格式化 SD 卡。                                                                                                                              |
| `device_restart(camera_restart, gimbal_restart)` | `camera_restart`: 0 或 1；`gimbal_restart`: 0 或 1 | 控制相机和云台重启，通过设置参数 0 或 1，确定是否重启。                                                                                     |
| `close()`                      | 无                              | 结束控制，程序最后调用。                                                                                                                    |


## Author

- Institution: Shanghai Jiao Tong University
- Email: [zhangpengcheng@sjtu.edu.cn](mailto:zhangpengcheng@sjtu.edu.cn)
- GitHub: [https://github.com/Percylevent](https://github.com/Percylevent)



## License

This project is licensed under the MIT License - see the [LICENSE](./LICENSE) file for details.

