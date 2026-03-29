from siyiA8mini import siyisdk

# 1. 初始化SDK，连接相机
siyi = siyisdk.SIYISDK("192.168.1.25", 37260, 1024)

# 2. 【新增】执行自动回中，使相机回到中心位置
siyi.one_click_back()
siyi.get_device_workmode()
siyi.get_config_info()

#siyi.turn_to(0.0, 0.0)
# 3. 进入键盘手动控制模式（此时相机已回中）
# siyi.keep_turn()

# 4. 关闭连接
siyi.close()
