# 标准消息头
std_msgs/Header header

# 无人机状态
uint8 status
uint8 MODE_IDLE=0
uint8 MODE_TAKEOFF=1
uint8 MODE_HOVER=2
uint8 MODE_MISSION=3
uint8 MODE_LAND=4
uint8 MODE_EMERGENCY=5

# 位置和方向
geometry_msgs/Pose current_pose
geometry_msgs/Twist current_velocity

# 传感器数据
float32 battery_voltage
float32 battery_percentage