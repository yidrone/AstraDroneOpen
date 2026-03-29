import numpy as np
import open3d as o3d

def create_rotation_matrix(alpha, beta, gamma):
    """创建按XYZ顺序的旋转矩阵"""
    # X轴旋转
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])
    
    # Y轴旋转
    Ry = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])
    
    # Z轴旋转
    Rz = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]
    ])
    
    # 组合旋转 (注意顺序: 先X, 再Y, 最后Z)
    R = Rz @ Ry @ Rx
    return R

def create_transform_matrix(alpha, beta, gamma, tx, ty, tz):
    """创建完整的变换矩阵 (旋转+平移)"""
    R = create_rotation_matrix(alpha, beta, gamma)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T

# 读取PCD文件
# pcd = o3d.io.read_point_cloud("/home/luli/sim_env_ws/src/env_map/car_test/car_test.pcd")
pcd = o3d.io.read_point_cloud("/home/luli/sim_env_ws/src/env_map/gully_test/gully_test1.pcd")
# pcd = o3d.io.read_point_cloud("/home/luli/sim_env_ws/src/env_map/pre_test/pre_test.pcd")

# 设置旋转角度 (弧度) 和平移量
alpha = np.pi/2      # X轴旋转90度
beta = 0             # Y轴旋转0度
# gamma = np.pi/2        # Z轴旋转180度
gamma = -np.pi/2        # Z轴旋转180度
tx, ty, tz = 0, 0, 0  # 不进行平移

# 创建变换矩阵
transform = create_transform_matrix(alpha, beta, gamma, tx, ty, tz)

# 应用变换
pcd.transform(transform)

# 保存修改后的点云
# o3d.io.write_point_cloud("/home/luli/sim_env_ws/src/env_map/car_test/car_test_rota.pcd", pcd)
o3d.io.write_point_cloud("/home/luli/sim_env_ws/src/env_map/gully_test/gully_test1_rota.pcd", pcd)
# o3d.io.write_point_cloud("/home/luli/sim_env_ws/src/env_map/pre_test/pre_test_rota.pcd", pcd)