#!/usr/bin/env python3
"""
思翼相机自动变焦校准程序
自动测试相机变焦范围，建立光学倍数与相机等级的映射关系
"""

import sys
import os
import json
import numpy as np
from time import sleep
import matplotlib.pyplot as plt

# 添加父目录到路径以导入siyi_sdk
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK

class AutoZoomCalibrator:
    """
    自动变焦校准器
    自动测试相机变焦功能，建立准确的映射关系
    """
    
    def __init__(self, server_ip="192.168.1.25", port=37260):
        """初始化校准器"""
        self.server_ip = server_ip
        self.port = port
        self.cam = None
        self.connected = False
        
        # 校准数据
        self.calibration_data = {}  # 光学倍数 -> 相机等级
        self.test_results = []      # 测试过程数据
        self.min_level = None
        self.max_level = None
        
        print("=== 思翼相机自动变焦校准程序 ===")
        print("将自动测试相机变焦范围并建立映射关系")
        print("==================================")
    
    def connect(self):
        """连接相机"""
        print(f"正在连接相机 {self.server_ip}:{self.port}...")
        
        try:
            self.cam = SIYISDK(server_ip=self.server_ip, port=self.port, debug=False)
            
            if self.cam.connect(maxWaitTime=5.0):
                self.connected = True
                print("✓ 相机连接成功")
                return True
            else:
                print("✗ 相机连接失败")
                return False
                
        except Exception as e:
            print(f"✗ 连接相机时发生错误: {e}")
            return False
    
    def disconnect(self):
        """断开相机连接"""
        if self.cam and self.connected:
            self.cam.requestZoomHold()
            self.cam.disconnect()
            self.connected = False
            print("已断开相机连接")
    
    def find_min_max_levels(self):
        """查找最小和最大变焦等级"""
        print("\n--- 查找变焦范围 ---")
        
        if not self.connected:
            print("相机未连接")
            return False
        
        try:
            # 1. 变焦到最小
            print("正在变焦到最小...")
            for i in range(50):
                self.cam.requestZoomOut()
                sleep(0.05)
            self.cam.requestZoomHold()
            sleep(1)
            
            self.min_level = self.cam.getZoomLevel()
            print(f"最小变焦等级: {self.min_level:.2f}")
            
            # 2. 变焦到最大
            print("正在变焦到最大...")
            for i in range(50):
                self.cam.requestZoomIn()
                sleep(0.05)
            self.cam.requestZoomHold()
            sleep(1)
            
            self.max_level = self.cam.getZoomLevel()
            print(f"最大变焦等级: {self.max_level:.2f}")
            
            print(f"变焦范围: {self.min_level:.2f} - {self.max_level:.2f}")
            return True
            
        except Exception as e:
            print(f"查找变焦范围时出错: {e}")
            return False
    
    def auto_test_zoom_range(self, test_points=20):
        """自动测试变焦范围"""
        print(f"\n--- 自动测试变焦范围 ({test_points}个测试点) ---")
        
        if not self.connected or self.min_level is None or self.max_level is None:
            print("请先连接相机并查找变焦范围")
            return False
        
        try:
            # 清空测试结果
            self.test_results = []
            
            # 回到最小变焦
            print("回到最小变焦...")
            for i in range(50):
                self.cam.requestZoomOut()
                sleep(0.05)
            self.cam.requestZoomHold()
            sleep(1)
            
            # 计算步长
            total_range = self.max_level - self.min_level
            step_size = total_range / (test_points - 1)
            
            print(f"测试步长: {step_size:.3f}")
            print("开始自动测试...")
            
            for i in range(test_points):
                # 计算目标等级
                target_level = self.min_level + i * step_size
                
                # 变焦到目标位置
                self.zoom_to_level(target_level)
                
                # 获取实际等级
                actual_level = self.cam.getZoomLevel()
                sleep(0.2)  # 等待稳定
                
                # 记录测试点
                test_point = {
                    'index': i,
                    'target_level': target_level,
                    'actual_level': actual_level,
                    'optical_multiple': None,  # 需要用户输入
                    'timestamp': sleep  # 使用实际时间
                }
                
                self.test_results.append(test_point)
                
                print(f"测试点 {i+1}/{test_points}: 等级={actual_level:.2f}")
            
            print("自动测试完成!")
            return True
            
        except Exception as e:
            print(f"自动测试时出错: {e}")
            return False
    
    def zoom_to_level(self, target_level, tolerance=0.05, max_iterations=100):
        """控制相机变焦到指定等级"""
        if not self.connected:
            return False
        
        try:
            for i in range(max_iterations):
                current_level = self.cam.getZoomLevel()
                error = target_level - current_level
                
                if abs(error) < tolerance:
                    self.cam.requestZoomHold()
                    return True
                
                # 控制变焦
                if error > 0:
                    self.cam.requestZoomIn()
                    sleep(0.05)
                else:
                    self.cam.requestZoomOut()
                    sleep(0.05)
                
                self.cam.requestZoomHold()
                sleep(0.05)
            
            return False
            
        except Exception as e:
            print(f"控制变焦时出错: {e}")
            return False
    
    def interactive_calibration(self):
        """交互式校准：用户输入每个测试点的光学倍数"""
        print("\n--- 交互式校准 ---")
        print("请为每个测试点输入对应的光学变焦倍数")
        print("(输入 's' 跳过该点，'q' 退出校准)")
        
        if not self.test_results:
            print("没有测试数据，请先运行自动测试")
            return
        
        calibration_points = {}
        
        for i, test_point in enumerate(self.test_results):
            level = test_point['actual_level']
            
            print(f"\n测试点 {i+1}/{len(self.test_results)}")
            print(f"当前变焦等级: {level:.2f}")
            
            # 显示当前画面（用户需要观察相机画面）
            print("请观察相机画面，输入当前看到的光学倍数 (如: 1.0, 2.5, 6.0)")
            
            while True:
                user_input = input("光学倍数 (或 s/q): ").strip()
                
                if user_input.lower() == 'q':
                    print("退出校准")
                    return
                elif user_input.lower() == 's':
                    print("跳过该点")
                    break
                
                try:
                    optical = float(user_input)
                    
                    # 验证输入范围
                    if optical < 1.0 or optical > 6.0:
                        print("光学倍数应在 1.0-6.0 范围内")
                        continue
                    
                    # 保存校准点
                    calibration_points[optical] = level
                    test_point['optical_multiple'] = optical
                    print(f"✓ 已记录: {optical:.1f}倍 -> {level:.2f}等级")
                    break
                    
                except ValueError:
                    print("请输入有效的数字")
        
        # 保存校准数据
        self.calibration_data = calibration_points
        print(f"\n校准完成，共收集 {len(calibration_points)} 个校准点")
        
        return True
    
    def auto_calibrate_common_points(self):
        """自动校准常见光学倍数点"""
        print("\n--- 自动校准常见光学倍数点 ---")
        
        if not self.connected or self.min_level is None or self.max_level is None:
            print("请先连接相机并查找变焦范围")
            return False
        
        try:
            # 常见的光学倍数点
            common_points = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0]
            
            # 假设线性关系进行初步估计
            level_range = self.max_level - self.min_level
            optical_range = 5.0  # 1-6倍，共5倍范围
            
            calibration_points = {}
            
            print("正在校准常见光学倍数点...")
            
            for optical in common_points:
                # 根据线性关系估计目标等级
                optical_ratio = (optical - 1.0) / optical_range
                estimated_level = self.min_level + optical_ratio * level_range
                
                print(f"\n校准 {optical:.1f}倍:")
                print(f"  估计等级: {estimated_level:.2f}")
                
                # 变焦到估计位置
                if self.zoom_to_level(estimated_level, tolerance=0.02):
                    actual_level = self.cam.getZoomLevel()
                    calibration_points[optical] = actual_level
                    
                    print(f"  实际等级: {actual_level:.2f}")
                    
                    # 显示确认提示
                    confirm = input(f"  相机当前看起来是 {optical:.1f}倍吗？(y/n): ").lower()
                    
                    if confirm != 'y':
                        user_optical = input("  实际是多少倍？: ")
                        try:
                            user_optical = float(user_optical)
                            calibration_points[user_optical] = actual_level
                        except:
                            print("  跳过该点")
                    else:
                        calibration_points[optical] = actual_level
                else:
                    print(f"  无法变焦到指定等级")
            
            self.calibration_data = calibration_points
            print(f"\n自动校准完成，共收集 {len(calibration_points)} 个校准点")
            
            return True
            
        except Exception as e:
            print(f"自动校准时出错: {e}")
            return False
    
    def generate_mapping_function(self):
        """生成映射函数"""
        if not self.calibration_data:
            print("没有校准数据")
            return None
        
        try:
            # 提取数据点
            optical_points = sorted(self.calibration_data.keys())
            level_points = [self.calibration_data[optical] for optical in optical_points]
            
            # 使用多项式拟合 (3阶多项式)
            degree = min(3, len(optical_points) - 1)
            coeffs = np.polyfit(optical_points, level_points, degree)
            
            # 生成拟合函数
            poly_func = np.poly1d(coeffs)
            
            # 生成映射表
            mapping_table = {}
            for i in range(1, 61):  # 1.0-6.0，步长0.1
                optical = i / 10.0
                if optical <= 6.0:
                    level = poly_func(optical)
                    mapping_table[optical] = float(level)
            
            return {
                'coeffs': coeffs.tolist(),
                'poly_func_str': str(poly_func),
                'mapping_table': mapping_table,
                'raw_data': self.calibration_data
            }
            
        except Exception as e:
            print(f"生成映射函数时出错: {e}")
            return None
    
    def plot_calibration_results(self):
        """绘制校准结果图表"""
        if not self.calibration_data:
            print("没有校准数据可绘制")
            return
        
        try:
            # 提取数据
            optical_points = sorted(self.calibration_data.keys())
            level_points = [self.calibration_data[optical] for optical in optical_points]
            
            # 创建图表
            plt.figure(figsize=(12, 8))
            
            # 绘制原始数据点
            plt.subplot(2, 2, 1)
            plt.scatter(optical_points, level_points, color='blue', s=50)
            plt.xlabel('光学变焦倍数')
            plt.ylabel('相机内部等级')
            plt.title('校准数据点')
            plt.grid(True, alpha=0.3)
            
            # 绘制拟合曲线
            plt.subplot(2, 2, 2)
            
            # 生成拟合函数
            degree = min(3, len(optical_points) - 1)
            coeffs = np.polyfit(optical_points, level_points, degree)
            poly_func = np.poly1d(coeffs)
            
            # 生成曲线数据
            x_curve = np.linspace(min(optical_points), max(optical_points), 100)
            y_curve = poly_func(x_curve)
            
            plt.scatter(optical_points, level_points, color='blue', s=50, label='数据点')
            plt.plot(x_curve, y_curve, 'r-', label=f'{degree}阶拟合')
            plt.xlabel('光学变焦倍数')
            plt.ylabel('相机内部等级')
            plt.title('校准曲线拟合')
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            # 绘制误差图
            plt.subplot(2, 2, 3)
            fitted_levels = [poly_func(optical) for optical in optical_points]
            errors = [fitted - actual for fitted, actual in zip(fitted_levels, level_points)]
            
            plt.bar(range(len(errors)), errors, color='orange', alpha=0.7)
            plt.axhline(y=0, color='red', linestyle='--')
            plt.xlabel('数据点索引')
            plt.ylabel('拟合误差')
            plt.title('拟合误差分析')
            plt.grid(True, alpha=0.3)
            
            # 绘制数据统计
            plt.subplot(2, 2, 4)
            stats_text = f"数据点数: {len(optical_points)}\n"
            stats_text += f"变焦范围: {min(optical_points):.1f}-{max(optical_points):.1f}倍\n"
            stats_text += f"等级范围: {min(level_points):.2f}-{max(level_points):.2f}\n"
            stats_text += f"拟合多项式: {poly_func}\n"
            stats_text += f"最大误差: {max(abs(errors)):.4f}"
            
            plt.text(0.1, 0.5, stats_text, fontsize=10, 
                    verticalalignment='center', transform=plt.gca().transAxes)
            plt.axis('off')
            plt.title('校准统计信息')
            
            plt.tight_layout()
            plt.show()
            
            # 保存图表
            save = input("是否保存图表？(y/n): ").lower()
            if save == 'y':
                filename = input("文件名 (默认: calibration_plot.png): ") or "calibration_plot.png"
                plt.savefig(filename, dpi=300)
                print(f"图表已保存为 {filename}")
            
        except Exception as e:
            print(f"绘制图表时出错: {e}")
    
    def save_calibration_data(self, filename=None):
        """保存校准数据到文件"""
        if not self.calibration_data:
            print("没有校准数据可保存")
            return False
        
        try:
            if filename is None:
                filename = input("文件名 (默认: zoom_calibration.json): ") or "zoom_calibration.json"
            
            # 生成完整校准数据
            calibration_data = {
                'camera_ip': self.server_ip,
                'camera_port': self.port,
                'min_level': float(self.min_level) if self.min_level else None,
                'max_level': float(self.max_level) if self.max_level else None,
                'calibration_points': {str(k): float(v) for k, v in self.calibration_data.items()},
                'test_results': self.test_results,
                'mapping_function': self.generate_mapping_function()
            }
            
            # 保存到文件
            with open(filename, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            print(f"✓ 校准数据已保存到 {filename}")
            return True
            
        except Exception as e:
            print(f"保存校准数据时出错: {e}")
            return False
    
    def load_calibration_data(self, filename=None):
        """从文件加载校准数据"""
        try:
            if filename is None:
                filename = input("文件名 (默认: zoom_calibration.json): ") or "zoom_calibration.json"
            
            if not os.path.exists(filename):
                print(f"文件 {filename} 不存在")
                return False
            
            with open(filename, 'r') as f:
                data = json.load(f)
            
            # 恢复数据
            self.calibration_data = {float(k): float(v) for k, v in data['calibration_points'].items()}
            self.min_level = data['min_level']
            self.max_level = data['max_level']
            
            print(f"✓ 已从 {filename} 加载校准数据")
            print(f"  共 {len(self.calibration_data)} 个校准点")
            
            return True
            
        except Exception as e:
            print(f"加载校准数据时出错: {e}")
            return False
    
    def export_python_code(self):
        """导出Python代码"""
        if not self.calibration_data:
            print("没有校准数据")
            return
        
        mapping = self.generate_mapping_function()
        if not mapping:
            return
        
        code = '''#!/usr/bin/env python3
"""
思翼相机变焦映射代码 (自动生成)
根据校准数据生成的变焦映射函数
"""

import numpy as np

class ZoomMapper:
    """变焦映射器"""
    
    def __init__(self):
        # 校准数据
        self.calibration_data = %s
        
        # 拟合多项式系数
        self.poly_coeffs = %s
        
        # 映射表 (光学倍数 -> 相机等级)
        self.zoom_mapping = %s
        
    def optical_to_level(self, optical):
        """光学倍数转相机等级"""
        # 使用多项式拟合
        level = np.polyval(self.poly_coeffs, optical)
        return float(level)
    
    def level_to_optical(self, level):
        """相机等级转光学倍数 (反向查找)"""
        # 在映射表中查找最接近的等级
        closest_optical = min(self.zoom_mapping.keys(),
                             key=lambda x: abs(self.zoom_mapping[x] - level))
        return closest_optical

# 使用示例
if __name__ == "__main__":
    mapper = ZoomMapper()
    
    # 测试几个点
    test_points = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    for optical in test_points:
        level = mapper.optical_to_level(optical)
        print(f"{optical:.1f}倍 -> 等级 {level:.2f}")
''' % (
            str({k: v for k, v in self.calibration_data.items()}),
            str(mapping['coeffs']),
            str(mapping['mapping_table'])
        )
        
        filename = input("导出文件名 (默认: zoom_mapper.py): ") or "zoom_mapper.py"
        with open(filename, 'w') as f:
            f.write(code)
        
        print(f"✓ Python代码已导出到 {filename}")
    
    def run(self):
        """运行校准程序"""
        if not self.connect():
            return
        
        try:
            while True:
                print("\n=== 主菜单 ===")
                print("1. 查找变焦范围")
                print("2. 自动测试变焦")
                print("3. 交互式校准")
                print("4. 自动校准常见点")
                print("5. 生成映射函数")
                print("6. 绘制校准图表")
                print("7. 保存校准数据")
                print("8. 加载校准数据")
                print("9. 导出Python代码")
                print("0. 退出")
                
                choice = input("请选择操作: ").strip()
                
                if choice == '0':
                    print("退出校准程序")
                    break
                elif choice == '1':
                    self.find_min_max_levels()
                elif choice == '2':
                    points = input("测试点数 (默认: 20): ").strip()
                    points = int(points) if points.isdigit() else 20
                    self.auto_test_zoom_range(points)
                elif choice == '3':
                    self.interactive_calibration()
                elif choice == '4':
                    self.auto_calibrate_common_points()
                elif choice == '5':
                    mapping = self.generate_mapping_function()
                    if mapping:
                        print(f"拟合多项式: {mapping['poly_func_str']}")
                        print("映射表示例:")
                        for optical, level in list(mapping['mapping_table'].items())[:10]:
                            print(f"  {optical:.1f}倍 -> {level:.2f}")
                elif choice == '6':
                    self.plot_calibration_results()
                elif choice == '7':
                    self.save_calibration_data()
                elif choice == '8':
                    self.load_calibration_data()
                elif choice == '9':
                    self.export_python_code()
                else:
                    print("无效选择")
                
                input("\n按回车键继续...")
        
        finally:
            self.disconnect()


def main():
    """主函数"""
    calibrator = AutoZoomCalibrator()
    calibrator.run()


if __name__ == "__main__":
    main()
