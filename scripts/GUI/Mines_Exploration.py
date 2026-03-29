#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import os
import signal
import time

class AdvancedDroneExplorationLauncher:
    def __init__(self, root):
        self.root = root
        self.exploration_process = None
        self.test_process = None
        self.setup_ui()
    
    def setup_ui(self):
        # 设置窗口
        self.root.title("-Modeled by YIDrone-")
        self.root.geometry("700x600")
        self.root.configure(bg='#1e1e2e')  # 深色背景
        
        # 设置窗口图标
        self.set_window_icon()
        
        # 设置现代主题
        self.setup_modern_theme()
        
        # 创建主框架
        main_frame = tk.Frame(self.root, bg='#1e1e2e', padx=30, pady=30)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 标题区域
        self.create_header(main_frame)
        
        # 状态显示区域
        self.create_status_section(main_frame)
        
        # 按钮区域
        self.create_button_section(main_frame)
        
        # 日志区域
        self.create_log_section(main_frame)
        
        # 底部状态栏
        self.create_footer(main_frame)
    
    def set_window_icon(self):
        """设置窗口图标 - 修复版本"""
        try:
            # 方法1: 使用PNG图片作为图标
            icon_path = "/home/yidrone/Downloads/YIDrone.png"
            if os.path.exists(icon_path):
                # 创建PhotoImage对象
                icon = tk.PhotoImage(file=icon_path)
                # 设置窗口图标
                self.root.tk.call('wm', 'iconphoto', self.root._w, icon)
                # 保持引用防止被垃圾回收
                self.root.icon = icon
                print("✓ 窗口图标设置成功")
            else:
                # 方法2: 使用内置符号作为备用
                self.root.tk.call('wm', 'iconbitmap', self.root._w, '@' + 
                                 os.path.join(os.path.dirname(__file__), 'drone_icon.xbm'))
        except Exception as e:
            print(f"窗口图标设置失败: {e}")
            # 方法3: 使用系统默认图标
            try:
                # 尝试使用XBM格式（Linux原生支持）
                self.root.iconbitmap('@/usr/share/pixmaps/python.xbm')
            except:
                # 最终回退：不设置图标
                print("使用默认窗口图标")
    
    def setup_modern_theme(self):
        """设置现代化主题样式"""
        style = ttk.Style()
        style.theme_use('clam')  # 使用clam主题作为基础
        
        # 配置自定义样式
        style.configure('Custom.TFrame', background='#1e1e2e')
        style.configure('Title.TLabel', 
                       background='#1e1e2e', 
                       foreground='#ffffff',
                       font=('Arial', 18, 'bold'))
        
        style.configure('Status.TLabel', 
                       background='#2d2d44', 
                       foreground='#89b4fa',
                       font=('Arial', 11, 'bold'),
                       padding=(15, 10))
        
        style.configure('Main.TButton',
                       font=('Arial', 11, 'bold'),
                       padding=(10, 8),
                       relief='flat')
        
        style.configure('Secondary.TButton',
                       font=('Arial', 10),
                       padding=(8, 6),
                       relief='flat')
        
        style.configure('Small.TButton',
                       font=('Arial', 9),
                       padding=(6, 4),
                       relief='flat')
    
    def create_header(self, parent):
        """创建标题区域"""
        header_frame = tk.Frame(parent, bg='#1e1e2e')
        header_frame.pack(fill=tk.X, pady=(0, 20))
        
        # 主标题
        title = tk.Label(header_frame, 
                        text="自主无人机矿洞探索控制系统",
                        font=("Arial", 20, "bold"),
                        fg="#cdd6f4",
                        bg='#1e1e2e')
        title.pack()
        
        # 副标题
        subtitle = tk.Label(header_frame,
                           text="Autonomous Drone Mine-Exploration Control System",
                           font=("Arial", 10),
                           fg="#a6adc8",
                           bg='#1e1e2e')
        subtitle.pack(pady=(5, 0))
    
    def create_status_section(self, parent):
        """创建状态显示区域"""
        status_frame = tk.Frame(parent, bg='#2d2d44', relief=tk.RAISED, bd=2, padx=20, pady=12)
        status_frame.pack(fill=tk.X, pady=(0, 25))
        
        # 状态指示器
        status_container = tk.Frame(status_frame, bg='#2d2d44')
        status_container.pack(fill=tk.X)
        
        # 状态图标
        self.status_icon = tk.Label(status_container,
                                   text="●",
                                   font=("Arial", 16),
                                   fg="#a6e3a1",  # 绿色表示就绪
                                   bg='#2d2d44')
        self.status_icon.pack(side=tk.LEFT)
        
        # 状态文本
        self.status_var = tk.StringVar(value="系统状态: 就绪")
        status_label = tk.Label(status_container,
                               textvariable=self.status_var,
                               font=("Arial", 12, "bold"),
                               fg="#cdd6f4",
                               bg='#2d2d44')
        status_label.pack(side=tk.LEFT, padx=(10, 0))
        
        # 系统时间
        self.time_var = tk.StringVar()
        time_label = tk.Label(status_container,
                             textvariable=self.time_var,
                             font=("Arial", 10),
                             fg="#a6adc8",
                             bg='#2d2d44')
        time_label.pack(side=tk.RIGHT)
        
        # 更新时间
        self.update_time()
    
    def create_button_section(self, parent):
        """创建按钮区域"""
        # 主按钮容器
        buttons_container = tk.Frame(parent, bg='#1e1e2e')
        buttons_container.pack(expand=True, pady=10)
        
        # 第一行按钮 - 主要功能
        button_frame1 = tk.Frame(buttons_container, bg='#1e1e2e')
        button_frame1.pack(pady=15)
        
        # 启动探索按钮
        self.launch_btn = self.create_modern_button(
            button_frame1, 
            "启动探索", 
            self.launch_exploration,
            "#40a02b",  # 绿色
            "#4caf50",
            width=16,
            height=2
        )
        self.launch_btn.grid(row=0, column=0, padx=12, pady=8)
        
        # 关闭探索按钮
        self.stop_btn = self.create_modern_button(
            button_frame1, 
            "关闭探索", 
            self.stop_exploration,
            "#d20f39",  # 红色
            "#f44336",
            width=16,
            height=2
        )
        self.stop_btn.grid(row=0, column=1, padx=12, pady=8)
        
        # 第二行按钮 - 辅助功能
        button_frame2 = tk.Frame(buttons_container, bg='#1e1e2e')
        button_frame2.pack(pady=15)
        
        # 显示地图按钮
        self.map_btn = self.create_modern_button(
            button_frame2, 
            "显示地图", 
            self.show_map,
            "#1e66f5",  # 蓝色
            "#2196f3",
            width=14,
            height=2
        )
        self.map_btn.grid(row=0, column=0, padx=10, pady=6)
        
        # 参数设置按钮
        self.config_btn = self.create_modern_button(
            button_frame2, 
            "参数设置", 
            self.open_config,
            "#fe640b",  # 橙色
            "#ff9800",
            width=14,
            height=2
        )
        self.config_btn.grid(row=0, column=1, padx=10, pady=6)
        
        # 附加功能容器
        extra_frame = tk.Frame(button_frame2, bg='#1e1e2e')
        extra_frame.grid(row=0, column=2, padx=10, pady=6)
        
        # 定位测试按钮
        self.test_btn = self.create_modern_button(
            extra_frame, 
            "定位测试", 
            self.start_test,
            "#8839ef",  # 紫色
            "#9c27b0",
            width=10,
            height=1,
            small=True
        )
        self.test_btn.pack(pady=(0, 3))
        
        # 结束测试按钮
        self.stop_test_btn = self.create_modern_button(
            extra_frame, 
            "结束测试", 
            self.stop_test,
            "#7c7c7c",  # 灰色
            "#795548",
            width=10,
            height=1,
            small=True
        )
        self.stop_test_btn.pack(pady=(3, 0))
    
    def create_modern_button(self, parent, text, command, color, hover_color, width=12, height=2, small=False):
        """创建现代化按钮"""
        font_size = 9 if small else (10 if width <= 12 else 11)
        font_weight = "normal" if small else "bold"
        
        btn = tk.Button(
            parent,
            text=text,
            command=command,
            bg=color,
            fg="white",
            font=("Arial", font_size, font_weight),
            width=width,
            height=height,
            relief="flat",
            bd=0,
            cursor="hand2",
            activebackground=hover_color,
            activeforeground="white"
        )
        
        # 添加悬停效果
        def on_enter(e):
            btn['background'] = hover_color
            
        def on_leave(e):
            btn['background'] = color
            
        btn.bind("<Enter>", on_enter)
        btn.bind("<Leave>", on_leave)
        
        return btn
    
    def create_log_section(self, parent):
        """创建日志区域"""
        log_container = tk.Frame(parent, bg='#1e1e2e')
        log_container.pack(fill=tk.BOTH, expand=True, pady=(15, 0))
        
        # 日志标题栏
        log_header = tk.Frame(log_container, bg='#2d2d44', padx=15, pady=8)
        log_header.pack(fill=tk.X)
        
        log_title = tk.Label(log_header,
                            text="系统日志",
                            font=("Arial", 11, "bold"),
                            fg="#cdd6f4",
                            bg='#2d2d44')
        log_title.pack(side=tk.LEFT)
        
        # 日志控制按钮
        control_frame = tk.Frame(log_header, bg='#2d2d44')
        control_frame.pack(side=tk.RIGHT)
        
        # 清空日志按钮
        clear_btn = tk.Button(control_frame,
                             text="清空日志",
                             command=self.clear_log,
                             bg='#45475a',
                             fg="#cdd6f4",
                             font=("Arial", 8),
                             width=8,
                             height=1,
                             relief="flat")
        clear_btn.pack(side=tk.LEFT, padx=(5, 0))
        
        # 日志文本区域
        log_frame = tk.Frame(log_container, bg='#11111b', relief=tk.SUNKEN, bd=1)
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = tk.Text(
            log_frame,
            height=12,
            wrap=tk.WORD,
            font=("Consolas", 9),
            bg="#11111b",
            fg="#cdd6f4",
            insertbackground="#cdd6f4",
            relief="flat",
            padx=12,
            pady=12,
            selectbackground="#45475a"
        )
        
        # 滚动条
        scrollbar = tk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 初始日志信息
        self.log_message("=== 自主无人机矿洞探索控制系统 ===")
        self.log_message("系统初始化完成")
        self.log_message("等待用户指令...")
    
    def create_footer(self, parent):
        """创建底部状态栏"""
        footer_frame = tk.Frame(parent, bg='#2d2d44', padx=15, pady=8)
        footer_frame.pack(fill=tk.X, pady=(20, 0))
        
        self.footer_var = tk.StringVar(value="就绪 | 系统正常运行")
        footer_label = tk.Label(footer_frame,
                               textvariable=self.footer_var,
                               font=("Arial", 9),
                               fg="#a6adc8",
                               bg='#2d2d44')
        footer_label.pack(side=tk.LEFT)
        
        # 版本信息
        version_label = tk.Label(footer_frame,
                                text="v2.0",
                                font=("Arial", 9),
                                fg="#6c7086",
                                bg='#2d2d44')
        version_label.pack(side=tk.RIGHT)
    
    def update_time(self):
        """更新时间显示"""
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.time_var.set(current_time)
        self.root.after(1000, self.update_time)
    
    def log_message(self, message):
        """添加消息到日志"""
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
        self.log_text.update()
    
    def clear_log(self):
        """清空日志"""
        self.log_text.delete(1.0, tk.END)
        self.log_message("日志已清空")
    
    def launch_exploration(self):
        """在新终端中启动探索脚本"""
        script_path = "/home/yidrone/AstraDrone/scripts/run_sh/demonstration.sh"
        
        # 检查脚本是否存在
        if not os.path.isfile(script_path):
            error_msg = f"错误：未找到探索脚本 {script_path}"
            self.log_message(error_msg)
            messagebox.showerror("错误", error_msg)
            return
        
        try:
            self.log_message(">>> 启动无人机探索任务...")
            self.status_var.set("系统状态: 探索中")
            self.status_icon.config(fg="#f9e2af")  # 黄色表示运行中
            self.footer_var.set("探索任务执行中...")
            
            # 在新终端中启动探索脚本
            subprocess.Popen(['terminator', '-e', f'bash -c "bash {script_path}; bash"'])
            
            self.log_message("✓ 探索脚本已在新终端中启动")
            self.log_message("无人机开始矿洞探索...")
            
        except Exception as e:
            error_msg = f"启动探索失败: {str(e)}"
            self.log_message(error_msg)
            messagebox.showerror("错误", error_msg)
    
    def show_map(self):
        """在新终端中显示地图"""
        try:
            self.log_message(">>> 启动地图显示...")
            
            # 在新终端中依次运行命令
            commands = [
                "cd ~/AstraDrone/AstraDrone_ros1_ws/pcd/",
                "ls",
                r"echo -e \"\033[32m请查看现有pcd文件，输入 'pcl_viewer [文件名]' 来查看地图。\033[0m\""
            ]
            
            # 将命令组合成一个bash命令
            full_command = " && ".join(commands)
            
            # 在terminator中执行
            subprocess.Popen(['terminator', '-e', f'bash -c "{full_command}; bash"'])
            
            self.log_message("✓ 地图显示已启动")
            self.log_message("PCL查看器正在新终端中运行...")
            
        except Exception as e:
            error_msg = f"启动地图显示失败: {str(e)}"
            self.log_message(error_msg)
            messagebox.showerror("错误", error_msg)
    
    def stop_exploration(self):
        """使用tmux命令停止探索"""
        try:
            self.log_message(">>> 停止探索任务...")
            
            # 使用tmux命令停止demonstration会话
            subprocess.Popen(['tmux', 'kill-session', '-t', 'demonstration'])
            
            self.log_message("✓ 探索任务已停止")
            self.status_var.set("系统状态: 已停止")
            self.status_icon.config(fg="#f38ba8")  # 红色表示停止
            self.footer_var.set("探索任务已停止")
            
        except Exception as e:
            error_msg = f"停止探索任务时出错: {str(e)}"
            self.log_message(error_msg)
            messagebox.showerror("错误", error_msg)
    
    def open_config(self):
        """在当前窗口打开参数设置"""
        try:
            self.log_message(">>> 打开参数设置...")
            
            # 在当前窗口依次运行命令
            commands = [
                "cd ~/AstraDrone/AstraDrone_ros1_ws/src/Exploration/",
                "code ."
            ]
            
            # 将命令组合成一个bash命令
            full_command = " && ".join(commands)
            
            # 在当前窗口执行
            subprocess.Popen(['bash', '-c', full_command])
            
            self.log_message("✓ 参数设置已打开")
            self.log_message("VSCode正在运行...")
            
        except Exception as e:
            error_msg = f"打开参数设置失败: {str(e)}"
            self.log_message(error_msg)
            messagebox.showerror("错误", error_msg)
    
    def start_test(self):
        """在新终端中启动定位测试"""
        script_path = "/home/yidrone/AstraDrone/scripts/run_sh/test.sh"
        
        # 检查脚本是否存在
        if not os.path.isfile(script_path):
            error_msg = f"错误：未找到测试脚本 {script_path}"
            self.log_message(error_msg)
            messagebox.showerror("错误", error_msg)
            return
        
        try:
            self.log_message(">>> 启动定位测试...")
            
            # 在新终端中启动测试脚本
            subprocess.Popen(['terminator', '-e', f'bash -c "bash {script_path}; bash"'])
            
            self.log_message("✓ 定位测试已在新终端中启动")
            self.log_message("正在进行定位测试...")
            
        except Exception as e:
            error_msg = f"启动定位测试失败: {str(e)}"
            self.log_message(error_msg)
            messagebox.showerror("错误", error_msg)
    
    def stop_test(self):
        """使用tmux命令停止测试"""
        try:
            self.log_message(">>> 停止定位测试...")
            
            # 使用tmux命令停止test会话
            subprocess.Popen(['tmux', 'kill-session', '-t', 'test'])
            
            self.log_message("✓ 定位测试已停止")
            
        except Exception as e:
            error_msg = f"停止定位测试时出错: {str(e)}"
            self.log_message(error_msg)
            messagebox.showerror("错误", error_msg)

def main():
    # 创建主窗口
    root = tk.Tk()
    
    # 设置窗口居中
    window_width = 700
    window_height = 600
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    
    # 创建应用
    app = AdvancedDroneExplorationLauncher(root)
    
    
    # 启动主循环
    root.mainloop()

if __name__ == "__main__":
    main()
