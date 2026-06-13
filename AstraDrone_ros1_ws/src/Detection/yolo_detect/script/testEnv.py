import os
import cv2
import time
from ultralytics import YOLO

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 加载模型
model = YOLO(os.path.join(_SCRIPT_DIR, "..", "pt", "yolov8n.pt"))

# 读取图片
image_path = os.path.join(_SCRIPT_DIR, "bus.jpg")  # 替换为你的图片路径
frame = cv2.imread(image_path)
if frame is None:
    raise FileNotFoundError(f"图片 {image_path} 不存在")

# 持续检测循环
frame_count = 0
start_time = time.time()

while True:
    # 执行检测
    results = model(frame, verbose=True)  # 禁用控制台输出
    
    # 绘制检测结果
    annotated_frame = results[0].plot()
    
    # 计算FPS
    frame_count += 1
    fps = frame_count / (time.time() - start_time)
    cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # 显示结果
    cv2.imshow("YOLOv8 持续检测", annotated_frame)
    
    # 按ESC或Q退出
    key = cv2.waitKey(1)
    if key == 27 or key == ord('q'):
        break

cv2.destroyAllWindows()
