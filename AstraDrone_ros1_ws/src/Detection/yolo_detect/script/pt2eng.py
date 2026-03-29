from ultralytics import YOLO

# Load a model
model = YOLO('yolov8n.pt')  # load a custom trained
# Export the model
model.export(format='engine',half=True,simplify=True)

#出现requirements报错，直接ctrl+c跳过

