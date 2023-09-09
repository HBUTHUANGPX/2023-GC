import sys
sys.path.append("/home/lc/2023GC/gc_ws/src/serial_pack/scripts/ultralytics/")
from ultralytics import YOLO

# Load a model
# model = YOLO('yolov8n.yaml')  # build a new model from YAML
model = YOLO('yolov8n.pt')  # load a pretrained model (recommended for training)
# model = YOLO('yolov8n.yaml').load('yolov8n.pt')  # build from YAML and transfer weights

# Train the model
results = model.train(data='/home/lc/2023GC/gc_ws/datasets/Rmx/rmx.yaml', epochs=300, imgsz=640,batch=4,conf=0.4, iou=0.25)