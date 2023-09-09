import os
import sys
# path = os.path.abspath(".")
# sys.path.insert(0,path + "/src/cv_control/scripts")
import rospy

import onnx
import sys
import os
import numpy as np
from pathlib import Path
from typing import Union
import cv2
from ultralytics import YOLO


def train():
    # 加载模型配置文件，这里使用v8的m模型结构
    model = YOLO('yolov8m.yaml')

    # 做预训练
    # model = YOLO('yolov8x.pt')
    # model = YOLO('yolov8n.yaml').load('yolov8n.pt')

    # 训练模型
    model.train(data="coco.yaml", epochs=100, imgsz=640)

def onnx():
    # 使用onnx导出文件
    # model = YOLO('yolov8n.pt')  # load an official model
    model = YOLO('YOLOv8/runs/detect/train1/weights/best.pt')  # load a custom trained
    # Export the model
    model.export(format='onnx')

def test_video():
    
    model = YOLO("/home/lc/2023GC/gc_ws/src/cv_control/scripts/rubbish1.pt")
                 #yolov8x.pt ")  
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        # cut_frame=frame[30:300,30:300]
        if ret:
            results = model(frame)
            print("===============//2/======================")
            for result in results:
                boxes_cls = result.boxes.cls.clone().detach().cpu().numpy()#.data  # Boxes object for bbox outputs
                boxes_xywh =result.boxes.xywh.clone().detach().cpu().numpy()
                print(boxes_cls)
                print(boxes_xywh)
            print("===============////======================")# x:150 y:53->x:526 y:422
            # print(results[0])
            ann = results[0].plot()
            cv2.imshow("yolo", ann)
            # out.write(ann)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    cv2.destroyAllWindows()
    cap.release()
 
# train()
# test_video()
# test_img()
# predict()
# tracker()
# onnx()

# 下面是使用netron导出模型结构
# netron.start("YOLOv8/runs/detect/train1/weights/best.onnx")


# class test():
#     def __init__(self,node_name):
#         rospy.init_node(node_name)
#         pub = rospy.Publisher("chatter",String,queue_size=10)
#     def pub_test(self):
        
#         pass
    
if __name__=="__main__":
    # test("test")
    test_video()
    pass
