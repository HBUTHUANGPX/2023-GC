import rospy
import sys
from std_msgs.msg import Int32MultiArray
sys.path.append("/home/lc/2023GC/gc_ws/src/serial_pack/scripts/ultralytics/")
import cv2
import warnings
warnings.filterwarnings('ignore')
from ultralytics import YOLO
import threading
import numpy as np
from serial_pack.msg import Detect
# uint16 x_move # 1=0.0381mm
# uint16 y_move # 1=0.0381mm
# uint16 yaw    # 360度/65535*yaw
# uint8 grasp   # 90度 /255  *grasp
# uint8 lift    # 0/1
class yolo():
    def __init__(self,node_name):
        rospy.init_node(node_name)
        self.model = YOLO("/home/lc/2023GC/gc_ws/runs/detect/train12/weights/best.pt")
        self.cap=cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FPS,30)
        self.detect_pub=rospy.Publisher("detect",Detect,queue_size=1)
        self.detect_msg=Detect()
        self.last=0
    def test_video(self):
        import time
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            # frame=frame[53:422,150:526]
            if ret:
                start=rospy.get_time()
                results = self.model(frame)
                for result in results:                    
                    self.boxes_cls = result.boxes.cls.clone().detach().cpu().numpy()#.data  # Boxes object for bbox outputs
                    self.boxes_xyxy =result.boxes.xyxy.clone().detach().cpu().numpy()
                    self.boxes_conf =result.boxes.conf.clone().detach().cpu().numpy()

                conf_=np.where(self.boxes_conf<0.8)[0]
                self.boxes_cls=np.delete(self.boxes_cls,conf_,0)
                self.boxes_xyxy=np.delete(self.boxes_xyxy,conf_,0)
                self.detect_msg.cls=list(map(int, self.boxes_cls.astype(np.int32)))
                self.detect_msg.xyxy = list(map(float, self.boxes_xyxy.flatten()))
                self.detect_msg.t=rospy.Time.now()
                self.detect_pub.publish(self.detect_msg)
                ann = results[0].plot()
                cv2.imshow("yolo", ann)
                # len_cls=len(self.boxes_cls)
                # self.out_flag_arr=np.ones(len_cls,dtype=np.uint8)
                
                # if len_cls>0:
                #     self.isDetect=True
                # else:
                #     self.isDetect=False
                    
                # for i in range(len_cls): # 濾除
                #     h_x=self.boxes_xyxy[i][0]
                #     h_y=self.boxes_xyxy[i][1]
                #     l_x=self.boxes_xyxy[i][2]
                #     l_y=self.boxes_xyxy[i][3]
                #     if h_x<150 or h_x>526 or l_x<150 or l_x>526 or h_y<53 or h_y>422 or l_y<53 or l_y>422:
                #         self.out_flag_arr[i]=0
                now=rospy.get_time()
                print("latence：%7.3f ms, FPS：%7.3f" %((now-start)*1000,1/(now-self.last))) 
                self.last=now     
                 
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        cv2.destroyAllWindows()
        self.cap.release()

if __name__ == "__main__":
    a=yolo("yolov8_node")
    thread_1=threading.Thread(target=a.test_video)
    thread_1.start()
    rospy.spin()
    a.cap.release()
    cv2.destroyAllWindows()