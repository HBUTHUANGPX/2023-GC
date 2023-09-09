import rospy
import os
import sys
sys.path.append("/home/lc/2023GC/gc_ws/src/serial_pack/scripts/ultralytics/")
import std_msgs
from serial_pack.msg import serial_low,serial_up
import cv2
import warnings
warnings.filterwarnings('ignore')
from ultralytics import YOLO
import threading
import numpy as np
import time
# uint16 x_move # 1=0.0381mm
# uint16 y_move # 1=0.0381mm
# uint16 yaw    # 360度/65535*yaw
# uint8 grasp   # 90度 /255  *grasp
# uint8 lift    # 0/1
class test():
    def __init__(self,node_name):
        rospy.init_node(node_name)
        self.pub_low = rospy.Publisher("low",serial_low,queue_size=10)
        self.pub_up = rospy.Publisher("up",serial_up,queue_size=10)
        self.model = YOLO("/home/lc/2023GC/gc_ws/runs/detect/train11/weights/best.pt")#("/home/lc/2023GC/gc_ws/src/serial_pack/scripts/rubbish1.pt")
        self.Up=serial_up()
        self.Low=serial_low()
        self.Up.x_move=0 #0-300
        self.Up.y_move=0
        self.Up.yaw=0
        self.Up.grasp=0
        self.Up.lift=1
        self.isDetect=False
        self.machine_init()
        time.sleep(3)
        self.cap=cv2.VideoCapture(0)
        
    def machine_init(self):
        self.Low.tp_flag=0
        self.Low.tg_flag=0
        self.Low.db_1_flag=1
        self.Low.db_2_flag=1
        self.Low.db_3_flag=1
        self.Low.db_4_flag=1
        self.pub_low.publish(self.Low)
        pass
    
    def low_pub(self):
        self.pub_low.publish(self.Low)
    def tp_do(self):
        print("===============//2/======================")     
        print("ok")
        self.Low.db_1_flag=0
        self.Low.db_2_flag=0
        self.Low.db_3_flag=0
        self.Low.db_4_flag=0
        self.low_pub()
        time.sleep(2)
        print("放下挡板")
        self.Low.tp_flag=2
        self.low_pub()
        time.sleep(2)
        print("tuo向可回收筒倾倒")
        self.Low.tp_flag=0
        self.low_pub()
        time.sleep(2)
        print("向可回收筒倾倒")
        self.Low.db_1_flag=1
        self.Low.db_2_flag=1
        self.Low.db_3_flag=1
        self.Low.db_4_flag=1
        self.low_pub()
        time.sleep(2)
        print("放下挡板")
        print("===============////======================")# x:150 y:53->x:526 y:422
    def last_rubbish(self):
        # while 1:
        #     self.Low.db_1_flag=1
        #     self.Low.db_2_flag=1
        #     self.Low.db_3_flag=1
        #     self.Low.db_4_flag=1
        #     self.low_pub()
        #     time.sleep(3)
        #     # self.tp_do()
        #     self.Low.db_1_flag=0
        #     self.Low.db_2_flag=0
        #     self.Low.db_3_flag=0
        #     self.Low.db_4_flag=0
        #     self.low_pub()
        #     time.sleep(3)
        while  self.cap.isOpened():
            # print(self.isDetect,self.out_flag_arr.sum())
            if(self.isDetect):
                out_flag_arr=self.out_flag_arr
                boxes_cls=self.boxes_cls
                # print("DETECT")
                if(out_flag_arr.sum()==1):
                    np.where(out_flag_arr==1)
                    # print("SUM = 1 is ",np.where(out_flag_arr==1)[0][0])
                    isZero=np.where(out_flag_arr==1)[0][0]
                    # print(boxes_cls,out_flag_arr)
                    if(boxes_cls[isZero]==0):#回收可回收垃圾
                        # print("")
                        if out_flag_arr[0]:
                            self.tp_do()
        pass
    def test_video(self):
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            # frame=frame[53:422,150:526]
            if ret:
                results = self.model(frame)
                for result in results:                    
                    self.boxes_cls = result.boxes.cls.clone().detach().cpu().numpy()#.data  # Boxes object for bbox outputs
                    self.boxes_xyxy =result.boxes.xyxy.clone().detach().cpu().numpy()
                    self.boxes_conf =result.boxes.conf.clone().detach().cpu().numpy()
                print(self.boxes_conf)
                ann = results[0].plot()
                cv2.imshow("yolo", ann)
                len_cls=len(self.boxes_cls)
                self.out_flag_arr=np.ones(len_cls,dtype=np.uint8)
                
                if len_cls>0:
                    self.isDetect=True
                else:
                    self.isDetect=False
                    
                for i in range(len_cls):
                    h_x=self.boxes_xyxy[i][0]
                    h_y=self.boxes_xyxy[i][1]
                    l_x=self.boxes_xyxy[i][2]
                    l_y=self.boxes_xyxy[i][3]
                    if h_x<150 or h_x>526 or l_x<150 or l_x>526 or h_y<53 or h_y>422 or l_y<53 or l_y>422:
                        self.out_flag_arr[i]=0
                       
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        cv2.destroyAllWindows()
        self.cap.release()

if __name__ == "__main__":
    a=test("test")
    thread_1=threading.Thread(target=a.test_video)
    thread_2=threading.Thread(target=a.last_rubbish)
    thread_1.start()
    thread_2.start()
    rospy.spin()
    a.cap.release()
    cv2.destroyAllWindows()
# if __name__ == "__main__":
#     #1.初始化 ROS 节点
#     rospy.init_node("test")
#     #2.创建发布者对象
#     pub_low = rospy.Publisher("low",serial_low,queue_size=10)
#     pub_up = rospy.Publisher("up",serial_up,queue_size=10)
#     #3.组织消息
#     Up=serial_up()
#     Low=serial_low
#     Up.x_move=0 #0-300
#     Up.y_move=0
#     Up.yaw=0
#     Up.grasp=0
#     Up.lift=1
#     rate = rospy.Rate(0.2)
#     index=0
#     xy=[[0,0,300,300],[0,300,300,0]]
#     while not rospy.is_shutdown():
#         Up.y_move=xy[1][index]
#         Up.x_move=xy[0][index]
#         pub_up.publish(Up)
#         print("x:",Up.x_move," y:",Up.y_move)
#         index+=1
#         if index==4:index=0
#         rate.sleep()
#         # rospy.loginfo("姓名:%s, 年龄:%d, 身高:%.2f",p.name, p.age, p.height)