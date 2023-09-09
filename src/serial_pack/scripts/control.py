from serial_pack.msg import serial_low, serial_up
from serial_pack.msg import Detect
import time
import numpy as np
import threading
import warnings
import cv2
from std_msgs.msg import Time
import rospy
import os
import sys
sys.path.append("/home/lc/2023GC/gc_ws/src/serial_pack/scripts/ultralytics/")
warnings.filterwarnings('ignore')
# uint16 x_move # 1=0.0381mm
# uint16 y_move # 1=0.0381mm
# uint16 yaw    # 360度/65535*yaw
# uint8 grasp   # 90度 /255  *grasp
# uint8 lift    # 0/1


class test():
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.lats_time = rospy.Time.now()
        self.pub_low = rospy.Publisher("low", serial_low, queue_size=10)
        self.pub_up = rospy.Publisher("up", serial_up, queue_size=10)
        self.Up = serial_up()
        self.Low = serial_low()
        self.Up.x_move = 0  # 0-300
        self.Up.y_move = 0
        self.Up.yaw = 0
        self.Up.grasp = 0
        self.Up.lift = 1
        self.isDetect = False
        # self.machine_init()
        self.sub_detect = rospy.Subscriber(
            "/detect", Detect, self.Detect_callback, queue_size=1)
        self.isDetect = False
        self.ys_start_time=rospy.Time.now()
        self.out_flag_arr = None
        self.tg_thread=threading.Thread(target=self.wait_tg_goback)
        self.ys_isDo_flag=False
    def Detect_callback(self, msg=Detect()):
        self.lats_time = msg.t
        # print(self.lats_time)
        self.boxes_cls = np.asarray(msg.cls)
        len_cls = len(self.boxes_cls)
        self.boxes_xyxy = np.asarray(list(msg.xyxy)).reshape((len_cls, 4))
        self.out_flag_arr = np.ones(len_cls, dtype=np.uint8)
        for i in range(len_cls):
            h_x = self.boxes_xyxy[i][0]
            h_y = self.boxes_xyxy[i][1]
            l_x = self.boxes_xyxy[i][2]
            l_y = self.boxes_xyxy[i][3]
            # x:130 y:48->x:526 y:426
            if h_x < 130 or h_x > 526 or l_x < 130 or l_x > 526 or h_y < 48 or h_y > 426 or l_y < 53 or l_y > 426:
                self.out_flag_arr[i] = 0

    def my_pub_low(self, db_flag=[0, 0, 0, 0], tg_flag=0, tp_flag=0):
        '''
            db_flag 挡板 0抬起 1放下
            tg_flag 推杆 0后退 1前进
            tp_flag 托盘 0/1/2/3/4
        '''
        self.Low.db_1_flag = db_flag[0]
        self.Low.db_2_flag = db_flag[1]
        self.Low.db_3_flag = db_flag[2]
        self.Low.db_4_flag = db_flag[3]
        self.Low.tg_flag = tg_flag
        self.Low.tp_flag = tp_flag
        for i in range(3):
            self.pub_low.publish(self.Low)
            time.sleep(0.05)

    def my_pub_up(self,x_move=0,y_move=0,grasp=0,yaw=0,lift=0):
        self.Up.x_move=x_move
        self.Up.y_move=y_move
        self.Up.grasp=grasp
        self.Up.yaw=yaw
        self.Up.lift=lift
        for i in range(3):
            self.pub_up.publish(self.Up)
            time.sleep(0.05)

    def machine_init(self):
        self.my_pub_low([0, 0, 0, 0], 0, 0)
        self.my_pub_up(0,0,0,0,0)
        print("init_ok")
        self.my_pub_up(0,0,0,0,0)#1->0逆时针旋转
        

    def db_ctr(self, flag=0):  # 0抬起 1放下
        """
        0抬起 1放下
        """
        self.Low.db_1_flag = flag
        self.Low.db_2_flag = flag
        self.Low.db_3_flag = flag
        self.Low.db_4_flag = flag

    def tp_do(self,rubbish_class=2):
        print("===============//2/======================")
        print("ok")
        self.my_pub_up(0,0,1,0,0)#1->0逆时针旋转
        time.sleep(1)
        self.my_pub_up(0,0,0,0,0)#1->0逆时针旋转
        self.my_pub_low([1,1,1,1],0,0)
        time.sleep(1)
        print("放下挡板")
        self.my_pub_low([1,1,1,1],0,rubbish_class)
        time.sleep(2)
        print("托盤向可回收筒倾倒")
        self.Low.tp_flag = 0
        self.my_pub_low([1,1,1,1],0,0)
        time.sleep(0.5)
        print("托盘回来")
        self.my_pub_low([0,0,0,0],0,0)
        print("抬起挡板")
        
        
        print("===============////======================")
    def wait_tg_goback(self):
        print("wait.......")
        time.sleep(9.0)
        print("time is ok .....")
        self.my_pub_low([0,0,0,0],0,0)
        print("推杆回来")
    def last_rubbish(self):
        print("start_last_rubbish")
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
        while not rospy.is_shutdown():
            # self.pub_low.publish(self.Low)
            # print(self.isDetect,self.out_flag_arr.sum())
            time_distance = (rospy.Time.now()-self.lats_time).to_sec()
            if time_distance < 1.0:
                self.isDetect = True
            else:
                self.isDetect = False
            # print(time_distance,self.isDetect)
            # if 0:
            if (self.isDetect):
                out_flag_arr = self.out_flag_arr
                boxes_cls = self.boxes_cls
                # print("DETECT")
                if (out_flag_arr.sum() == 1):
                    np.where(out_flag_arr == 1)
                    print("SUM = 1 is ", np.where(out_flag_arr == 1)[0][0])
                    isZero = np.where(out_flag_arr == 1)[0][0]
                    print(boxes_cls, out_flag_arr)
                    if (boxes_cls[isZero] == 0):  # 回收可回收垃圾
                        # print("")
                        if out_flag_arr[0]:
                            self.tp_do()
                            break
        pass
    def ys_do_once(self):
        if not self.ys_isDo_flag:
            self.ys_isDo_flag=True
            print("压缩")
            self.my_pub_low([0,0,0,0],1,0)
            self.ys_start_time=rospy.Time.now()
            self.tg_thread.start()
        else:
            ...        
    def mode_single(self):
        """
        进行单分类,默认先丢可回收垃圾，然后再丢其他
        """
        while not rospy.is_shutdown():
            time_distance = (rospy.Time.now()-self.lats_time).to_sec()
            if time_distance < 1.0:
                self.isDetect = True
            else:
                self.isDetect = False
            if (self.isDetect):
                out_flag_arr = self.out_flag_arr
                boxes_cls = self.boxes_cls
                if (out_flag_arr.sum() == 1):
                    np.where(out_flag_arr == 1)
                    print("SUM = 1 is ", np.where(out_flag_arr == 1)[0][0])
                    isZero = np.where(out_flag_arr == 1)[0][0]
                    print(boxes_cls, out_flag_arr)
                    if (boxes_cls[isZero] == 0):  # 可回收垃圾
                        if out_flag_arr[0]:
                            self.tp_do(2)
                    else:
                        if out_flag_arr[0]:
                            if boxes_cls[isZero]==1: #电池一类不可回收垃圾
                                self.tp_do(3)
                                self.ys_do_once()
if __name__ == "__main__":
    a = test("test")
    time.sleep(1.5)
    # a.my_pub_low([1,0,1,0],1,0)
    # time.sleep(1.5)
    # a.my_pub_low([0,0,0,0],0,0)
    a.machine_init()
    thread_2 = threading.Thread(target=a.mode_single)
    thread_2.start()
    rospy.spin()
