# 
import rospy
import cv2
from play_video.srv import Play,PlayRequest,PlayResponse
import threading
import numpy as np
class play_video():
    def __init__(self,node_name):
        rospy.init_node(node_name)
        self.server = rospy.Service("Play",Play,self.doreq)
        self.cap = cv2.VideoCapture('/home/lc/2023GC/gc_ws/src/play_video/scripts/ljfl.mp4')
        self.ap_image=np.ones((100, 640,3), dtype=np.uint8)
        self.type_name=['RECOVERABLE GARBAGE',
                        'HARMFUL     GARBAGE',
                        'KITCHEN     GARBAGE',
                        'OTHER       GARBAGE']
        self.conter_num=int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.play_flag=False
    def doreq(self,req=Play()): # 回调用来控制视频播放的改变
        # rospy.loginfo("ser_num=%d",req.ser_num)
        # rospy.loginfo("type=%d",req.type)
        # rospy.loginfo("number=%d",req.number)
        resp=PlayResponse(req.ser_num+req.type+req.number)
        self.ap_image=np.zeros((100, 640,3), dtype=np.uint8)
        self.draw_a('serial',(10,25))
        self.draw_a('type',(250,25))
        self.draw_a('number',(500,25))
        self.draw_a(str(req.ser_num),(10,75))
        self.draw_a(self.type_name[req.type],(100,75))
        self.draw_a(str(req.number),(550,75))
        return resp
    def play_video_th(self):
        self.frame_counter=0
        rate=rospy.Rate(30)
        while(self.cap.isOpened()):
            ret, self.frame = self.cap.read()
            self.play_flag=True
            self.frame_counter += 1
            if self.frame_counter == self.conter_num-1:
                self.frame_counter = 0
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            rate.sleep()
    def draw_a(self,str_context,start_point):
        # str_context = ""
        font = cv2.FONT_HERSHEY_COMPLEX # 字体类型
        color = (255,255,255) # 颜色选择，单通道只有黑白
        # start_point = (50,50) # 文字起始点
        print_size = 1 # 字体大小
        thickness = 1 # 文字粗细
        cv2.putText(self.ap_image, str_context, start_point,font,print_size,color,thickness)
        pass
    def my_show(self):
        while self.play_flag==False:
            pass
        
        while self.play_flag:
            frame=cv2.resize(self.frame, ([640,400]), interpolation = cv2.INTER_AREA)
            frame=np.vstack((frame,self.ap_image))
            frame=cv2.resize(frame, ([768,600]), interpolation = cv2.INTER_AREA)
            
            cv2.imshow('image', frame)
            cv2.waitKey(1)
            pass
            
if __name__=="__main__":
    print("ok")
    a=play_video("fuck_you_server")
    
    
    thread_1 = threading.Thread(target=a.play_video_th)
    thread_2 = threading.Thread(target=a.my_show)
    thread_1.start()
    thread_2.start()
    rospy.spin()
    a.play_flag=False
    a.cap.release()
    cv2.destroyAllWindows()