# 提出请求
import rospy
import cv2
from play_video.srv import Play,PlayRequest,PlayResponse


class play_video():
    def __init__(self,node_name):
        rospy.init_node(node_name)
        self.client = rospy.ServiceProxy("Play",Play)
    def make_request(self,ser_num,type,number):
        req=PlayRequest()
        req.ser_num=ser_num
        req.type=type
        req.number=number
        resp=self.client.call(req)
        rospy.loginfo("response:%d",resp.flag)
        pass
if __name__=="__main__":
    print("ok")
    a=play_video("fuck_you_client")
    rate=rospy.Rate(1)
    while True:
        for i in range(4):    
            a.make_request(i,i,i)
            rate.sleep()
    rospy.spin()