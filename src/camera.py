#! /usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from projeto_11.srv import Camservice, CamserviceResponse
import numpy as np

class myCamera():

    def __init__(self):
        
        print('Camera ativa')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_color",Image,self.imageCallBack)
        self.service = rospy.Service('detect', Camservice, self.callback_ServiceCamera)
        self.ok = False
        
    def callback_ServiceCamera(self, request):
        self.res = CamserviceResponse()
        self.res = self.ok
        return self.res
    
    def imageCallBack(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        red_mask = self.cv_image[:,:,2]<20
        green_mask= self.cv_image[:,:,1]>60
        blue_mask = self.cv_image[:,:,0]<20
        
        color_mask = np.logical_and.reduce((red_mask,green_mask,blue_mask))
        self.ok = np.any(color_mask)

if __name__ == '__main__':

    rospy.init_node('camera_11')
    tiago_camera = myCamera()
    rospy.spin()