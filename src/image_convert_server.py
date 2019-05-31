#! /usr/bin/env python
#coding:utf-8
import rospy
import numpy as np
import cv2
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ricoh_theta_ros.srv import DualFisheye2Equirectangular, DualFisheye2EquirectangularResponse

class ImageConvertServer():
    def __init__(self):
        self.service = rospy.Service("/ricoh_theta_ros/dual_fisheye2equirectangular", DualFisheye2Equirectangular, self.server)

    def image_converter(self, cv_img):
        # size : 1280x720
        vertex = 640
        src_cx = 319
        src_cy = 319
        src_r = 283
        src_cx2 = 1280 - src_cx

        map_x = np.zeros((vertex,vertex*2))
        map_y = np.zeros((vertex,vertex*2))
        for y in range(vertex):
            for x in range(vertex*2):
                phi1 = math.pi * x / vertex
                theta1 = math.pi * y / vertex

                X = math.sin(theta1) * math.cos(phi1)
                Y = math.sin(theta1) * math.sin(phi1)
                Z = math.cos(theta1)

                phi2 = math.acos(-X)
                theta2 = np.sign(Y)*math.acos(-Z/math.sqrt(Y*Y + Z*Z))

                #0 等距離射影
                #1 立体射影
                #2 立体射影逆変換
                #3 正射影
                #4 正射影逆変換
                method = 3

                if(phi2 < math.pi / 2):
                    #等距離射影
                    if method == 0:
                        r_ = phi2 / math.pi * 2
                    #立体射影
                    elif method == 1:
                        r_ =  math.tan((phi2) / 2)
                    #立体射影逆変換
                    elif method == 2:
                        r_ =  1 - math.tan((math.pi / 2 - phi2) / 2)
                    #正射影
                    elif method == 3:
                        r_ = math.sin(phi2)
                    #正射影逆変換
                    elif method == 4:
                        r_ = 1 - math.sin(math.pi / 2 - phi2)
                    map_x[y,x] = src_r * r_ * math.cos(theta2) + src_cx
                    map_y[y,x] = src_r * r_ * math.sin(theta2) + src_cy

                else:
                    #等距離射影
                    if method == 0:
                        r_ = (math.pi - phi2) / math.pi * 2
                    #立体射影
                    elif method == 1:
                        r_ =  math.tan((math.pi - phi2) / 2)
                    #立体射影逆変換
                    elif method == 2:
                        r_ =  1 - math.tan((-math.pi/2 + phi2) / 2)
                    #正射影
                    elif method == 3:
                        r_ = math.sin(math.pi - phi2)
                    #正射影逆変換
                    elif method == 4:
                        r_ = 1 - math.sin(- math.pi / 2 + phi2)
                    map_x[y,x] = src_r * r_ * math.cos(math.pi - theta2) + src_cx2
                    map_y[y,x] = src_r * r_ * math.sin(math.pi - theta2) + src_cy


        map_x = map_x.astype('float32')
        map_y = map_y.astype('float32')

        result_img = cv2.remap(cv_img, map_x, map_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT);

        img_msg = CvBridge().cv2_to_imgmsg(result_img, "bgr8")

        rospy.loginfo("### Converted ###")
        return DualFisheye2EquirectangularResponse(img_msg)

    def server(self, srv_msg):
        try:
            cv_img = CvBridge().imgmsg_to_cv2(srv_msg.dualFisheye, "bgr8")
            output_img = self.image_converter(cv_img)
            return output_img

        except CvBridgeError, e:
            rospy.logerr("%s",e)

if __name__ == "__main__":
    rospy.init_node("image_convert_server")
    rospy.loginfo("### [/ricoh_theta_ros/image_convert_server] ON ###")
    ics = ImageConvertServer()
    rospy.spin()
