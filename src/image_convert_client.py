#! /usr/bin/env python
#coding:utf-8
import rospy
import numpy as np
import cv2
import math

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ricoh_theta_ros.srv import DualFisheye2Equirectangular

class ImageConvertClient():
    def __init__(self):
        self.img_sub = rospy.Subscriber("/ricoh_theta/image_raw", Image, self.img_sub_callback)
        self.img_pub = rospy.Publisher("/ricoh_theta/equirectangular/image_raw", Image, queue_size=10)

        self.img_msg = Image()

    def img_sub_callback(self, msg):
        #rospy.loginfo("### waiting for [ricoh_theta_ros/dual_fisheye2equirectangular] ###")
        #rospy.wait_for_service("ricoh_theta_ros/dual_fisheye2equirectangular")
        #try:
        #    srv = rospy.ServiceProxy("ricoh_theta_ros/dual_fisheye2equirectangular", DualFisheye2Equirectangular)
        #    result_img = srv(msg)
        #    cv_img = CvBridge().imgmsg_to_cv2(result_img, "bgr8")
        #    cv2.imshow("result", cv_img)
        #    cv2.waitKey(1)

        #except rospy.ServiceException, e:
        #    rospy.logerr("Service call failed: %s"%e)
        self.img_msg = msg

    def client(self):
        rospy.loginfo("### waiting for [/ricoh_theta_ros/dual_fisheye2equirectangular] ###")
        rospy.wait_for_service("/ricoh_theta_ros/dual_fisheye2equirectangular")
        try:
            srv = rospy.ServiceProxy("/ricoh_theta_ros/dual_fisheye2equirectangular", DualFisheye2Equirectangular)
            result_img = srv(self.img_msg)
            ## image publish
            self.img_pub.publish(result_img.equirectangular)
            rospy.loginfo("### equirectangular image publish ###")

            ## image show
            #cv_img = CvBridge().imgmsg_to_cv2(result_img.equirectangular, "bgr8")
            #cv2.imshow("result", cv_img)
            #cv2.waitKey(1)


        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.init_node("image_convert_client")

    icc = ImageConvertClient()

    while not rospy.is_shutdown():
        icc.client()
