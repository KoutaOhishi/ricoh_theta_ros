#! /usr/bin/env python
#coding:utf-8
import io
import json
import inspect
import time
import cv2
import numpy as np
import requests
import signal
import sys
import roslib
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class WifiStreamer(): #API ver2.0
    def __init__(self):
        self.exe = "http://192.168.1.1:80/osc/commands/execute"
        self.img_pub = rospy.Publisher("/ricoh_theta/equirectangular/image_raw", Image, queue_size=1)

    def handler(self, signal, frame):
        rospy.loginfo("### Shutting Down... ###")
        sys.exit(0)

    def startSession(self):
        j = {'name': 'camera.{}'.format(inspect.currentframe().f_code.co_name), 'parameters': {}}
        r = requests.post(self.exe, data=json.dumps(j))
        rospy.loginfo("### Start Session ###")
        #rospy.loginfo(r.json())
        return r.json()

    def closeSession(self, id):
        j = {'name': 'camera.{}'.format(inspect.currentframe().f_code.co_name), 'parameters': {'sessionId': '{}'.format(id)}}
        r = requests.post(self.exe, data=json.dumps(j))
        rospy.loginfo("### Close Session ###")
        #rospy.loginfo(r.json())
        return r.json()

    def _getLivePreview(self, id):
        j = {'name':'camera.{}'.format(inspect.currentframe().f_code.co_name),
        'parameters': {'sessionId': id}}
        #rospy.loginfo(j)
        r = requests.post(self.exe, data=json.dumps(j), stream=True)

        bytes = b''
        for byteData in r.iter_content(chunk_size=1024):
            bytes += byteData
            a = bytes.find(b'\xff\xd8')
            b = bytes.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes[a:b+2]
                bytes = bytes[b+2:]
                try:
                    i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    self.img_pub.publish(CvBridge().cv2_to_imgmsg(i, "bgr8"))
                except:
                    rospy.logerr("OpenCV Error")

        return r.json()

    def main(self):
        signal.signal(signal.SIGINT, self.handler)

        d = self.startSession()
        if d['state'] == 'error':
            return self.closeSession("SID_001")

        id = d["results"]["sessionId"]
        img = self._getLivePreview(id)

        self.closeSession(id)


if __name__ == "__main__":
    rospy.init_node("wifi_streamer")
    ws = WifiStreamer()
    ws.main()
