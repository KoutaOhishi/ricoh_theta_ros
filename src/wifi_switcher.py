#!/usr/bin/env python
#coding:utf-8
import rospy
import subprocess
import roslib.packages

# WiFi接続をthetaに切り替えるノード

if __name__ == "__main__":
    rospy.init_node("wifi_switcher")

    package_path = roslib.packages.get_pkg_dir("ricoh_theta_ros")
    file_path = package_path + "/config/wifi_switcher.sh"

    subprocess.call(["sh", file_path])
