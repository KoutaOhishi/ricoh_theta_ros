#!/usr/bin/env python
#coding:utf-8
import rospy
import subprocess
import roslib.packages

if __name__ == "__main__":
    rospy.init_node("permission")

    package_path = roslib.packages.get_pkg_dir("ricoh_theta_ros")
    file_path = package_path + "/src/permission.sh"

    subprocess.call(["sh", file_path])
