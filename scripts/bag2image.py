#!/usr/bin/python3.8
import ros
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import message_filters
import cv2 as cv
import os

dataDir = "20200323"

conv = CvBridge()

def sync_save(image1, image2):
    imgl = conv.imgmsg_to_cv2(image1)
    imgr = conv.imgmsg_to_cv2(image2)
    namel = str(image1.header.stamp.secs) + '.' + str(image1.header.stamp.nsecs) + '.jpg'
    namer = str(image1.header.stamp.secs) + '.' + str(image1.header.stamp.nsecs) + '.jpg'
    dirl = "/media/E/All_codes/A_lab/A_grad/stero_slam/slam_catkin_ws/src/stereo_dso_ros/data/" + dataDir + "/image_0"
    dirr = "/media/E/All_codes/A_lab/A_grad/stero_slam/slam_catkin_ws/src/stereo_dso_ros/data/" + dataDir + "/image_1"
    if not os.path.exists(dirl):
        os.makedirs(dirl)
    if not os.path.exists(dirr):
        os.makedirs(dirr)
    
    cv.imwrite(dirl + '/' + namel, imgl)
    cv.imwrite(dirr + '/' + namer, imgr)

    rospy.loginfo_once("image saved to " + dirl + '/' + namel)
    rospy.loginfo_once("image saved to " + dirr + '/' + namer)

if __name__ == "__main__":
    rospy.init_node("image_save")
    imageL_sub = message_filters.Subscriber("/module/image_left", Image, queue_size=10)
    imageR_sub = message_filters.Subscriber("/module/image_right", Image, queue_size=10)
    ts = message_filters.ApproximateTimeSynchronizer([imageL_sub, imageR_sub], 10, 0.5)
    ts.registerCallback(sync_save)
    rospy.spin()
