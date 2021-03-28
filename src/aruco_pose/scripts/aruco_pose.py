#!/usr/bin/env python

import rospy
import sensor_msgs
import geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
import tf
import cv2
import cv2.aruco
import numpy as np

rospy.init_node('aruco_pose', anonymous=True)

tag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

def getObjPnts(tag_id):
    if tag_id==0:
        return np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]])
    else:
        return None

pose_pub = rospy.Publisher('tf_out', geometry_msgs.msg.PoseStamped)
rmat4 = np.identity(4)
def publishTransform(rmat, tvec):
    rmat4[:3, :3] = rmat
    quaternion = tf.transformations.quaternion_from_matrix(rmat4)
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position = geometry_msgs.msg.Point(*tvec)
    pose.pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)
    pose_pub.publish(pose)

image_pub = rospy.Publisher('image_out', sensor_msgs.msg.Image)
bridge = CvBridge()
subPixCriteria = criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
def callback(data):
    if cameraMatrix is None:
        return
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    if map1 is not None:
        img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
    #cv2.imshow('raw', img)
    #cv2.waitKey(1)
    (corners, tag_ids, rej_corners) = cv2.aruco.detectMarkers(img, tag_dict)
    #print(tag_ids, reject_tags)
    #cv2.aruco.drawDetectedMarkers(img, rej_corners, np.array([]), np.array([255, 255, 0]))
    grayImg = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    if len(corners)>0:
        flatCorners = None
        for i in range(len(corners)):
            if i==0:
                flatCorners = corners[i]
            else:
                flatCorners = np.concatenate((flatCorners, corners[i]), axis=1)
        flatCorners = cv2.cornerSubPix(grayImg, flatCorners, (5,5), (-1,-1), subPixCriteria)
        subcorners = []
        for i in range(0, flatCorners.shape[1], 4):
            subcorners.append(flatCorners[:, i:i+4, :])
        #print(subcorners)
        cv2.aruco.drawDetectedMarkers(img, subcorners, tag_ids, np.array([0, 255, 0]))

        objPnts = np.zeros((0,3))
        imgPnts = np.zeros((0,2))
        for i in range(len(tag_ids)):
            newPnts = getObjPnts(tag_ids[i])
            if newPnts is not None:
                objPnts = np.concatenate((objPnts, newPnts))
                imgPnts = np.concatenate((imgPnts, subcorners[i][0, :, :]))
        #print(objPnts)
        #print(imgPnts)
        if objPnts.shape[0]>0:
            retv, rvec, tvec = cv2.solvePnP( objPnts, imgPnts, cameraMatrix, distCoeffs )
            print(rvec)
            print(tvec)
            rmat, _ = cv2.Rodrigues(rvec)
            print(rmat)
            publishTransform(np.linalg.inv(rmat), -tvec)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
image_sub = rospy.Subscriber("/usb_cam/image_raw", sensor_msgs.msg.Image, callback)

map1 = None
map2 = None
cameraMatrix = None
distCoeffs = None
def callback2(data):
    global map1, map2, cameraMatrix, distCoeffs
    cameraMatrix = np.array(data.K).reshape((3,3))
    R = np.array(data.R).reshape((3,3))
    newCameraMatrix = np.array(data.P).reshape((3,4))
    (distCoeffs, size) = (data.D, (data.width, data.height))
    #print(data.K)   # intrinsic for raw img
    #print(data.D)   # distortion
    #print(data.P)   # intrinsic for rectified img
    m1type = cv2.CV_32FC2
    map1, map2 = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, R, newCameraMatrix, size, m1type)
    info_sub.unregister()
info_sub = rospy.Subscriber("/usb_cam/camera_info", sensor_msgs.msg.CameraInfo, callback2)


rospy.spin()



_='''
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('aruco_pose', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  #cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
'''
