#!/usr/bin/env python

import rospy
import sensor_msgs
import geometry_msgs
import visualization_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import tf
import cv2
import cv2.aruco
import numpy as np
import math

rospy.init_node('aruco_pose', anonymous=True)

tag_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

marker_dict = {}
marker_shape = {}
marker_scale = {}
def fetchMarker(markerArr):
    marker_dict.clear()
    marker_shape.clear()
    marker_scale.clear()
    for marker in markerArr.markers:
        #print(marker)
        sqr = np.array([[0,0,0,1], [1,0,0,1], [1,-1,0,1], [0,-1,0,1]]).T
        s = marker.scale
        sqr = np.matmul(np.array([[s.x,0,0,0], [0,s.y,0,0], [0,0,s.z,0], [0,0,0,1]]), sqr)
        #print(sqr)
        marker_shape[marker.id] = sqr[0:3, :].T
        marker_scale[marker.id] = marker.scale
        if marker.action!=marker.ADD:
            continue
        q = marker.pose.orientation
        rtmat = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        #print(rtmat)
        p = marker.pose.position
        rtmat[0:3,3:4] = np.array([[p.x], [p.y], [p.z]])
        #print(rtmat)
        pnts = np.matmul(rtmat, sqr)
        #print(pnts)
        marker_dict[marker.id] = pnts[0:3,:].T
marker_sub = rospy.Subscriber("marker_ideal", visualization_msgs.msg.MarkerArray, fetchMarker)

def getObjPnts(tag_id):
    tag_id = int(tag_id)
    #print(marker_dict)
    if tag_id in marker_dict:
        return marker_dict[tag_id]
    else:
        return None
    #if tag_id==0:
    #    return np.array([[0, 0, 0], [0.031, 0, 0], [0.031, 0.031, 0], [0, 0.031, 0]])
    #else:
    #    return None
def getObjShape(tag_id):
    tag_id = int(tag_id)
    if tag_id in marker_shape:
        return marker_shape[tag_id]
    else:
        return None

pose_pub = rospy.Publisher('camera_pose', geometry_msgs.msg.PoseStamped)
def publishTransform(mat4):
    quaternion = tf.transformations.quaternion_from_matrix(mat4)
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = '/map'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position = geometry_msgs.msg.Point(*mat4[0:3, 3])
    pose.pose.orientation = geometry_msgs.msg.Quaternion(*quaternion)
    pose_pub.publish(pose)

def buildMarker(tag_id, mat4):
    tag_id = int(tag_id)
    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = '/map'
    marker.ns = 'aruco'
    marker.id = tag_id
    marker.type = marker.CUBE
    marker.action = marker.MODIFY
    marker.scale = marker_scale[tag_id]
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    q = tf.transformations.quaternion_from_matrix(mat4)
    marker.pose.orientation = geometry_msgs.msg.Quaternion(*q)
    marker.pose.position = geometry_msgs.msg.Point(mat4[0,3], mat4[1,3], mat4[2,3])
    return marker

mat4_cw = np.identity(4)
mat4_tc = np.identity(4)
image_pub = rospy.Publisher('image_out', sensor_msgs.msg.Image)
markerArr_pub = rospy.Publisher('marker_seen', visualization_msgs.msg.MarkerArray)
bridge = CvBridge()
subPixCriteria = criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
rvec = None
tvec = None
def callback(data):
    global rvec, tvec
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
            #if rvec is None or tvec is None:
            #    retv, rvec, tvec = cv2.solvePnP( objPnts, imgPnts, cameraMatrix, distCoeffs, rvec, tvec, False )
            #else:
            #    retv, rvec, tvec = cv2.solvePnP( objPnts, imgPnts, cameraMatrix, distCoeffs, rvec, tvec, True )
            retv, rvec, tvec = cv2.solvePnP( objPnts, imgPnts, cameraMatrix, distCoeffs )
            #print(rvec)
            #print(tvec)
            rmat, _ = cv2.Rodrigues(rvec)
            #print(rmat)
            mat4_cw[0:3, 0:3] = rmat.T
            mat4_cw[0:3, 3:4] = np.matmul(rmat.T, -tvec)
            publishTransform(mat4_cw)
            
            tagOutFile = open('/tmp/marker_seen.txt', 'w')
            markers_out = []
            tagOutFile.write("{}, {}, {}, {}, {}, {}, {}\n".format('id', 'x', 'y', 'z', 'yaw', 'pitch', 'roll'))
            for i in range(len(tag_ids)):
                objPnts = getObjShape(tag_ids[i])
                if objPnts is None:
                    continue
                imgPnts = subcorners[i][0, :, :]
                #print(objPnts)
                #print(imgPnts)
                retv, rvec2, tvec2 = cv2.solvePnP( objPnts, imgPnts, cameraMatrix, distCoeffs )
                rmat2, _ = cv2.Rodrigues(rvec2)
                mat4_tc[0:3, 0:3] = rmat2
                mat4_tc[0:3, 3:4] = tvec2
                mat4_tw = np.matmul(mat4_cw, mat4_tc)
                mrk = buildMarker(tag_ids[i], mat4_tw)
                markers_out.append(mrk)
                q = mrk.pose.orientation
                p = mrk.pose.position
                euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w), axes='rzxz')
                tagOutFile.write("{}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}\n".format(mrk.id, p.x, p.y, p.z, euler[0]/math.pi*180.0, euler[1]/math.pi*180.0, euler[2]/math.pi*180.0))
            tagOutFile.close()
            markerArr = visualization_msgs.msg.MarkerArray()
            markerArr.markers = markers_out
            markerArr_pub.publish(markerArr)
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
image_sub = rospy.Subscriber("/usb_cam/image_raw", sensor_msgs.msg.Image, callback)

map1 = None
map2 = None
cameraMatrix = None
distCoeffs = None
def cb_sub_intrinsic(camInfo):
    global map1, map2, cameraMatrix, distCoeffs
    cameraMatrix = np.array(camInfo.K).reshape((3,3))
    R = np.array(camInfo.R).reshape((3,3))
    newCameraMatrix = np.array(camInfo.P).reshape((3,4))
    (distCoeffs, size) = (camInfo.D, (camInfo.width, camInfo.height))
    #print(data.K)   # intrinsic for raw img
    #print(data.D)   # distortion
    #print(data.P)   # intrinsic for rectified img
    m1type = cv2.CV_32FC2
    map1, map2 = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, R, newCameraMatrix, size, m1type)
    info_sub.unregister()
info_sub = rospy.Subscriber("/usb_cam/camera_info", sensor_msgs.msg.CameraInfo, cb_sub_intrinsic)



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
