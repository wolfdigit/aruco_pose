#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import visualization_msgs.msg
#from visualization_msgs.msg
import tf
#import numpy as np
import math

rospy.init_node('aruco_pose_marker_pub', anonymous=True)

def getObjPnts(tag_id):
    if tag_id==0:
        return np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]])
    else:
        return None

# yaw & pitch: world; roll: rotated (nose pointing)
def buildMarker(id, x, y, z, yaw, pitch, roll, newcome=False):
    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = '/map'
    marker.ns = 'aruco'
    marker.id = id
    marker.type = marker.CUBE
    marker.action = marker.DELETE if newcome else marker.ADD
    marker.scale.x = 0.06
    marker.scale.y = 0.06
    marker.scale.z = 0.01
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    q = tf.transformations.quaternion_from_euler(yaw/180.0*math.pi, (-pitch-90)/180.0*math.pi, (-roll)/180.0*math.pi, 'rzxz')
    marker.pose.orientation = geometry_msgs.msg.Quaternion(*q)
    marker.pose.position = geometry_msgs.msg.Point(x, y, z)
    return marker

markerArr = visualization_msgs.msg.MarkerArray()
markers = []
#markers.append(buildMarker(0, 0, 0, 0, 0, 0, 0))
#markers.append(buildMarker(40, 0, 0, 0, 0, 0, 0, True))
#markers.append(buildMarker(0, 0.00, 0.00, 0.00, -1.14, -1.79, 1.66))
#markers.append(buildMarker(40, 0.33, 0.05, -0.01, -1.44, -1.74, 1.53))
markers.append(buildMarker(24, 0, 0, 0, 0, 0, 0))
markers.append(buildMarker(53, 0, 0, 0, 0, 0, 0, True))
markers.append(buildMarker(29, 0, 0, 0, 0, 0, 0, True))
markers.append(buildMarker(71, 0, 0, 0, 0, 0, 0, True))
markers.append(buildMarker(52, 0, 0, 0, 0, 0, 0, True))
markers.append(buildMarker(70, 0, 0, 0, 0, 0, 0, True))
markers.append(buildMarker(27, 0, 0, 0, 0, 0, 0, True))

markerArr.markers = markers

markerArr_pub = rospy.Publisher('marker_ideal', visualization_msgs.msg.MarkerArray)
rate = rospy.Rate(3)
while not rospy.is_shutdown():
    markerArr_pub.publish(markerArr)
    rate.sleep()
