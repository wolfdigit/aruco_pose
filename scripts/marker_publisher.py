#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import visualization_msgs.msg
import tf
import math

rospy.init_node('aruco_pose_marker_pub', anonymous=True)

# yaw & pitch: world; roll: rotated (nose pointing)
def buildMarker(id, x, y, z, yaw, pitch, roll, newcome=False):
    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = '/map'
    marker.ns = 'aruco'
    marker.id = id
    marker.type = marker.CUBE
    marker.action = marker.DELETE if newcome else marker.ADD
    marker.scale.x = 0.08
    marker.scale.y = 0.08
    marker.scale.z = 0.01
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    q = tf.transformations.quaternion_from_euler(yaw/180.0*math.pi, pitch/180.0*math.pi, roll/180.0*math.pi, 'rzxz')
    marker.pose.orientation = geometry_msgs.msg.Quaternion(*q)
    marker.pose.position = geometry_msgs.msg.Point(x, y, z)
    return marker

def loadMarkers(filename):
    markers = []
    marker_ids = []
    file = open(filename, 'r')
    for line in file.readlines():
        tok = line.split(',')
        if not tok[0].isdigit():
            continue
        if len(tok)!=7:
            print("ERROR: marker csv length should equal 7!")
        markers.append(buildMarker( int(tok[0]), float(tok[1]), float(tok[2]), float(tok[3]), float(tok[4]), float(tok[5]), float(tok[6]) ))
        marker_ids.append(int(tok[0]))
    file.close()
    for i in range(0, 100):
        if not i in marker_ids:
            markers.append(buildMarker(i, 0, 0, 0, 0, 0, 0, True))
    markerArr = visualization_msgs.msg.MarkerArray()
    markerArr.markers = markers
    return markerArr

markerArr_pub = rospy.Publisher('marker_ideal', visualization_msgs.msg.MarkerArray)
rate = rospy.Rate(3)
while not rospy.is_shutdown():
    markerArr_pub.publish(loadMarkers(rospy.get_param("~marker_known")))
    rate.sleep()
