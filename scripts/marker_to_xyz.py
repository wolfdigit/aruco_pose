#!/usr/bin/env python
import rospy
import visualization_msgs.msg
import tf
import numpy as np

rospy.init_node('marker_xyz', anonymous=True)

def fetchMarker(markerArr):
    for marker in markerArr.markers:
        #print(marker)
        sqr = np.array([[0,0,0,1], [1,0,0,1], [1,1,0,1], [0,1,0,1]]).T
        s = marker.scale
        sqr = np.matmul(np.array([[s.x,0,0,0], [0,s.y,0,0], [0,0,s.z,0], [0,0,0,1]]), sqr)
        #print(sqr)
        q = marker.pose.orientation
        rtmat = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        #print(rtmat)
        p = marker.pose.position
        rtmat[0:3,3:4] = np.array([[p.x], [p.y], [p.z]])
        #print(rtmat)
        pnts = np.matmul(rtmat, sqr)
        p = pnts[0:3,:].T
        print "%d\t[" % (marker.id),
        for i in range(4):
            print "[%.2f,%.2f,%.2f],"%(p[i,0], p[i,1], p[i,2]),
        print "],"
        #marker_dict[marker.id] = pnts[0:3,:].T
marker_sub = rospy.Subscriber("marker_seen", visualization_msgs.msg.MarkerArray, fetchMarker)

rospy.spin()
