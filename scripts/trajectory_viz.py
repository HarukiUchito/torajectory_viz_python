#!/usr/bin/env python
# -*- coding: utf-8 -*-
from trajectory_viz_python.srv import *
import rospy
import random
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

import sys

points_dict = dict()
markerArray = MarkerArray()

def callback(req):
    ret = 1
    print "new point of %s is added to (%s, %s)"%(req.name, req.x, req.y)
    p = Point(req.x, req.y, 0.0)

    if req.name in points_dict:
        markerArray.markers[points_dict[req.name]].points.append(p)
    else:
        m = Marker()
        m.header.frame_id = "/gnss"
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3
        m.color.a = 1.0
        m.color.r = random.random()#1.0
        m.color.g = random.random()#0.0
        m.color.b = random.random()#0.0
        m.points.append(p)
        m.id = len(markerArray.markers)
        points_dict[req.name] = len(markerArray.markers)
        markerArray.markers.append(m)
    
    for key, value in points_dict.items():
        print key, value
    print markerArray

    return AddNewPointResponse(ret)


def main():
    rospy.init_node('trajectory_viz_python', anonymous=True)

    s = rospy.Service('trajectory_server', AddNewPoint, callback)

    markerArray.markers = list()
    marker_pub = rospy.Publisher("trajectory_viz_marker",MarkerArray,queue_size=1)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        #for v in markerArray.markers:
         #   v.header.stamp = rospy.Time.now()
        marker_pub.publish(markerArray)
#        marker_pub.publish(points)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

