#!/usr/bin/env python
# -*- coding: utf-8 -*-
from trajectory_viz_python.srv import *
import rospy
import random
import webcolors
from std_msgs.msg import ColorRGBA, Float32
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from jsk_rviz_plugins.msg import *

import sys

points_dict = dict()
points_color = dict()
markerArray = MarkerArray()
pointsArray = MarkerArray()

def closest_colour(requested_colour):
    min_colours = {}
    for key, name in webcolors.css3_hex_to_names.items():
        r_c, g_c, b_c = webcolors.hex_to_rgb(key)
        rd = (r_c - requested_colour[0]) ** 2
        gd = (g_c - requested_colour[1]) ** 2
        bd = (b_c - requested_colour[2]) ** 2
        min_colours[(rd + gd + bd)] = name
    return min_colours[min(min_colours.keys())]

def get_colour_name(requested_colour):
    try:
        closest_name = actual_name = webcolors.rgb_to_name(requested_colour)
    except ValueError:
        closest_name = closest_colour(requested_colour)
        actual_name = None
    return actual_name, closest_name

def callback(req):
    ret = 1
    p = Point(req.x, req.y, 0.0)

    if req.name in points_dict:
        try:
            markerArray.markers[points_dict[req.name]].points.append(p)
            pointsArray.markers[points_dict[req.name]].points.append(p)
        except:
            rospy.logerr("eception adding new marker to markers")
    else:
        m = Marker()
        m.header.frame_id = "/gnss"
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.1
        m.color.a = 0.5
        m.color.r = random.random()#1.0
        m.color.g = random.random()#0.0
        m.color.b = random.random()#0.0
        rr, g, b = m.color.r * 255.0, m.color.g * 255.0, m.color.b * 255.0
        requested_colour = (int(rr), int(g), int(b))
        _, closest_name = get_colour_name(requested_colour)
        points_color[req.name] = closest_name
        m.points.append(p)
        m.id = len(markerArray.markers)

        m2 = Marker()
        m2.header.frame_id = "/gnss"
        m2.type = Marker.POINTS
        m2.action = Marker.ADD
        m2.scale.x = 0.3
        m2.scale.y = 0.3
        m2.scale.z = 0.3
        m2.color.a = 0.5
        m2.color = m.color
        m2.id = len(pointsArray.markers)

        points_dict[req.name] = len(markerArray.markers)
        try:
            markerArray.markers.append(m)
            pointsArray.markers.append(m2)
        except:
            rospy.logerr("exception adding new markers")
    
    print "new point of %s is added to (%s, %s)\n"%(req.name, req.x, req.y)
    #for key, value in points_dict.items():
    #    print key, value
    #print markerArray

    return AddNewPointResponse(ret)

text = OverlayText()
text.width = 100
text.height = 300
#text.height = 600
text.left = 10
text.top = 10
text.text_size = 12
text.line_width = 2
text.font = "DejaVu Sans Mono"
text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
text.bg_color = ColorRGBA(1.0, 1.0, 1.0, 0.8)

def main():
    rospy.init_node('trajectory_viz_python', anonymous=True)

    s = rospy.Service('trajectory_server', AddNewPoint, callback)

    markerArray.markers = list()
    marker_pub = rospy.Publisher("trajectory_viz_marker",MarkerArray,queue_size=1)
    points_pub = rospy.Publisher("trajectory_viz_points",MarkerArray,queue_size=1)
    text_pub = rospy.Publisher("text_sample", OverlayText, queue_size=1)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        text.text = ""
        for sat, idx in points_dict.items():
            m = markerArray.markers[idx]
            text.text += """<span style="color: %s;">%s</span>\n
            """ % (points_color[sat], sat)

        text_pub.publish(text)

        marker_pub.publish(markerArray)
        points_pub.publish(pointsArray)

        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass

