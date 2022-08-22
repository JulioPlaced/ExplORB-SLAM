#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# This node initializes the frontier detectors with a pre defined starting map
# size.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import os

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

MAP_SIZE = 25


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def init_point_selector(i):
    points = [[-MAP_SIZE, MAP_SIZE], [-MAP_SIZE, -MAP_SIZE], [MAP_SIZE, -MAP_SIZE], [MAP_SIZE, MAP_SIZE],
              [0.140955, -0.0519512]]

    five_points = []
    for (x, y) in points:
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        five_points.append(p)

    init_points = Marker()
    init_points.header.frame_id = "map"
    init_points.header.stamp = rospy.get_rostime()
    init_points.points = five_points[0:i]

    return init_points


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node():
    rospy.init_node('initializer', anonymous=False)

    rate_hz = rospy.get_param('~rate', 1)
    rate = rospy.Rate(rate_hz)
    marker_pub = rospy.Publisher('init_points', Marker, queue_size=5)

    i = 0
    while not rospy.is_shutdown() and i <= 5:
        if i > 5:
            i = 5
        init_points = init_point_selector(i)
        marker_pub.publish(init_points)
        i += 1
        rate.sleep()

    rospy.loginfo(rospy.get_name() + ": Shutting down node.")
    try:
        os.system("rosnode kill /point_init")
    except KeyError:
        pass


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
