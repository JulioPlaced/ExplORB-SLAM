#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# This node draws the pose-graph in RViZ as a marker array. Only for visualization purposes.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import tf
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float64MultiArray

from Map import Map

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Callbacks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
vertices_ = []
edges_ = []


def vertexCallBack(data):
    global vertices_
    n = data.layout.dim[0].stride

    vertices_ = []
    for i in range(0, len(data.data), n):
        vertices_.append(data.data[i:i + n])


def edgesCallBack(data):
    global edges_
    n = data.layout.dim[0].stride

    edges_ = []
    for i in range(0, len(data.data), n):
        edges_.append(data.data[i:i + n])


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Node~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def node():
    global vertices_, edges_

    rospy.init_node('hall_graph_pub', anonymous=True)
    rate = rospy.Rate(1)

    camera_type = rospy.get_param('~camera_type', 'rgbd')

    rospy.Subscriber("/orb_slam2_" + camera_type + "/vertex_list", Float64MultiArray, vertexCallBack)
    rospy.Subscriber("/orb_slam2_" + camera_type + "/edge_list", Float64MultiArray, edgesCallBack)

    marker_graph_pub_ = rospy.Publisher('marker_G', MarkerArray, queue_size=10)
    tf_listener = tf.TransformListener()

    map_ = Map()

    # Get tf from camera link to base frame
    cond = 0
    while cond == 0:
        try:
            (t_camera_base, q_camera_base) = tf_listener.lookupTransform("base_footprint", "camera_link_optical",
                                                                         rospy.Time(0))
            map_.setCameraBaseTf(t_camera_base, q_camera_base)
            cond = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr(rospy.get_name() + ": Could not get TF from base to camera link.")
            cond = 0

    while not rospy.is_shutdown():
        map_.setNodes(vertices_)
        map_.setEdges(edges_)
        marker_graph_pub_.publish(map_.getGraphAsMarkerArray(only_nodes=False, color=False))

    rate.sleep()


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
