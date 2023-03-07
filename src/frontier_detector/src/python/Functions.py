#!/usr/bin/env python3The filter nodes receives the detected frontier points from all the detectors,
# filters the points, and passes them to the assigner node to command the robots.
# Filtration includes the deletion of old and invalid points, and it also
# discards redundant points.

# jplaced
# 2022, Universidad de Zaragoza

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy

import numpy as np

from numpy.linalg import norm
from visualization_msgs.msg import Marker

try:
    import numba as nb
    from numba import cuda
    cuda.select_device(0)
except ModuleNotFoundError:
    print("Import error: Numba module not found")
except Exception as e:
    print("GPU not found: " + format(e))


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def yawBtw2Points(pointA: np.array, pointB: np.array) -> float:
    """
    Computes angle between two 2-dimensional vectors (radians)
    """
    changeInX = pointB[0] - pointA[0]
    changeInY = pointB[1] - pointA[1]
    return np.arctan2(changeInY, changeInX)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Returns grid value at "Xp" location
# Map data:  100 occupied      -1 unknown       0 free
def gridValue(map_data, Xp):
    resolution = map_data.info.resolution
    Xstartx = map_data.info.origin.position.x
    Xstarty = map_data.info.origin.position.y
    width = map_data.info.width
    Data = map_data.data
    index = (np.floor((Xp[1] - Xstarty) // resolution) * width) + (np.floor((Xp[0] - Xstartx) // resolution))
    return [100, Data[int(index)]][int(index) < len(Data)]


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Returns the mapData's index of a given point (x,y) in the map
def index_of_point(map_data, Xp):
    resolution = map_data.info.resolution
    Xstartx = map_data.info.origin.position.x
    Xstarty = map_data.info.origin.position.y
    width = map_data.info.width
    index = int((np.floor((Xp[1] - Xstarty) // resolution) * width) + (np.floor((Xp[0] - Xstartx) // resolution)))
    return index


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Returns the point (x,y) of a given mapData's index
def point_of_index(map_data, i):
    y = map_data.info.origin.position.y + (i // map_data.info.width) * map_data.info.resolution
    x = map_data.info.origin.position.x + (
            i - (i // map_data.info.width) * map_data.info.width) * map_data.info.resolution
    return np.array([x, y])


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# It is defined as the area of unknown region expected to be explored for a given
# frontier point.The information gain is quantified by counting the number of
# unknown cells surrounding a frontier point within a user defined radius. This
# radius is referred to as the information gain radius, which should be set to a
# value equal to the perception sensor range. The area is then calculated by
# multi-plying the number of cells within the information gain radius,by the
# area of each cell (which is computed from the map resolution).
def informationGain(mapData, point, r):
    infoGain = 0
    index = index_of_point(mapData, point)
    resolution = mapData.info.resolution
    width = mapData.info.width
    Data = mapData.data
    r_region = int(r // resolution)
    init_index = index - r_region * (width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * width + init_index
        end = start + 2 * r_region
        limit = ((start // width) + 2) * width
        for i in range(start, end + 1):
            if 0 <= i < len(Data) and i < limit:
                if Data[i] == -1 and norm(np.array(point) - point_of_index(mapData, i)) <= r:
                    infoGain += 1
    return infoGain * (resolution ** 2)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
@nb.jit(nb.float64(nb.float64, nb.int64, nb.float64, nb.float64, nb.int64[:], nb.float64, nb.float64, nb.int64),
        nopython=True)  # forceobj=True)
def informationGain_NUMBA(resolution, width, Xstartx, Xstarty, data, pointx, pointy, r):
    infoGain = 0
    index = int((np.floor((pointy - Xstarty) // resolution) * width) + (np.floor((pointx - Xstartx) // resolution)))
    r_region = int(r // resolution)
    init_index = index - r_region * (width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * width + init_index
        end = start + 2 * r_region
        limit = ((start // width) + 2) * width
        for i in range(start, end + 1):
            if 0 <= i < len(data) and i < limit:
                y = Xstarty + (i // width) * resolution
                x = Xstartx + (i - (i // width) * width) * resolution
                poi = np.array([x, y])
                if data[i] == -1 and norm(np.array([pointx, pointy]) - poi) <= r:
                    infoGain += 1
    return infoGain * (resolution ** 2)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def wait_enterKey():
    input("Press Enter to continue...")


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def createMarker(type="point", frame="/map", ns="marker_ns", lifetime=0.12, colors=[255, 0, 0], alpha=1.0, scale=0.3):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = 0
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.color.a = alpha
    marker.color.r = colors[0]
    marker.color.g = colors[1]
    marker.color.b = colors[2]
    marker.lifetime = rospy.Duration(lifetime)
    marker.scale.x = marker.scale.y = scale

    if type == "sphere":
        marker.type = Marker.SPHERE
        marker.scale.z = 0.05
        marker.pose.position.z = 0.0
    elif type == "point":
        marker.type = Marker.POINTS

    return marker
