#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import numpy as np

from nptyping import ndarray
from typing import Tuple

from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

# NUMBA imports for GPU computations
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
def waitEnterKey():
    input("Press Enter to continue...")


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def euler2quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    Converts Euler angles (radians) to quaternion
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return qx, qy, qz, qw


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def quaternion2euler(w: float, x: float, y: float, z: float):
    """
    Converts quaternion to Euler angles (radians)
    """
    y2 = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y2)
    X = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2 > +1.0, +1.0, t2)

    t2 = np.where(t2 < -1.0, -1.0, t2)
    Y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y2 + z * z)
    Z = np.arctan2(t3, t4)

    return X, Y, Z


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def yawBtw2Points(pointA: np.array, pointB: np.array) -> float:
    """
    Computes angle between two 2-dimensional vectors (radians)
    """
    changeInX = pointB[0] - pointA[0]
    changeInY = pointB[1] - pointA[1]
    return np.arctan2(changeInY, changeInX)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def createMarker(mtype: str = "point", frame: str = "/map", ns: str = "marker_ns", lifetime: float = 0,
                 colors: ndarray = None, alpha: float = 1.0, scale: float = 0.3) -> Marker:
    """
    Initializes a ROS visualization_msgs Marker
    """
    if colors is None:
        colors = [255, 0, 0]

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

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    if mtype == "point":
        marker.type = Marker.POINTS
        marker.scale.x = marker.scale.y = scale
    elif mtype == "sphere":
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = scale  # Diameter
    elif mtype == "arrow":
        marker.type = Marker.ARROW
        marker.scale.x = scale  # Arrow length
        marker.scale.y = marker.scale.z = 0.05  # Arrow head diameter and length
    elif mtype == "cube":
        marker.type = Marker.CUBE
        marker.scale.x = marker.scale.y = marker.scale.z = scale
    elif mtype == "circumference":
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = scale
        marker.scale.z = 0.05
        marker.pose.position.z = 0.0
    elif mtype == "lines":
        marker.type = Marker.LINE_STRIP
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.scale.x = scale

    return marker


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
def cellInformation(mapData: OccupancyGrid, point: np.array, r: float):
    cells = [0, 0, 0]  # every cell, unknown cells, occupied cells
    index = index_of_point(mapData, point)
    r_region = int(r // mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)

    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = start + 2 * mapData.info.width
        for i in range(start, end + 1):
            if 0 <= i < np.min([limit, len(mapData.data)]) and \
                    np.linalg.norm(np.array(point) - point_of_index(mapData, i)) <= r:
                cells[0] += 1
                if mapData.data[i] == -1:
                    cells[1] += 1  # Unknown

    # Normalized information gain due to unknown region
    percentage_new_cells = (float(cells[1]) / float(cells[0]))

    return percentage_new_cells


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Returns the mapData's index of a given point (x,y) in the map
def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    index = int((np.floor((Xp[1] - Xstarty) // resolution) * width) + (np.floor((Xp[0] - Xstartx) // resolution)))
    return index


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Returns the point (x,y) of a given mapData's index
def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + (i // mapData.info.width) * mapData.info.resolution
    x = mapData.info.origin.position.x + (
            i - (i // mapData.info.width) * mapData.info.width) * mapData.info.resolution
    return np.array([x, y])


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# NUMBA FUNCTIONS
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

@nb.jit(nb.float64(nb.int64[:], nb.float64, nb.int64, nb.float64, nb.float64, nb.float64,
                   nb.float64, nb.float64), nopython=True)
def cellInformation_NUMBA(data, resolution, width, Xstartx, Xstarty, pointx, pointy, r):
    cells = [0, 0, 0]  # every cell, unknown cells, occupied cells
    index = int((np.floor((pointy - Xstarty) // resolution) * width) + (np.floor((pointx - Xstartx) // resolution)))
    r_region = int(r // resolution)
    init_index = index - r_region * (width + 1)

    for n in range(0, 2 * r_region + 1):
        start = n * width + init_index
        end = start + 2 * r_region
        limit = start + 2 * width
        for i in range(start, end + 1):
            if 0 <= i < len(data) and i < limit:
                y = Xstarty + (i // width) * resolution
                x = Xstartx + (i - (i // width) * width) * resolution
                poi = np.array([x, y])
                if np.linalg.norm(np.array([pointx, pointy]) - poi) <= r:
                    cells[0] += 1
                    if data[i] == -1:
                        cells[1] += 1  # Unknown

    # Normalized information gain due to unknown region
    percentage_new_cells = (float(cells[1]) / float(cells[0]))

    return percentage_new_cells
