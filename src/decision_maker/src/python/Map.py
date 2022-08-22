#!/usr/bin/env python3

# jplaced
# jjgomez
# 2022, Universidad de Zaragoza

# Map class

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

import numpy as np
import rospy
import sophus as sp

from scipy.spatial.transform import Rotation
from typing import Tuple

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
from sensor_msgs import point_cloud2

from Functions import createMarker


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Class~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class Map:
    def __init__(self):
        """
        Constructor
        """

        # KF (vertices) poses
        self.Tcw = {}
        self.nKFs = 0

        # Map points positions and observability
        self.p3D = {}
        self.mpObs = {}
        self.nMPs = 0

        # Camera transformations
        t0 = [0., 0., 0.]
        rot0 = Rotation.from_quat([0., 0., 0., 1.])
        self.rot_camera_base = rot0
        self.t_camera_base = t0
        self.T_camera_base = sp.SE3(self.rot_camera_base.as_dcm(), self.t_camera_base)

        # Add KF at origin (kfId = 0)
        self.Tcw['0'] = sp.SE3(rot0.as_dcm(), t0)

        # Calibration values
        camera_info_topic = rospy.get_param('/decision_maker/cameraInfo_topic', '/robot_1/camera/rgb/camera_info')
        camera_info = rospy.wait_for_message(camera_info_topic, CameraInfo, timeout=5)

        # Intrinsic camera matrix for the raw (distorted) images.
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        self.fx = camera_info.K[0]
        self.fy = camera_info.K[4]
        self.cx = camera_info.K[2]
        self.cy = camera_info.K[5]
        self.imRows = camera_info.height
        self.imCols = camera_info.width

        self.edgeList = []

    def setCameraBaseTf(self, t_camera_base, quat_camera_base):
        """
        Sets the transformation between the camera and the robot base
        :param t_camera_base:
        :param quat_camera_base:
        """
        self.rot_camera_base = Rotation.from_quat(quat_camera_base)
        self.t_camera_base = t_camera_base
        self.T_camera_base = sp.SE3(self.rot_camera_base.as_dcm(), self.t_camera_base)

    def setEdges(self, edges_in: list):
        """
        Sets edges from information from ORB-SLAM in the following form:
            ID1, ID2, FIM (in R^36)
        Saved information in the form:
            edges[node1, node2, FIM]
        """
        edges = []
        H = np.zeros([6, 6])

        for data in edges_in:
            if len(data) != 38:
                print("Error reading edges file line no. " + format(lines.index(line)))
            else:
                H[0, 0] = float(data[2])
                H[0, 1] = float(data[3])
                H[0, 2] = float(data[4])
                H[0, 3] = float(data[5])
                H[0, 4] = float(data[6])
                H[0, 5] = float(data[7])
                H[1, 0] = float(data[8])
                H[1, 1] = float(data[9])
                H[1, 2] = float(data[10])
                H[1, 3] = float(data[11])
                H[1, 4] = float(data[12])
                H[1, 5] = float(data[13])
                H[2, 0] = float(data[14])
                H[2, 1] = float(data[15])
                H[2, 2] = float(data[16])
                H[2, 3] = float(data[17])
                H[2, 4] = float(data[18])
                H[2, 5] = float(data[19])
                H[3, 0] = float(data[20])
                H[3, 1] = float(data[21])
                H[3, 2] = float(data[22])
                H[3, 3] = float(data[23])
                H[3, 4] = float(data[24])
                H[3, 5] = float(data[25])
                H[4, 0] = float(data[26])
                H[4, 1] = float(data[27])
                H[4, 2] = float(data[28])
                H[4, 3] = float(data[29])
                H[4, 4] = float(data[30])
                H[4, 5] = float(data[31])
                H[5, 0] = float(data[32])
                H[5, 1] = float(data[33])
                H[5, 2] = float(data[34])
                H[5, 3] = float(data[35])
                H[5, 4] = float(data[36])
                H[5, 5] = float(data[37])

                i = int(data[0])
                j = int(data[1])
                if i != 0 and j != 0:  # Not including KF 0
                    edges.append([i, j, H])

        self.edgeList = edges

    def setNodes(self, nodes: list):
        """
        Sets nodes from information from ORB-SLAM in the following form:
            ID, pose in SE3 (qw, qx, qy, qz, x, y, z)
        """

        for data in nodes:
            kfId = int(data[0])
            rot_kf = Rotation.from_quat([float(data[2]), float(data[3]), float(data[4]), float(data[1])])
            t_kf = np.array([float(data[5]), float(data[6]), float(data[7])])

            # Transform to ROS coordinate system on camera coordinates
            rot_cw = self.rot_camera_base * rot_kf
            tcw = self.rot_camera_base.apply(t_kf, inverse=False)
            # Inverse matrix
            rot_cw = rot_cw.inv()
            tcw = - (rot_cw.apply(tcw, inverse=False))
            # Transform to ROS coordinate system on base coordinates
            rot_cw = self.rot_camera_base * rot_cw
            tcw = self.rot_camera_base.apply(tcw, inverse=False)

            Tcw = sp.SE3(rot_cw.as_dcm(), tcw)

            self.nKFs = max(self.nKFs, int(kfId))
            self.Tcw[kfId] = Tcw

    def setMapPoints(self, mps: list):
        """
        Sets map points, saving all points with their R^3 pose and covisible KFs, from information from ORB-SLAM in
         the following form: Id, pose in R^3 (x,y,z), FIM (in R^6)
        """

        for data in mps:
            mpId = int(data[0])
            p3Dw = np.array([float(data[1]), float(data[2]), float(data[3])])

            # Points are in map frame. We need to transform from cam to base frame to take their offset into account
            p3Dw = np.asarray(p3Dw, dtype='float64')
            p3Dw = self.T_camera_base * p3Dw
            # p3Dw = self.rot_camera_base.apply(p3Dw, inverse=False)
            # p3Dw += self.t_camera_base

            obs = []
            for i in range(4, np.size(data)):
                obs.append(int(data[i]))

            self.nMPs = max(self.nMPs, int(mpId))
            self.p3D[mpId] = p3Dw
            self.mpObs[mpId] = obs  # Other KF Ids that also see this map point

    def getNodes(self) -> list:
        """
        Returns list of nodes[node, pose[x,y,z,qx,qy,qz,qw]]
        :return: list of nodes
        """

        nodes = []
        for i in range(1, self.nKFs + 1):  # Not including KF 0
            i_dict = int(i)
            # TODO Handle culled KFs (that dict key no longer exist). KF culling disabled for now
            if i_dict in self.Tcw:
                t = self.Tcw[i_dict].translation()
                r = Rotation.from_dcm(self.Tcw[i_dict].rotationMatrix())
                pose = [t, r.as_quat()]  # x, y, z, w
                nodes.append(np.concatenate([np.array([i]), pose]).ravel())

        return nodes

    def getEdges(self) -> list:
        """
        Returns list of edges[node1, node2, FIM]
        :return: list of edges
        """

        return self.edgeList

    def getNodesEdges(self) -> Tuple[list, list]:
        """
        Returns nodes[node, pose[x,y,z,qx,qy,qz,qw]] & edges[node1, node2, FIM]
        :return: list of nodes, list of edges
        """

        return self.getNodes(), self.getEdges()

    def getMapPointsAsROSPointCloud2(self, global_frame: str = "map", mp_IDs: np.array = None) -> PointCloud2:
        """
        Saves the map points as sensor_msgs/PointCloud2. Debug only
        If no map points are given, all map is used
        :param global_frame: global frame name
        :param mp_IDs: map point IDs to be transformed
        :return: PointCloud2 of map points
        """

        points = []
        if mp_IDs is None:
            map_points = self.p3D.values()
            for i in map_points:
                points.append(i)
        else:
            for i in mp_IDs:
                points.append(self.p3D[i])

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  # PointField('rgb', 12, PointField.UINT32, 1),
                  # PointField('rgba', 12, PointField.UINT32, 1),
                  ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = global_frame
        map_PointCloud2 = point_cloud2.create_cloud(header, fields, points)

        return map_PointCloud2

    def getGraphAsMarkerArray(self, global_frame: str = "map", only_nodes: bool = False,
                              color: bool = False) -> MarkerArray:
        """
        Saves the graph as visualization_msgs/MarkerArray message for RViZ/ROS visualization
        :param color: change color to distinguish between real and hallucinated graphs
        :param only_nodes: only plot vertices or not
        :param global_frame: global frame name
        :return: Marker array of graph nodes and edges
        """
        if color:
            c1 = [1.0, 0.0, 0.0]
            c2 = [1.0, 0.49, 0.49]
            a = 1.0
            sc1 = 0.1
            sc2 = 0.04
        else:
            c1 = [1.0, 0.0, 0.0]
            c2 = [0.0, 0.0, 1.0]
            a = 1.0
            sc1 = 0.08
            sc2 = 0.01

        graph_marker = MarkerArray()
        graph_marker.markers.clear()
        id_markers = 1

        # Add vertices
        for i in range(0, self.nKFs + 1):
            i_dict = int(i)

            # TODO Handle culled KFs (that dict key no longer exist). KF culling disabled for now
            if i_dict in self.Tcw:
                vertex_marker = createMarker(mtype="sphere", frame=global_frame, ns="graph_ns", colors=c1,
                                             lifetime=0, alpha=a, scale=sc1)
                vertex_marker.id = id_markers
                t = self.Tcw[i_dict].translation()
                vertex_marker.pose.position.x = t[0]
                vertex_marker.pose.position.y = t[1]
                vertex_marker.pose.position.z = 0.1

                graph_marker.markers.append(vertex_marker)
                id_markers += 1

        # Add edges
        if not only_nodes:
            edges = self.getEdges()
            for edge in edges:  # parse edges file
                i = edge[0]
                j = edge[1]
                i_dict = int(i)
                j_dict = int(j)

                # TODO Handle culled KFs (that dict key no longer exist). KF culling disabled for now
                if i_dict in self.Tcw and j_dict in self.Tcw:
                    edge_marker = createMarker(mtype="lines", frame=global_frame, ns="graph_ns", colors=c2,
                                               lifetime=0, alpha=a, scale=sc2)
                    edge_marker.id = id_markers
                    p = Point()
                    # Edge's starting position
                    t1 = self.Tcw[i_dict].translation()
                    p.x = t1[0]
                    p.y = t1[1]
                    p.z = 0.1
                    edge_marker.points.append(p)
                    # Edge's ending position
                    p = Point()
                    t2 = self.Tcw[j_dict].translation()
                    p.x = t2[0]
                    p.y = t2[1]
                    p.z = 0.1
                    edge_marker.points.append(p)

                    graph_marker.markers.append(edge_marker)
                    id_markers += 1

        return graph_marker

    def hallucinateHessianFromPose(self, robot_pose: Pose, robot_pose_last: Pose, id_last: int, FIM_last: np.array,
                                   seen_cells_pct: float) -> dict:
        """
        Computes hallucinated hessian of a robot's pose using the points in the map, and the other vertices this one
        should be connected to (re localization)
            1st: compute Hessian relative to exploitation of the known map points
            2nd: set odometry Hessian
            3rd: compute Hessian relative to exploration
        :param FIM_last: last FIM of the graph (6x6 array)
        :param seen_cells_pct: percentage of new cells seen in that pose
        :param robot_pose: geometry_msgs/Pose
        :param robot_pose_last: geometry_msgs/Pose
        :param id_last: last id of the graph
        :return: numpy array of size 6x6, dict of vertices co-visible with the current one & no. of covisible points
        """

        MIN_TH_COV_POINTS = 150
        MAX_TH_COV_POINTS = 400

        # Get all map points in camera's frustum
        mpIDs_inside_frustum = self.frustumCulling(robot_pose)

        t_robot = [robot_pose.position.x, robot_pose.position.y, robot_pose.position.z]
        rot_robot = Rotation.from_quat([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z,
                                        robot_pose.orientation.w])
        T_map_base = sp.SE3(rot_robot.as_dcm(), t_robot)

        FIMS = {}
        exploitation_odom_H = {}
        exploitation_hessian = np.zeros([6, 6])

        # \\ 1 // Compute Hessian relative to exploiting the known map points
        # Compute re localization KFs and their probabilities of LC
        reloc_kfs = self.covisibleKeyFrames(mpIDs_inside_frustum, MIN_TH_COV_POINTS, MAX_TH_COV_POINTS)

        T_map_camera = self.T_camera_base.inverse() * T_map_base.inverse()

        for (k, v) in reloc_kfs.items():
            if int(k) in self.Tcw:
                p_temp = self.Tcw[int(k)].translation()
                r_temp = Rotation.from_dcm(self.Tcw[int(k)].rotationMatrix())
                q_temp = r_temp.as_quat()
                robot_pose2 = Pose()
                robot_pose2.position.x = p_temp[0]
                robot_pose2.position.y = p_temp[1]
                robot_pose2.position.z = p_temp[2]
                robot_pose2.orientation.x = q_temp[0]
                robot_pose2.orientation.y = q_temp[1]
                robot_pose2.orientation.z = q_temp[2]
                robot_pose2.orientation.w = q_temp[3]

                mpIDs_inside_frustum2 = self.frustumCulling(robot_pose2)
                mpIDs_covisible = list(set(mpIDs_inside_frustum) & set(mpIDs_inside_frustum2))
                # mpIDs_covisible = list(set(mpIDs_inside_frustum).intersection(mpIDs_inside_frustum2))

                n_cov_MPs = len(mpIDs_covisible)
                if n_cov_MPs > 0:
                    for point in mpIDs_covisible:
                        # Compute point coordinates in the camera reference
                        p3D = self.p3D[point]
                        p3Dc = T_map_camera * p3D

                        # Compute projection Jacobian
                        jac_projection = np.array([[self.fx / p3Dc[2], 0.0, -self.fx * p3Dc[0] / (p3Dc[2] * p3Dc[2])],
                                                   [0.0, self.fy / p3Dc[2], -self.fy * p3Dc[1] / (p3Dc[2] * p3Dc[2])]])

                        # Compute Jacobian w.r.t. the camera pose
                        jac_se3 = np.array([[1.0, 0.0, 0.0, 0.0, p3D[2], -p3D[1]],
                                            [0.0, 1.0, 0.0, -p3D[2], 0.0, p3D[0]],
                                            [0.0, 0.0, 1.0, p3D[1], -p3D[0], 0.0]])

                        # Full Jacobian matrix
                        jac = - np.matmul(jac_projection, jac_se3)

                        # Accumulate Hessian
                        exploitation_hessian += np.matmul(jac.transpose(), jac)

                # Add relocalization probability
                exploitation_odom_H[k] = exploitation_hessian * v

        # \\ 2 // Set odometry Hessian
        exploitation_odom_H[id_last] = FIM_last

        # \\ 3 // Compute Hessian of the all edges connected to current hallucinated vertex, as in Shannon-Renyi entropy
        sigma = seen_cells_pct  # Use the grid map to extract % of expected unknown seen cells
        if sigma > 0:
            alpha = 1 + (1 / sigma)
            for (k, v) in exploitation_odom_H.items():
                FIMS[k] = v - v * 1 / (1 - alpha)

        return FIMS

    def covisibleKeyFrames(self, points_ids: np.array, th_min: int, th_max: int) -> dict:
        """
        Given the set of map points visible from a certain KF (vertex), computes the number of map points that are also
        visible from other KFs (nodes). If the shared set of map points between the given vertex and any other one is
        less than th_min, there is no re localization probability. From th_max on, there is a re localization
        probability of 100 %, i.e., we add a edge between them.
        :param points_ids: map points to be considered
        :param th_min: minimum points threshold
        :param th_max: maximum points threshold
        :return: IDs of KFs that also see that points
        """

        cov_KFs = {}
        reloc_KF_IDs = {}
        for point in points_ids:
            kfs = self.mpObs[point]
            for kf in kfs:
                if kf in cov_KFs:
                    cov_KFs[kf] = cov_KFs[kf] + 1
                else:
                    cov_KFs[kf] = 1

        for (k, v) in cov_KFs.items():
            if v >= th_min:
                if v > th_max:
                    reloc_KF_IDs[k] = 1
                else:
                    reloc_KF_IDs[k] = v / th_max
                # prob = np.random.rand()
                # if prob <= v / th_max:
                #     reloc_KF_IDs[k] = v

        return reloc_KF_IDs

    def frustumCulling(self, robot_pose: Pose) -> np.array:
        """
        Computes the IDs of the map points inside robot camera frustum
        :param robot_pose: geometry_msgs/Pose
        :return: list of map points IDs
        """

        MAX_FRUSTUM_DISTANCE = 3.0  # meters

        t_robot = [robot_pose.position.x, robot_pose.position.y, robot_pose.position.z]
        rot_robot = Rotation.from_quat([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z,
                                        robot_pose.orientation.w])
        T_map_base = sp.SE3(rot_robot.as_dcm(), t_robot)

        T_map_cam = self.T_camera_base.inverse() * T_map_base.inverse()

        points_IDs_in_frustum = []
        for pointID, value in self.p3D.items():
            x = t_robot - value
            if x[0] > MAX_FRUSTUM_DISTANCE or x[1] > MAX_FRUSTUM_DISTANCE or x[2] > MAX_FRUSTUM_DISTANCE:
                continue

            # Transform points from map frame into the current camera frame
            p3Dc = T_map_cam * value

            if MAX_FRUSTUM_DISTANCE < p3Dc[2] or p3Dc[2] < 0:  # Check point inside range
                continue

            # Project point into the image
            u = self.fx * p3Dc[0] / p3Dc[2] + self.cx
            v = self.fy * p3Dc[1] / p3Dc[2] + self.cy

            if 0 <= u < self.imCols and 0 <= v < self.imRows:  # Check point inside image
                points_IDs_in_frustum.append(pointID)

        return points_IDs_in_frustum

    def getBestRelocPoses(self, robot_pose: Pose) -> np.array:
        """
        Computes a set of recovery goals to help the SLAM system relocalize
        :param robot_pose: geometry_msgs/Pose
        :return: list of poses in the global frame
        """

        t_robot = [robot_pose.position.x, robot_pose.position.y, robot_pose.position.z]
        rot_robot = Rotation.from_quat([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z,
                                        robot_pose.orientation.w])

        best_reloc_poses = []
        # First option, rotate 180 deg in place
        rot_180 = Rotation.from_euler('xyz', [0, 0, 180], degrees=True)
        r = rot_robot * rot_180
        q = r.as_quat()
        reloc_pose = Pose()
        reloc_pose.position = robot_pose.position
        reloc_pose.orientation.x = q[0]
        reloc_pose.orientation.y = q[1]
        reloc_pose.orientation.z = q[2]
        reloc_pose.orientation.w = q[3]
        best_reloc_poses.append(reloc_pose)

        # Second option, near place with high map point density
        nodes = self.getNodes()  # [node, pose[x,y,z,qx,qy,qz,qw]]
        nodes_selected = []
        for node in nodes:
            euclidean_dist = (t_robot[0] - node[1][0]) ** 2 + (t_robot[1] - node[1][1]) ** 2
            if 0.2 <= euclidean_dist <= 2.0:
                seenMPs = self.getMapPointsSeen(node[0])
                nodes_selected.append([seenMPs, node])

        if len(nodes_selected) > 0:
            nodes_selected.sort(reverse=True, key=lambda x: x[0])
            node = nodes_selected[0]
            pose = node[1][1]
            quat = node[1][2]
            reloc_pose = Pose()
            reloc_pose.position.x = pose[0]
            reloc_pose.position.y = pose[1]
            reloc_pose.orientation.x = quat[0]
            reloc_pose.orientation.y = quat[1]
            reloc_pose.orientation.z = quat[2]
            reloc_pose.orientation.w = quat[3]
            best_reloc_poses.append(reloc_pose)

        return best_reloc_poses

    def getMapPointsSeen(self, KF_Id: int) -> int:
        """
        Computes the number of visible map points from a keyframe
        :param KF_Id: int
        :return: integer number of points
        """

        n = 0
        for kfs in self.mpObs.items():
            if KF_Id in kfs[1]:
                n = n + 1
        return n
