#!/usr/bin/python3

# jplaced
# 2022, Universidad de Zaragoza

# Weighted pose-graph class


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import sys
import warnings
import rospy
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

from operator import itemgetter
from nptyping import ndarray
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import MarkerArray

from Map import Map
from Robot import Robot
from Functions import createMarker, yawBtw2Points

debug = False

PYTHON_VERSION_ = sys.version_info[0]
if PYTHON_VERSION_ < 3:
    warnings.warn("Careful, using python version 2")


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Main Class~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class WeightedPoseGraph:
    def __init__(self, nodes: list = None, edges: list = None, criteria: str = 'd_opt'):
        """
        Constructor
        Nodes: nodeId, SE3 pose as translation and quaternion rotation
        Edges: nodeId1, nodeId2, edge_type, Information matrix, graph weight (d-opt)
        edge_type is 1 if Loop closure or 0 otherwise
        """

        self.th_plan_points = rospy.get_param('~hallucinated_plan_th', 50)

        self.graph = nx.Graph()
        self.criteria = criteria
        if (nodes is not None) and (edges is not None):
            for i in range(0, np.size(nodes, 0)):
                p = nodes[i][1]  # array of pose
                q = nodes[i][2]  # array of quaternion
                self.graph.add_node(nodes[i][0], translation=p, orientation=q)  # qx, qy, qz, qw
            for i in range(0, np.size(edges, 0)):
                edge = (edges[i][0], edges[i][1])
                edge_type = 0 if abs(edges[i][0] - edges[i][1]) == 1 else 1
                FIM = edges[i][2]
                A = FIM
                A = np.nan_to_num(A, nan=np.nanmax(A), posinf=np.nanmax(A), neginf=-np.nanmax(A))
                eigv2, _ = np.linalg.eig(A)
                eigv = eigv2[eigv2 > 1e-8]
                n = np.size(A, 1)
                if criteria == 'd_opt':
                    opt_cri = np.exp(np.sum(np.log(eigv)) / n)
                    self.graph.add_edge(*edge, type=edge_type, information=FIM, weight=opt_cri)
                else:
                    print("WeightedPoseGraph Class: Optimality criteria not recognized.")
        elif nodes is not None:
            print("WeightedPoseGraph Class: Edges initialized to None.")
            for i in range(0, np.size(nodes, 0)):
                p = nodes[i][1]
                q = nodes[i][2]
                self.graph.add_node(nodes[i][0], translation=p, orientation=q)  # qx, qy, qz, qw
        elif edges is not None:
            print("WeightedPoseGraph Class: Nodes initialized to None.")
            for i in range(0, np.size(edges, 0)):
                edge = (edges[i][0], edges[i][1])
                edge_type = 0 if abs(edges[i][0] - edges[i][1]) == 1 else 1
                FIM = edges[i][2:]
                A = [[FIM[0], FIM[1], FIM[2], FIM[3], FIM[4], FIM[5]],
                     [FIM[6], FIM[7], FIM[8], FIM[9], FIM[10], FIM[11]],
                     [FIM[12], FIM[13], FIM[14], FIM[15], FIM[16], FIM[17]],
                     [FIM[18], FIM[19], FIM[20], FIM[21], FIM[22], FIM[23]],
                     [FIM[24], FIM[25], FIM[26], FIM[27], FIM[28], FIM[29]],
                     [FIM[30], FIM[31], FIM[32], FIM[33], FIM[34], FIM[35]]]
                A = np.nan_to_num(A, nan=np.nanmax(A), posinf=np.nanmax(A), neginf=-np.nanmax(A))
                eigv2 = np.linalg.eigvals(A)
                eigv = eigv2[eigv2 > 1e-8]
                n = np.size(A, 1)
                if criteria == 'd_opt':
                    opt_cri = np.exp(np.sum(np.log(eigv)) / n)
                    self.graph.add_edge(*edge, type=edge_type, information=FIM, weight=opt_cri)
                else:
                    print("WeightedPoseGraph Class: Optimality criteria not recognized.")

    def addEdge(self, id1: int, id2: int, FIM: ndarray):
        edge_type = 0 if abs(id1 - id2) == 1 else 1
        A = [[FIM[0], FIM[1], FIM[2], FIM[3], FIM[4], FIM[5]],
             [FIM[6], FIM[7], FIM[8], FIM[9], FIM[10], FIM[11]],
             [FIM[12], FIM[13], FIM[14], FIM[15], FIM[16], FIM[17]],
             [FIM[18], FIM[19], FIM[20], FIM[21], FIM[22], FIM[23]],
             [FIM[24], FIM[25], FIM[26], FIM[27], FIM[28], FIM[29]],
             [FIM[30], FIM[31], FIM[32], FIM[33], FIM[34], FIM[35]]]
        A = np.nan_to_num(A, nan=np.nanmax(A), posinf=np.nanmax(A), neginf=-np.nanmax(A))
        eigv2 = np.linalg.eigvals(A)
        eigv = eigv2[eigv2 > 1e-8]
        n = np.size(A, 1)
        if self.criteria == 'd_opt':
            opt_cri = np.exp(np.sum(np.log(eigv)) / n)
            self.graph.add_edge(id1, id2, type=edge_type, information=FIM, weight=opt_cri)
        else:
            print("WeightedPoseGraph Class: Optimality criteria not recognized.")

    def copyGraph(self, frozenGraph: nx.Graph):
        """
        Creates a graph from an existing one (which may be frozen)
        """
        self.graph.clear()
        self.graph = nx.Graph(frozenGraph)

    def computeAdjacency(self):
        """
        Computes the adjacency matrix of the graph (scipy sparse matrices)
        :rtype: scipy sparse matrix csr
        """
        return nx.adjacency_matrix(self.graph, nodelist=None, weight='weight')

    def computeL(self):
        """
        Computes the Laplacian matrix of the graph
        Laplacian = Degree - Adjacency
        :rtype: scipy sparse matrix csr
        """
        return nx.laplacian_matrix(self.graph, weight='weight')

    def computeAnchoredL(self):
        """
        Computes the anchored Laplacian matrix along a random node
        :rtype: scipy sparse matrix csr
        """
        L = nx.laplacian_matrix(self.graph, weight='weight')
        idx_to_drop = np.random.randint(0, np.shape(L)[1], 1)
        if debug:
            print("WeightedPoseGraph Class: Laplacian anchored through index" + format(idx_to_drop))
        C = L.tocoo()
        keep = ~np.in1d(C.col, idx_to_drop)
        C.data, C.row, C.col = C.data[keep], C.row[keep], C.col[keep]
        C.col -= idx_to_drop.searchsorted(C.col)  # decrement column indices
        C._shape = (C.shape[0], C.shape[1] - len(idx_to_drop))

        keep = ~np.in1d(C.row, idx_to_drop)
        C.data, C.row, C.col = C.data[keep], C.row[keep], C.col[keep]
        C.row -= idx_to_drop.searchsorted(C.row)  # decrement column indices
        C._shape = (C.shape[0] - len(idx_to_drop), C.shape[1])

        return C.tocsr()

    def computeSpectrum(self) -> ndarray:
        """
        Computes all eigenvalues of the Laplacian
        """
        return nx.laplacian_spectrum(self.graph, weight='weight')

    def computeRedSpectrum(self) -> ndarray:
        """
        Computes the non-zero eigenvalues of the Laplacian
        """
        eigen_v = nx.laplacian_spectrum(self.graph, weight='weight')
        eigen_v_reduced = eigen_v[1:]

        return eigen_v_reduced

    def computeMaxEig(self) -> float:
        return np.max(nx.laplacian_spectrum(self.graph, weight='weight'))

    def computeAlgCon(self) -> float:
        """
        Computes the algebraic connectivity of the graph
        """
        return nx.algebraic_connectivity(self.graph, weight='weight', normalized=False, tol=1e-08, method='tracemin')

    def computeFiedler(self) -> ndarray:
        """
        Computes the Fiedler vector of the graph
        """
        return nx.fiedler_vector(self.graph, weight='weight', normalized=False, tol=1e-08, method='tracemin')

    def getLcEdges(self) -> list:
        """
        Gets the edges that represent a loop closure (type 1 edges)
        Returns an ordered list of loop closure edges (id1, id2) by its order of
        appearance, that is, sorted by id2
        """
        edges = self.graph.edges.data('type')
        edges_LC = []
        for (u, v, wt) in edges:
            edges_LC.append([u, v]) if wt == 1 else True

        return sorted(edges_LC, key=itemgetter(1))

    def getLcNodeList(self) -> list:
        """
        Gets the node number list of LC
        """
        edges = self.graph.edges.data('type')
        idx_LC = []
        for (u, v, wt) in edges:
            if wt == 1:
                idx_LC.append([u, v])
        idx_LC.sort(key=lambda x: x[1])

        return idx_LC

    def getLcNodes(self) -> list:
        """
        Gets the nodes where there is a loop closure
        """
        nodes = self.graph.nodes.data('trans')
        edges = self.graph.edges.data('type')
        idx_LC = []
        for (u, v, wt) in edges:
            if wt == 1:
                idx_LC.append(u) if u not in idx_LC else True
                idx_LC.append(v) if v not in idx_LC else True
        nodes_LC = [ith_node if ith_idx == ith_node[0] else 0 for ith_idx in idx_LC for ith_node in nodes]
        nodes_LC = list(filter(lambda num: num != 0, nodes_LC))

        return nodes_LC

    def getMaxId(self) -> int:
        """
        Gets the max node id
        """
        nodes = self.graph.nodes.data('trans')
        max_id = 0
        for (u, p) in nodes:
            if u > max_id:
                max_id = u

        return max_id

    def getNEdges(self) -> int:
        """
        Gets the number of edges of the graph
        """
        return nx.number_of_edges(self.graph)

    def getNNodes(self) -> int:
        """
        Gets the number of edges of the graph
        """
        return nx.number_of_nodes(self.graph)

    def plotGraph(self, label: str = 'Data', color: str = 'Blue', drawLcSimple: bool = False,
                  drawLcComplex: bool = False):
        """
        Draws the pose graph (i.e. trajectory)
        - none, only plots the XY trajectory
        :param color: plot color
        :param label: plot label
        :param drawLcComplex: also plots LC connections
        :param drawLcSimple: also plots the starting point and the LC locations
        """
        if self.graph is not None:
            nodes = self.graph.nodes.data('trans')
            poses_x = [el[1][0] for el in nodes]
            poses_y = [el[1][1] for el in nodes]
            plt.plot(poses_x, poses_y, '-', label=label, alpha=1, color=color)
            plt.suptitle('Trajectory')

            if drawLcSimple and not drawLcComplex:
                # poses_LC = list(zip(*self.getLcNodes()))[1] if (PYTHON_VERSION_ >= (3, 0)) else \
                #     zip(*self.getLcNodes())[1]
                poses_LC = list(zip(*self.getLcNodes()))[1]
                poses_LC_x = [el[0] for el in poses_LC]
                poses_LC_y = [el[1] for el in poses_LC]
                plt.plot(poses_LC_x, poses_LC_y, '.', label='LC', color='black', alpha=1, markersize=2)
                plt.plot(poses_x[0], poses_y[0], '*', label='Start', color='black', alpha=1, markersize=10)

            elif drawLcComplex:
                plt.plot(poses_x[0], poses_y[0], '*', label='Start', color='black', alpha=1, markersize=10)
                label_added = False
                edges_LC = self.getLcEdges()
                for (u, v) in edges_LC:
                    poses_LC_x = []
                    poses_LC_y = []
                    poses_LC_x.append(self.graph.nodes[u]['trans'][0])
                    poses_LC_x.append(self.graph.nodes[v]['trans'][0])
                    poses_LC_y.append(self.graph.nodes[u]['trans'][1])
                    poses_LC_y.append(self.graph.nodes[v]['trans'][1])
                    if not label_added:
                        plt.plot(poses_LC_x, poses_LC_y, linestyle='-', color='orange', label='LC', alpha=1, marker='o',
                                 markeredgecolor='b', markersize=2)
                        label_added = True
                    else:
                        plt.plot(poses_LC_x, poses_LC_y, linestyle='-', color='orange', alpha=1, marker='o',
                                 markeredgecolor='b', markersize=2)

    def getGraphAsMarkerArray(self, global_frame: str = "map", color: bool = False) -> MarkerArray:
        """
        Saves the graph as visualization_msgs/MarkerArray message for RViZ/ROS visualization
        :param global_frame: global frame name
        :param color: change color to distinguish between real and hallucinated graphs
        :return: Marker array of graph nodes and edges
        """
        if color:
            c1 = [0.0, 0.0, 1.0]
            c2 = [0.49, 0.49, 1.0]
            a = 1
            lt = 5
            sc1 = 0.1
            sc2 = 0.04
        else:
            c1 = [1.0, 0.0, 0.0]
            c2 = [0.0, 0.0, 1.0]
            a = 1.0
            lt = 1
            sc1 = 0.08
            sc2 = 0.01

        graph_marker = MarkerArray()
        graph_marker.markers.clear()
        id_markers = 1

        n = self.getNNodes()
        all_t = nx.get_node_attributes(self.graph, 'translation')
        # Add vertices
        for i in range(1, n+1):
            vertex_marker = createMarker(mtype="sphere", frame=global_frame, ns="graph_ns", colors=c1, lifetime=lt,
                                         alpha=a, scale=sc1)
            vertex_marker.id = id_markers

            if i in all_t:
                t = all_t[i]
                vertex_marker.pose.position.x = t[0]
                vertex_marker.pose.position.y = t[1]
                vertex_marker.pose.position.z = t[2]

                graph_marker.markers.append(vertex_marker)
                id_markers += 1

        # Add edges
        edge_marker = createMarker(mtype="lines", frame=global_frame, ns="graph_ns", colors=c2, lifetime=lt, alpha=a,
                                   scale=sc2)
        for (u, v, wt) in self.graph.edges.data('type'):
            u_int = int(u)
            v_int = int(v)
            if u_int != 0 and v_int != 0 and u_int in all_t and v_int in all_t:
                p = Point()
                # Edge's starting position
                t1 = all_t[u_int]
                p.x = t1[0]
                p.y = t1[1]
                p.z = t1[2]
                edge_marker.points.append(p)
                # Edge's ending position
                p = Point()
                t2 = all_t[v_int]
                p.x = t2[0]
                p.y = t2[1]
                p.z = t2[2]
                edge_marker.points.append(p)

        edge_marker.id = id_markers
        graph_marker.markers.append(edge_marker)

        return graph_marker

    def hallucinateGraph(self, robot: Robot, orb_map: Map, seen_cells_pct: float, xy_frontier: ndarray,
                         return_path: bool = False):
        """
        Hallucinates graph towards a frontier
        :rtype: MarkerArray, WeightedPoseGraph
        """

        # Initialize hallucinated graph
        G_frontier = WeightedPoseGraph()
        G_frontier.copyGraph(self.graph)

        hallucinated_graph_marker = MarkerArray()

        # Get robot's current pose
        pose_robot = robot.getPoseAsGeometryMsg()
        xy_robot = robot.getPosition()

        ################################################################################################################
        # First, we hallucinate the path to the frontier
        ################################################################################################################
        pose_frontier = Pose()
        pose_frontier.position.x = xy_frontier[0]  # == plan[n_points-1].pose.position.x
        pose_frontier.position.y = xy_frontier[1]  # == plan[n_points-1].pose.position.y
        pose_frontier.position.z = 0
        # Add orientation at frontier's location
        R_frontier = Rotation.from_euler('xyz', [0., 0., yawBtw2Points(xy_robot, xy_frontier)], degrees=False)
        q_frontier = R_frontier.as_quat()
        pose_frontier.orientation.x = q_frontier[0]
        pose_frontier.orientation.y = q_frontier[1]
        pose_frontier.orientation.z = q_frontier[2]
        pose_frontier.orientation.w = q_frontier[3]

        # Make plan to frontier
        plan = robot.makePlan(pose_robot, pose_frontier)
        n_points = int(len(plan))

        if n_points > 0:
            ############################################################################################################
            # Second, we select certain points along the path
            ############################################################################################################
            plan_nodes = np.ceil(n_points / self.th_plan_points)
            new_nodes = np.sort(np.random.choice(np.arange(1, n_points - 2), int(plan_nodes), replace=False))
            new_nodes = np.append(new_nodes, n_points-1)  # Add one last node at frontier's location
            rospy.loginfo(rospy.get_name() + ": Plan length " + format(n_points+1) + " generated " + format(plan_nodes)
                          + " new nodes with path ids: " + format(new_nodes.ravel()))

            ############################################################################################################
            # Third, we hallucinate the graph with vertices at location of those points
            ############################################################################################################
            # Add new vertices to the graph

            last_known_edge = list(nx.edges(G_frontier.graph))[-1]
            id_last = id_new = last_known_edge[1]
            FIM_last = G_frontier.graph[last_known_edge[0]][id_last]["information"]

            for i in new_nodes:
                # Add orientation at path's intermediate points
                xy_prev_point = [plan[i-1].pose.position.x, plan[i-1].pose.position.y]
                t_new_point = [plan[i].pose.position.x, plan[i].pose.position.y, plan[i].pose.position.z]
                R_new_point = Rotation.from_euler('xyz', [0., 0., yawBtw2Points(xy_prev_point, t_new_point[0:2])],
                                                  degrees=False)
                q_new_point = R_new_point.as_quat()
                plan[i].pose.orientation.x = q_new_point[0]
                plan[i].pose.orientation.y = q_new_point[1]
                plan[i].pose.orientation.z = q_new_point[2]
                plan[i].pose.orientation.w = q_new_point[3]
                q_path_point = [plan[i].pose.orientation.x, plan[i].pose.orientation.y, plan[i].pose.orientation.z,
                                plan[i].pose.orientation.w]

                id_new += 1
                G_frontier.graph.add_node(id_new, translation=t_new_point, orientation=q_path_point)

                ########################################################################################################
                # Fourth, we compute FIMs and create odometry and re localization edges, if any
                ########################################################################################################
                edges_with_FIMs = orb_map.hallucinateHessianFromPose(plan[i].pose, plan[i-1].pose, id_last, FIM_last,
                                                                     seen_cells_pct)

                # Add re localization/odometry edges
                for (k, v) in edges_with_FIMs.items():
                    G_frontier.addEdge(k, id_new, v.flatten())

                id_last = id_new

            # Save points along path as MarkerArray for visualization purposes
            if return_path:
                hallucinated_graph_marker.markers.clear()
                id_markers = 0
                for i in new_nodes:
                    marker = createMarker(mtype="arrow", frame="map", colors=[0.0, 0.0, 1.0], lifetime=4, alpha=0.9,
                                          scale=0.5)
                    marker.id = id_markers
                    marker.pose.position.x = plan[i].pose.position.x
                    marker.pose.position.y = plan[i].pose.position.y
                    marker.pose.orientation.x = plan[i].pose.orientation.x
                    marker.pose.orientation.y = plan[i].pose.orientation.y
                    marker.pose.orientation.z = plan[i].pose.orientation.z
                    marker.pose.orientation.w = plan[i].pose.orientation.w

                    hallucinated_graph_marker.markers.append(marker)
                    id_markers += 1

        else:  # Void returns if no points in path, most times due to frontiers lying in high potential area of cost map
            rospy.logerr(rospy.get_name() + ": No points in plan to frontier at " + format(xy_frontier) +
                         ". Probably a high potential area!!")

        if return_path:
            return hallucinated_graph_marker, G_frontier
        else:
            return G_frontier
