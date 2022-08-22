#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# Robot class

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import tf
import actionlib
import numpy as np

from scipy.spatial.transform import Rotation
from typing import Tuple

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Pose, PoseStamped

from Functions import yawBtw2Points


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Class~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class Robot:
    def __init__(self, name: str):
        """
        Constructor
        """
        self.goal = MoveBaseGoal()
        self.start = PoseStamped()
        self.end = PoseStamped()
        self.pose = Pose()

        self.assigned_point = []
        self.name = name  # robot_1
        rospy.loginfo(rospy.get_name() + ': Robot Class started with robot name: ' + name)

        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_footprint')
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(self.global_frame, self.robot_frame, rospy.Time(0), rospy.Duration(5))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo(rospy.get_name() + ': Robot Class is waiting for the robot transform.')
                (trans, rot) = self.listener.lookupTransform(self.global_frame, self.robot_frame, rospy.Time(0))
                self.position = np.array([trans[0], trans[1]])
                self.rotation = np.array([rot[0], rot[1], rot[2], rot[3]])
                self.pose.position.x = trans[0]
                self.pose.position.y = trans[1]
                self.pose.position.z = 0
                self.pose.orientation.x = rot[0]
                self.pose.orientation.y = rot[1]
                self.pose.orientation.z = rot[2]
                self.pose.orientation.w = rot[3]
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr(tf.LookupException)
                cond = 0
        rospy.loginfo(rospy.get_name() + ': Robot Class received the robot transform.')

        self.assigned_point = self.position
        self.client = actionlib.SimpleActionClient(self.name + '/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal.target_pose.header.frame_id = self.global_frame
        self.goal.target_pose.header.stamp = rospy.Time.now()

        self.plan_service = rospy.get_param('~plan_service', '/move_base_node/NavfnROS/make_plan')
        rospy.wait_for_service(self.name + self.plan_service)
        self.make_plan = rospy.ServiceProxy(self.name + self.plan_service, GetPlan)

        self.start.header.frame_id = self.global_frame
        self.end.header.frame_id = self.global_frame
        rospy.loginfo(rospy.get_name() + ': Initialized robot.')

        rospy.loginfo(rospy.get_name() + ": Moving robot to +[0.1,0.1].")
        x = self.position[0] + 0.1
        y = self.position[1] + 0.1
        self.sendGoal([x, y])
        rospy.loginfo(rospy.get_name() + ": Moved robot to: [" + str(x) + ", " + str(y) + "].")

    def getPosition(self) -> np.array:
        """
        Gets robot's current position
        """
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(self.global_frame, self.robot_frame, rospy.Time(0))
                self.position = np.array([trans[0], trans[1]])
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0

        return self.position

    def getPose(self) -> Tuple[np.array, np.array]:
        """
        Gets robot's current pose as numpy arrays: position & quaternion
        """
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(self.global_frame, self.robot_frame, rospy.Time(0))
                self.position = np.array([trans[0], trans[1]])
                self.rotation = np.array([rot[0], rot[1], rot[2], rot[3]])
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr(tf.LookupException)
                cond = 0

        return self.position, self.rotation

    def getPoseAsGeometryMsg(self) -> Pose:
        """
        Gets robot's current pose as geometry_msgs/Pose
        """
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(self.global_frame, self.robot_frame, rospy.Time(0))
                self.pose.position.x = trans[0]
                self.pose.position.y = trans[1]
                self.pose.position.z = 0
                self.pose.orientation.x = rot[0]
                self.pose.orientation.y = rot[1]
                self.pose.orientation.z = rot[2]
                self.pose.orientation.w = rot[3]
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr(tf.LookupException)
                cond = 0

        return self.pose

    def sendGoal(self, point: np.array, waitGoalReached: bool = True):
        """
        Send goal and wait for task completion
        """
        self.goal.target_pose.pose.position.x = point[0]
        self.goal.target_pose.pose.position.y = point[1]
        current_pos = self.getPosition()
        goal_pos = [self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y]
        R_goal = Rotation.from_euler('xyz', [0., 0., yawBtw2Points(current_pos, goal_pos)], degrees=False)
        q_goal = R_goal.as_quat()
        self.goal.target_pose.pose.orientation.x = q_goal[0]
        self.goal.target_pose.pose.orientation.y = q_goal[1]
        self.goal.target_pose.pose.orientation.z = q_goal[2]
        self.goal.target_pose.pose.orientation.w = q_goal[3]

        if waitGoalReached:
            self.client.send_goal_and_wait(self.goal)
        else:
            self.client.send_goal(self.goal)
        self.assigned_point = np.array(point)

    def sendGoalAsPose(self, pose: Pose, waitGoalReached: bool = True):
        """
        Send goal and wait for task completion
        """
        self.goal.target_pose.pose = pose

        if waitGoalReached:
            self.client.send_goal_and_wait(self.goal)
        else:
            self.client.send_goal(self.goal)

        self.assigned_point = np.array([pose.position.x, pose.position.y])

    def cancelGoal(self):
        rospy.loginfo(rospy.get_name() + ': Cancelling goal requested.')
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()
        rospy.loginfo(rospy.get_name() + ': Goal cancelled.')

    def getState(self) -> int:
        """
        Returns status of goal
        http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        :rtype: actionlib_msgs/GoalStatus
        """
        return self.client.get_state()

    def makePlan(self, start: Pose, end: Pose) -> Tuple[PoseStamped]:
        """
        Returns poses in plan (nav_msgs/Path)
        :rtype: geometry_msgs/PoseStamped
        """
        self.start.header.seq += 1
        self.start.header.stamp = rospy.Time.now()
        self.start.pose.position.x = start.position.x
        self.start.pose.position.y = start.position.y
        self.start.pose.position.z = start.position.z
        self.start.pose.orientation.x = start.orientation.x
        self.start.pose.orientation.y = start.orientation.y
        self.start.pose.orientation.z = start.orientation.z
        self.start.pose.orientation.w = start.orientation.w

        self.end.header.seq += 1
        self.end.header.stamp = rospy.Time.now()
        self.end.pose.position.x = end.position.x
        self.end.pose.position.y = end.position.y
        self.end.pose.position.z = end.position.z
        self.end.pose.orientation.x = end.orientation.x
        self.end.pose.orientation.y = end.orientation.y
        self.end.pose.orientation.z = end.orientation.z
        self.end.pose.orientation.w = end.orientation.w

        start = self.listener.transformPose(self.global_frame, self.start)
        end = self.listener.transformPose(self.global_frame, self.end)
        plan = self.make_plan(start=start, goal=end, tolerance=0.1)

        return plan.plan.poses
