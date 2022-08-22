#!/usr/bin/env python3

# jplaced
# 2022, Universidad de Zaragoza

# Robot class

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Include modules~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
import rospy
import tf
import numpy as np

from typing import Tuple

from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Pose, PoseStamped


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Class~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
class Robot:
    def __init__(self, name: str):
        """
        Constructor
        """
        self.start = PoseStamped()
        self.end = PoseStamped()
        self.pose = Pose()

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

        self.plan_service = rospy.get_param('~plan_service', '/move_base_node/NavfnROS/make_plan')
        print(self.name + self.plan_service)
        rospy.wait_for_service(self.name + self.plan_service)
        self.make_plan = rospy.ServiceProxy(self.name + self.plan_service, GetPlan)

        self.start.header.frame_id = self.global_frame
        self.end.header.frame_id = self.global_frame
        rospy.loginfo(rospy.get_name() + ': Initialized robot.')

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
