#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy

import roslib
roslib.load_manifest('nextage_seqplay_util')
import rospy
from tf.transformations import euler_from_matrix
from hrpsys_ros_bridge.srv import *
from nextage_ros_bridge import nextage_client

from nextage_seqplay_util.srv import *


class NextageSeqPlayUtil(object):
    def __init__(self):
        self.init_pose = nextage_client.NextageClient._InitialPose
        self.off_pose = nextage_client.NextageClient.OffPose

        # degrees to radians
        for angles in self.init_pose:
            for i, a in enumerate(angles):
                angles[i] = math.radians(a)
        for angles in self.off_pose:
            for i, a in enumerate(angles):
                angles[i] = math.radians(a)
        rospy.loginfo('start setup services')
        rospy.wait_for_service('/SequencePlayerServiceROSBridge/setTargetPose')
        rospy.loginfo('/SequencePlayerServiceROSBridge/setTargetPose is ready')
        self.set_target_pose = rospy.ServiceProxy(
            '/SequencePlayerServiceROSBridge/setTargetPose',
            OpenHRP_SequencePlayerService_setTargetPose)
        self.set_joint_angles_of_group = rospy.ServiceProxy(
            '/SequencePlayerServiceROSBridge/setJointAnglesOfGroup',
            OpenHRP_SequencePlayerService_setJointAnglesOfGroup)

        rospy.wait_for_service('/ForwardKinematicsServiceROSBridge/getCurrentPose')
        rospy.loginfo('/SequencePlayerServiceROSBridge/getCurrentPose is ready')
        self.get_current_pose = rospy.ServiceProxy(
            '/ForwardKinematicsServiceROSBridge/getCurrentPose',
            OpenHRP_ForwardKinematicsService_getCurrentPose)

    def set_target_pose_relative(self, name, delta_xyz, delta_rpy, tm):
        if name.lower() == 'rarm':
            joint = 'RARM_JOINT5'
        elif name.lower() == 'larm':
            joint = 'LARM_JOINT5'
        else:
            raise rospy.ServiceException()

        matrix = self.get_current_pose(joint).pose.data
        xyz = numpy.array([matrix[3], matrix[7], matrix[11]])
        rpy = numpy.array(euler_from_matrix([matrix[0:3], matrix[4:7], matrix[8:11]], 'sxyz'))
        xyz += [delta_xyz[0], delta_xyz[1], delta_xyz[2]]
        rpy += [delta_rpy[0], delta_rpy[1], delta_rpy[2]]
        return self.set_target_pose(name, list(xyz), list(rpy), tm)

    def go_pose(self, pose, tm):
        self.set_joint_angles_of_group('torso', pose[0], tm)
        self.set_joint_angles_of_group('head', pose[1], tm)
        self.set_joint_angles_of_group('rarm', pose[2], tm)
        return self.set_joint_angles_of_group('larm', pose[3], tm)

    def go_initial(self, tm):
        return self.go_pose(self.init_pose, tm)

    def go_off_pose(self, tm):
        return self.go_pose(self.off_pose, tm)


if __name__ == '__main__':
    rospy.init_node('nextage_seqplay_util')

    util = NextageSeqPlayUtil()

    def set_target_pose_relative(request):
        return OpenHRP_SequencePlayerService_setTargetPoseResponse(
            util.set_target_pose_relative(
                request.name, request.xyz, request.rpy, request.tm).operation_return)

    def go_initial(request):
        return goPoseResponse(util.go_initial(request.tm).operation_return)

    def go_off_pose(request):
        return goPoseResponse(util.go_off_pose(request.tm).operation_return)

    # regsiter ros services
    set_target_pose_relative_service = rospy.Service('nextage_seqplay_util/setTargetPoseRelative',
                        OpenHRP_SequencePlayerService_setTargetPose,
                        set_target_pose_relative)
    go_initial_service = rospy.Service('nextage_seqplay_util/goInitial',
                        goPose, go_initial)
    go_off_pose = rospy.Service('nextage_seqplay_util/goOffPose',
                        goPose, go_off_pose)
    rospy.loginfo('nextage_seqplay_util is ready')
    rospy.spin()
