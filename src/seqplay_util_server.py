#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy

try:
    import roslib
    import nextage_ros_bridge
except:
    import roslib
    roslib.load_manifest('nextage_ros_bridge')
    import nextage_ros_bridge
import rospy
from tf.transformations import euler_from_matrix
from hrpsys_ros_bridge.srv import *
from nextage_ros_bridge import nextage_client

from nextage_ros_seqplay_util.srv import *


class NextageSeqPlayUtil(object):
    def __init__(self):
        self.init_pose = nextage_client.NextageClient.InitialPose
        self.off_pose = nextage_client.NextageClient.OffPose

        # degrees to radians
        for angles in self.init_pose:
            for i, a in enumerate(angles):
                angles[i] = math.radians(a)
        for angles in self.off_pose:
            for i, a in enumerate(angles):
                angles[i] = math.radians(a)

        rospy.wait_for_service('/SequencePlayerServiceROSBridge/setTargetPose')
        self.set_target_pose = rospy.ServiceProxy(
            '/SequencePlayerServiceROSBridge/setTargetPose',
            OpenHRP_SequencePlayerService_setTargetPose)
        self.set_joint_angles_of_group = rospy.ServiceProxy(
            '/SequencePlayerServiceROSBridge/setJointAnglesOfGroup',
            OpenHRP_SequencePlayerService_setJointAnglesOfGroup)

        rospy.wait_for_service('/ForwardKinematicsServiceROSBridge/getCurrentPose')
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
    util = NextageSeqPlayUtil()

    def set_target_pose_relative(request):
        return OpenHRP_SequencePlayerService_setTargetPoseResponse(
            util.set_target_pose_relative(
                request.name, request.xyz, request.rpy, request.tm))

    def go_initial(request):
        return goPoseResponse(util.go_initial(request.tm))

    def go_off_pose(request):
        return goPoseResponse(util.go_off_pose(request.tm))

    rospy.init_node('nextage_ros_seqplay_util')
    # regsiter ros services
    print rospy.Service('nextage_ros_seqplay_util/setTargetPoseRelative',
                        OpenHRP_SequencePlayerService_setTargetPose,
                        set_target_pose_relative)
    print rospy.Service('nextage_ros_seqplay_util/goInitial',
                        goPose, go_initial)
    print rospy.Service('nextage_ros_seqplay_util/goOffPose',
                        goPose, go_off_pose)
    rospy.spin()
