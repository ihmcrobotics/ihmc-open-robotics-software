#!/usr/bin/env python3
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, PoseStamped, PointStamped, TransformStamped
from std_msgs.msg import Header
import numpy
# import rospy
from scipy.spatial.transform import Rotation as Rot
import math

_EPS = numpy.finfo(float).eps * 4.0

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    q = numpy.array(quaternion[:4], dtype=numpy.float64, copy=True)
    nq = numpy.dot(q, q)
    if nq < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = numpy.outer(q, q)
    return numpy.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=numpy.float64)

def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix.

    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True

    """
    q = numpy.empty((4, ), dtype=numpy.float64)
    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
    t = numpy.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q

def PoseStamped_2_mat(p):
    q = p.pose.orientation
    pos = p.pose.position
    T = quaternion_matrix([q.x,q.y,q.z,q.w])
    T[:3,3] = numpy.array([pos.x,pos.y,pos.z])
    return T

def Pose_2_mat(p):
    q = p.orientation
    pos = p.position
    T = quaternion_matrix([q.x,q.y,q.z,q.w])
    T[:3,3] = numpy.array([pos.x,pos.y,pos.z])
    return T

def Mat_2_posestamped(m,f_id="test"):
    q = quaternion_from_matrix(m)
    p = PoseStamped(header = Header(frame_id=f_id), #robot.get_planning_frame()
                    pose=Pose(position=Point(*m[:3,3]), 
                    orientation=Quaternion(*q)))
    return p

def T_inv(T_in):
    R_in = T_in[:3,:3]
    t_in = T_in[:3,[-1]]
    R_out = R_in.T
    t_out = -numpy.matmul(R_out,t_in)
    return numpy.vstack((numpy.hstack((R_out,t_out)),numpy.array([0, 0, 0, 1])))

def transformation(pose):
    p_OwrtC=Pose() 
    p_OwrtC.position.x=float(pose[0])
    p_OwrtC.position.y=float(pose[1])
    p_OwrtC.position.z=float(pose[2])
    p_OwrtC.orientation.w=float(pose[6])
    p_OwrtC.orientation.x=float(pose[3])
    p_OwrtC.orientation.y=float(pose[4])
    p_OwrtC.orientation.z=float(pose[5])
    Tco = Pose_2_mat(p_OwrtC)

    p_CwrtB=Pose() 
    p_CwrtB.position.x=-0.143361
    p_CwrtB.position.y=-1.45842
    p_CwrtB.position.z=0.375607
    p_CwrtB.orientation.w=0.575573
    p_CwrtB.orientation.x=-0.817741
    p_CwrtB.orientation.y=-0.00388839
    p_CwrtB.orientation.z=-0.000290818
    Tbc = Pose_2_mat(p_CwrtB)

    Tbo = numpy.matmul(Tbc, Tco)

    rot_mat1 = Rot.from_matrix(Tbo[:3,:3])
    quat1 = rot_mat1.as_quat()
    rot_vec1 = rot_mat1.as_rotvec()
    position1 = Tbo[:3,3]

    pose= numpy.concatenate((position1, quat1))
    object_pose_base = [pose[0], pose[1], pose[2], pose[6], pose[3], pose[4], pose[5]] #wxyz
    return object_pose_base
