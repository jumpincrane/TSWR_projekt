import roslib
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from transforms3d import quaternions
import tf
import tf.transformations as tr
import numpy as np

def pose_to_matrix(pose):
    """geometry_msgs.msg.Pose to 4x4 matrix"""
    transformer = tf.TransformerROS(True, rospy.Duration(1.0))
    p = [pose.position.x, pose.position.y, pose.position.z]
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    norm = np.linalg.norm(q)
    if np.abs(norm - 1.0) > 1e-3:
        raise ValueError(
            "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                str(q), np.linalg.norm(q)))
    elif np.abs(norm - 1.0) > 1e-6:
        q = q / norm
    g = transformer.fromTranslationRotation(p, q)
    return g


def pq_to_pose(p,q):
    pose = Pose()
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.position.z = p[2]
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def matrix_to_pose(mat):
    pose = Pose()
    quat = quaternions.mat2quat(mat[0:3, 0:3])
    quat_prim = [quat[1], quat[2], quat[3], quat[0]]
    pose = pq_to_pose([mat[0,3], mat[1,3], mat[2,3]], quat_prim)
    return pose