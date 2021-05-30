#!/usr/bin/env python3


import rospy
import moveit_commander
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from my_robot_world.srv import *
from math import pi
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from gazebo_ros_link_attacher.srv import *
import numpy as np


def pq_to_pose(p, q):
    pose = Pose()
    pose.position.x = p[0]
    pose.position.y = p[1]
    pose.position.z = p[2]
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose


def move_robot(pose):
    goal_pose = FollowJointTrajectoryActionGoal()
    goal_pose.goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    goal_pose.goal.trajectory.points.append(JointTrajectoryPoint())
    goal_pose.goal.trajectory.points[0].positions = pose
    goal_pose.goal.trajectory.points[0].velocities = [0, 0, 0, 0, 0, 0]
    goal_pose.goal.trajectory.points[0].time_from_start = rospy.Duration.from_sec(1.0)

    return goal_pose


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_robot")
    pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal,
                          queue_size=1000)
    rate = rospy.Rate(10.0)

    arm_move_group = moveit_commander.MoveGroupCommander("manipulator")
    robot_commander = moveit_commander.RobotCommander()

    compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
    rospy.wait_for_service("/compute_ik")
    spawn_client = rospy.ServiceProxy("object_detected_service", ObjectDetectMSG)
    attach_client = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_client = rospy.ServiceProxy("/link_attacher_node/detach", Attach)

    request = AttachRequest()

    wait_product_pos = np.radians([77, -51, 74, 57, 90, 0])
    pallet_pos = np.radians([-77, -39, 74, 57, 90, 0])
    take_product = np.radians([77, -39, 74, 57, 90, 0])

    while not rospy.is_shutdown():

        goal_pose = move_robot(wait_product_pos)
        rospy.sleep(1)
        sensor = spawn_client.call(1)

        if sensor.object_detected:
            rospy.sleep(0.5)
            goal_pose = move_robot(take_product)
            pub.publish(goal_pose)
            rospy.sleep(1.5)
            request.model_name_1 = sensor.model_name
            request.link_name_1 = "base_link"
            request.model_name_2 = "robot"
            request.link_name_2 = "wrist_3_link"
            attach_client.call(request)
            rospy.sleep(0.5)
            goal_pose = move_robot(wait_product_pos)
            pub.publish(goal_pose)
            rospy.sleep(1.5)
            goal_pose = move_robot(pallet_pos)
            pub.publish(goal_pose)
            rospy.sleep(1.5)
            detach_client.call(request)

        pub.publish(goal_pose)
        rate.sleep()


main()
