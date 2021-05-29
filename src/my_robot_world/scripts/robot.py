#!/usr/bin/env python3


import rospy
import moveit_commander
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from my_robot_world.srv import *
from math import pi


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_robot")
    pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal,
                          queue_size=1000)
    rate = rospy.Rate(10.0)
    spawn_client = rospy.ServiceProxy("object_detected_service", ObjectDetectMSG)

    product_pos = [pi/2, -pi/4, pi/2, pi/4, pi/2, 0.0]
    pallet_pos = [-pi/2, -pi/4, pi/2, pi/4, pi/2, 0.0]
    home_pos = [0, -pi/4, pi/2, pi/4, pi/2, 0.0]
    while not rospy.is_shutdown():
        goal_pose = FollowJointTrajectoryActionGoal()
        goal_pose.goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        goal_pose.goal.trajectory.points.append(JointTrajectoryPoint())
        goal_pose.goal.trajectory.points[0].positions = home_pos
        goal_pose.goal.trajectory.points[0].velocities = [0, 0, 0, 0, 0, 0]
        goal_pose.goal.trajectory.points[0].time_from_start = rospy.Duration.from_sec(1.0)
        rospy.sleep(1)
        sensor = spawn_client.call(1)

        if sensor.object_detected:
            goal_pose.goal.trajectory.points[0].positions = product_pos

        pub.publish(goal_pose)
        rate.sleep()


main()
