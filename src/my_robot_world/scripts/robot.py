#!/usr/bin/env python3


import rospy
import moveit_commander
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK
from gazebo_msgs.srv import SpawnModel, ApplyBodyWrench, SpawnModelRequest, ApplyBodyWrenchRequest
from my_robot_world.srv import *
from math import pi
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from gazebo_ros_link_attacher.srv import *
import numpy as np
from my_robot_world.srv import cmd, cmdRequest

command = 0

def handle_cmd(req):
    global command
    command = req.cmd



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


def spawn_pallet(pallet_xml, model_name):
    spawn_pallet_client = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    spawn_model_msg = SpawnModelRequest()
    spawn_model_msg.model_xml = pallet_xml
    spawn_model_msg.initial_pose.position.x = 0.0
    spawn_model_msg.initial_pose.position.y = -0.843470
    spawn_model_msg.initial_pose.position.z = 0.80
    spawn_model_msg.initial_pose.orientation.x = 0.0
    spawn_model_msg.initial_pose.orientation.y = 0.0
    spawn_model_msg.initial_pose.orientation.z = 0.0
    spawn_model_msg.initial_pose.orientation.w = 1.0
    spawn_model_msg.reference_frame = "world"
    spawn_model_msg.model_name = model_name  # f"pallet{i}"

    spawn_pallet_client.call(spawn_model_msg)


def apply_force(model_name):
    body_wrench_client = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)

    body_wrench_msg = ApplyBodyWrenchRequest()
    body_wrench_msg.wrench.force.x = -12.0
    body_wrench_msg.wrench.force.y = 0.0
    body_wrench_msg.wrench.force.z = 0.0
    body_wrench_msg.reference_frame = "world"
    body_wrench_msg.start_time = rospy.Time(0, 0)
    body_wrench_msg.duration = rospy.Duration(1000, 0)

    body_wrench_msg.body_name = model_name + "::bottom"
    body_wrench_client.call(body_wrench_msg)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_robot")
    pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal,
                          queue_size=1000)
    s = rospy.Service('cmd', cmd, handle_cmd)
    rate = rospy.Rate(10.0)


    pallet_path = rospy.get_param("pallet_path")

    # arm_move_group = moveit_commander.MoveGroupCommander("manipulator")
    robot_commander = moveit_commander.RobotCommander()

    compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
    rospy.wait_for_service("/compute_ik")
    spawn_client = rospy.ServiceProxy("object_detected_service", ObjectDetectMSG)
    attach_client = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_client = rospy.ServiceProxy("/link_attacher_node/detach", Attach)

    request = AttachRequest()

    wait_product_pos = np.radians([77, -51, 74, 57, 90, 0])
    pallet_pos = np.radians([-100, -63, 74, 57, 90, 0])
    take_product = np.radians([77, -39, 74, 57, 90, 0])

    pallet_xml = open(pallet_path).read()

    pallet_id = 0
    product_number = 0
    spawn_pallet(pallet_xml, f"pallet{pallet_id}")
    while not rospy.is_shutdown():
        print(f"CMD: {command}")
        goal_pose = move_robot(wait_product_pos)
        rospy.sleep(1)
        sensor = spawn_client.call(1)

        if sensor.object_detected:
            rospy.sleep(0.5)
            goal_pose = move_robot(take_product)
            pub.publish(goal_pose)
            rospy.sleep(1.0)
            request.model_name_1 = sensor.model_name
            request.link_name_1 = "base_link"
            request.model_name_2 = "robot"
            request.link_name_2 = "wrist_3_link"
            attach_client.call(request)
            rospy.sleep(0.5)
            goal_pose = move_robot(wait_product_pos)
            pub.publish(goal_pose)
            rospy.sleep(1.0)
            goal_pose = move_robot(pallet_pos)
            pub.publish(goal_pose)
            rospy.sleep(1.5)
            detach_client.call(request)

            if product_number == 3:
                rospy.sleep(0.5)
                apply_force(f"pallet{pallet_id}")
                rospy.sleep(0.5)
                pallet_id += 1
                spawn_pallet(pallet_xml, f"pallet{pallet_id}")
                product_number = 0

            product_number += 1

        pub.publish(goal_pose)
        rate.sleep()


main()
