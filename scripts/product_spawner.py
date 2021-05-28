#!/usr/bin/env python3

# spawn products on product's conveyor belt and add bodywrench
from xml.dom import minidom
import sys
import rospy
from gazebo_msgs.srv import SpawnModel, ApplyBodyWrench, SpawnModelRequest, ApplyBodyWrenchRequest, SpawnModelResponse
from std_msgs.msg import Int8MultiArray, Float32MultiArray


def main():
    rospy.init_node("products_spawner")
    spawn_client = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    body_wrench_client = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
    rate = rospy.Rate(0.1)  # 10 sec

    # srv variables
    body_wrench_msg = ApplyBodyWrenchRequest()
    spawn_model_msg = SpawnModelRequest()
    # wait for services
    spawn_client.wait_for_service()
    rospy.loginfo("spawn_urdf_model service is ready")
    body_wrench_client.wait_for_service()
    rospy.loginfo("apply_body_wrench service is ready")

    # get path for urdf file
    product_path = rospy.get_param("product_path")
    product_xml = open(product_path).read()

    # initial conveyor belt movement
    body_wrench_msg.wrench.force.x = -0.000012
    body_wrench_msg.wrench.force.y = 0.0
    body_wrench_msg.wrench.force.z = 0.0
    body_wrench_msg.reference_frame = "world"
    body_wrench_msg.start_time = rospy.Time(0, 0)
    body_wrench_msg.duration = rospy.Duration(-1, 0)
    # initial spawn model
    i = 0
    spawn_model_msg.model_xml = product_xml
    spawn_model_msg.initial_pose.position.x = 7.0
    spawn_model_msg.initial_pose.position.y = 0.843470
    spawn_model_msg.initial_pose.position.z = 0.71
    spawn_model_msg.initial_pose.orientation.x = 0.0
    spawn_model_msg.initial_pose.orientation.y = 0.0
    spawn_model_msg.initial_pose.orientation.z = 0.0
    spawn_model_msg.initial_pose.orientation.w = 0.0
    spawn_model_msg.reference_frame = "world"
    spawn_model_msg.model_name = f"product_{i}"

    while not rospy.is_shutdown():
        spawn_client.call(spawn_model_msg)
        body_wrench_msg.body_name = spawn_model_msg.model_name + "::base_link"
        body_wrench_client.call(body_wrench_msg)
        i = i + 1
        spawn_model_msg.model_name = f"product_{i}"
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Error: ROS CLOSED")
