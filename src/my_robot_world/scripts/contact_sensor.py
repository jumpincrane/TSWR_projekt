#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ContactsState
from my_robot_world.srv import *

contact_state = ContactsState()


def callback_sensor(data):
    global contact_state
    contact_state = data.states


def check_sensor(req):
    global contact_state
    object_detected = False
    if not contact_state:
        object_detected = False
    else:
        object_detected = True

    return object_detected


def main():
    rospy.init_node("contact_sensor_service")
    rospy.Subscriber("/sensor_contact", ContactsState, callback_sensor)
    srv = rospy.Service("object_detected_service", ObjectDetectMSG, check_sensor)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Error: ROS CLOSED")
