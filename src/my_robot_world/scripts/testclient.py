#!/usr/bin/env python3

import rospy
from my_robot_world.srv import *

def main():
    rospy.init_node("testclient")
    spawn_client = rospy.ServiceProxy("object_detected_service", ObjectDetectMSG)
    rate = rospy.Rate(5)  # 10 sec
    msgsrv = ObjectDetectMSGRequest()
    msgsrv = 1

    while not rospy.is_shutdown():
        p = spawn_client.call(msgsrv)
        print(p)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Error: ROS CLOSED")