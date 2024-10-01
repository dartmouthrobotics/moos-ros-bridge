#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32

if __name__ == "__main__":
    rospy.init_node("test")

    pub = rospy.Publisher("/robot_0/test_speed", Float32)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        d = Float32()
        d.data = 10.0

        pub.publish(d)
        r.sleep()
