#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray



def obstacles_pub():
    pub = rospy.Publisher('obstacles_pub', Float64MultiArray, queue_size=10)
    data_to_send = Float64MultiArray()
    data_to_send.data = [0,1.5,0,3,0,4.5,1.5,0,1.5,1.5,1.5,3,1.5,4.5,3,0,3,1.5,3,3,3,4.5,4.5,0,4.5,1.5,4.5,3,4.5,4.5]
    rospy.init_node('obstacles_pub', anonymous = True)
    rate = rospy.Rate(50) # 50 Hz

    while not rospy.is_shutdown():
        pub.publish(data_to_send)
        rate.sleep

if __name__ == '__main__':
    try:
        obstacles_pub()
    except rospy.ROSInterruptException:
        pass