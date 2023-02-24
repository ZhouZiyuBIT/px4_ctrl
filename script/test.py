#! /usr/bin/python3.8

import rospy

from quadrotor_sim.msg import thrust_rates
from nav_msgs.msg import Odometry

import time

last_t = time.time()
def odom_cb(msg):
    global last_t
    rospy.loginfo(time.time()-last_t)
    last_t = time.time()
    # print(msg)

if __name__ == "__main__":
    rospy.init_node("test")
    rospy.loginfo("ROS: Hello")
    tr = thrust_rates()
    # rospy.Subscriber("/quadrotor_sim/Odometry", Odometry, odom_cb)
    rospy.Subscriber("/quadrotor_sim/Odometry", Odometry, odom_cb, queue_size=1, tcp_nodelay=True)

    rospy.spin()
    import hhels
    rospy.loginfo("ROS: Byby")