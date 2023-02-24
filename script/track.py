#! /usr/bin/python3.8

import numpy as np
import casadi as ca
import time

# ROS
import rospy

from quadrotor_sim.msg import thrust_rates
from nav_msgs.msg import Odometry

# 
import os, sys
BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/fast-go/'
sys.path += [BASEPATH]

from quadrotor import QuadrotorModel
from tracker import TrackerOpt, TrackerOpt2
from trajectory import Trajectory
from gates.gates import Gates

from plotting import plot_gates_2d, plot_traj_xy

traj = Trajectory(BASEPATH+"results/res_t_n6.csv")
quad = QuadrotorModel(BASEPATH+'quad/quad.yaml')
gates = Gates(BASEPATH+"gates/gates_n6.yaml")

tracker = TrackerOpt(quad)
tracker.define_opt()
tracker.reset_xul()

stop_tracker = TrackerOpt2(quad)
stop_tracker.define_opt()
stop_tracker.reset_xul()

ctrl_pub = rospy.Publisher("/quadrotor_sim/thrust_rates", thrust_rates, tcp_nodelay=True, queue_size=1)

r_x = []
r_y = []
# last_t = time.time()
cnt = 0
def odom_cb(msg: Odometry):
    # global last_t 
    # rospy.loginfo(time.time()-last_t)
    # last_t = time.time()
    global cnt
    cnt += 1

    x0 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                   msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
                   msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                   msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
    r_x.append(msg.pose.pose.position.x)
    r_y.append(msg.pose.pose.position.y)
    trjp = traj.sample(tracker._trj_N, x0[:3]).reshape(-1)
    if cnt<1000:
        res = tracker.solve(x0, trjp, 20)
    else:
        res = stop_tracker.solve(x0, trjp, 20)
    x = res['x'].full().flatten()
    u = thrust_rates()
    u.thrust = 1.0*(x[tracker._Herizon*13+0]+x[tracker._Herizon*13+1]+x[tracker._Herizon*13+2]+x[tracker._Herizon*13+3])/4
    u.wx = x[10]
    u.wy = x[11]
    u.wz = x[12]
    ctrl_pub.publish(u)

if __name__ == "__main__":
    rospy.init_node("test")
    rospy.loginfo("ROS: Hello")
    tr = thrust_rates()
    # rospy.Subscriber("/quadrotor_sim/Odometry", Odometry, odom_cb)
    rospy.Subscriber("/quadrotor_sim/Odometry", Odometry, odom_cb, queue_size=1, tcp_nodelay=True)

    rospy.spin()
    rospy.loginfo("ROS: Goodby")
    import matplotlib.pyplot as plt
    ax = plt.gca()
    plot_gates_2d(ax, gates)
    plt.plot(traj._pos[:,0], traj._pos[:,1])
    plt.plot(r_x, r_y)
    plt.show()
