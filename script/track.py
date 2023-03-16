#! /usr/bin/python3.8

import numpy as np
import casadi as ca
import time

# ROS
import rospy

from px4_ctrl.msg import TrackTraj, ThrustRates
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# 
import os, sys
BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/fast-go/'
sys.path += [BASEPATH]

from quadrotor import QuadrotorModel
from tracker import TrackerOpt, TrackerOpt2, TrackerPosVel, TrackerPosVel2, TrackerMPCC, TrackerP
from trajectory import Trajectory
from gates.gates import Gates

from plotting import plot_gates_2d, plot_traj_xy

rospy.init_node("track")
rospy.loginfo("ROS: Hello")

# traj = Trajectory(BASEPATH+"results/res_t_n6.csv")
traj = Trajectory()
quad =  QuadrotorModel(BASEPATH+'quad/quad_real.yaml')

# tracker = TrackerPosVel(quad)
tracker = TrackerOpt(quad)
# tracker = TrackerMPCC(quad)
# tracker = TrackerP(quad)
# tracker.define_opt()
tracker.load_so(BASEPATH+"generated/tracker_opt.so")

stop_tracker = TrackerOpt2(quad)
stop_tracker.define_opt()

ctrl_pub = rospy.Publisher("~thrust_rates", ThrustRates, tcp_nodelay=True, queue_size=1)

stop_flag = False
def stop_cb(msg: Bool):
    global stop_flag
    stop_flag = msg.data

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
    if traj._N != 0:
        x0 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
                    msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        qw=x0[6]
        qz=x0[9]
        ql=np.sqrt(qw*qw+qz*qz)
        # print(qw/ql, qz/ql)
        r_x.append(msg.pose.pose.position.x)
        r_y.append(msg.pose.pose.position.y)
        trjp, trjv, trjdt, ploy = traj.sample(tracker._trj_N, x0[:3])
        # print(trjdt)
        if not stop_flag:
            print(cnt)
            # res = tracker.solve(x0, trjp.reshape(-1), trjv.reshape(-1), trjdt, np.array([-10000,100,-1]), 20)
            # res = tracker.solve(x0, ploy.reshape(-1), trjdt, 20)
            print("track:")
            res = tracker.solve(x0, trjp.reshape(-1), 20)
            # res = tracker.solve(x0, trjp.reshape(-1), trjv.reshape(-1))
        else:
            res = stop_tracker.solve(x0, trjp.reshape(-1), 20)
        
        x = res['x'].full().flatten()
        # print(x[-10:])
        Tt = 1*(x[tracker._Herizon*13+0]+x[tracker._Herizon*13+1]+x[tracker._Herizon*13+2]+x[tracker._Herizon*13+3])
        
        # thrust to z_accel
        # qw, qx, qy, qz = x0[6], x0[7], x0[8], x0[9]
        # vx, vy, vz = x0[3], x0[4], x0[5]
        # c_n = ( Tt/quad._m + quad._D[2,2]*(2*(qw*qy+qx*qz)*vx+2*(qy*qz-qw*qx)*vy+(qw*qw-qx*qx-qy*qy+qz*qz)*vz) )
        # print(c_n)

        u = ThrustRates()
        u.thrust = Tt/4/quad._T_max
        u.wx = x[10]
        u.wy = x[11]
        u.wz = x[12]
        ctrl_pub.publish(u)

def track_traj_cb(msg: TrackTraj):
    pos = []
    vel = []
    quat = []
    angular = []
    dt = []
    for i in range(len(msg.position)):
        pos.append([msg.position[i].x, msg.position[i].y, msg.position[i].z])
        vel.append([msg.velocity[i].x, msg.velocity[i].y, msg.velocity[i].z])
        quat.append([msg.orientation[i].w, msg.orientation[i].x, msg.orientation[i].y, msg.orientation[i].z])
        angular.append([msg.angular[i].x, msg.angular[i].y, msg.angular[i].z])
        dt.append(msg.dt[i])

    traj.load_data(np.array(pos), np.array(vel), np.array(quat), np.array(angular), np.array(dt))

# rospy.Subscriber("/quadrotor_sim/Odometry", Odometry, odom_cb)
rospy.Subscriber("~odom", Odometry, odom_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("~track_traj", TrackTraj, track_traj_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("stop_flag", Bool, stop_cb)

rospy.spin()
rospy.loginfo("ROS: Goodby")
import matplotlib.pyplot as plt
ax = plt.gca()
plt.plot(traj._pos[:,0], traj._pos[:,1])
plt.plot(r_x, r_y)
plt.show()
