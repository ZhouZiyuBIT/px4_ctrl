#! /usr/bin/python3.8

import rospy
import numpy as np

from quadrotor_sim.msg import thrust_rates
from px4_ctrl.msg import track_traj
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

import os, sys
BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/fast-go/'
sys.path += [BASEPATH]

from time_optimal_planner import WayPointOpt, cal_Ns
from gates.gates import Gates
from quadrotor import QuadrotorModel

import time

rospy.init_node("plan")
rospy.loginfo("ROS: Hello")
traj_pub = rospy.Publisher("/px4_ctrl/track_traj", track_traj, tcp_nodelay=True, queue_size=1)
gat_displacement = np.array([0,0,0])

gate = Gates(BASEPATH+"gates/gates_test.yaml")
quad = QuadrotorModel(BASEPATH+'quad/quad_real.yaml')
Ns = cal_Ns(gate, 0.1)
dts = np.array([0.1]*gate._N)
wp_opt = WayPointOpt(quad, gate._N, Ns, loop=True)
wp_opt.define_opt()
wp_opt.define_opt_t()
res = wp_opt.solve_opt([], gate._pos.flatten(), dts)

def pub_traj(opt_t_res, opt):
    x = opt_t_res['x'].full().flatten()
    traj = track_traj()
    for i in range(opt._wp_num):
        for j in range(opt._Ns[i]):
            idx = opt._N_wp_base[i]+j
            s = x[idx*opt._X_dim: (idx+1)*opt._X_dim]
            pos = Point()
            pos.x = s[0]
            pos.y = s[1]
            pos.z = s[2]
            traj.pos_pts.append(pos)
    traj_pub.publish(traj)

def timer_cb(event):
    # print(msg)
    global gat_displacement
    gate._pos[2]+=gat_displacement
    gat_displacement = np.array([0,0,0])
    res_t = wp_opt.solve_opt_t([], gate._pos.flatten())
    pub_traj(res_t, wp_opt)

def gate_displacement(msg):
    global gat_displacement
    gat_displacement[0] = msg.x
    gat_displacement[1] = msg.y
    gat_displacement[2] = msg.z

rospy.Subscriber("/plan/gate_displacement", Vector3, gate_displacement, queue_size=1)
rospy.Timer(rospy.Duration(0.01), timer_cb)

rospy.spin()
rospy.loginfo("ROS: Byby")