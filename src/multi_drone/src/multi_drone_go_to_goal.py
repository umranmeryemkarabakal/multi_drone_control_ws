#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import TwistStamped, PoseStamped
import tf.transformations as tft


# ===================== DRONE =====================
class Drone:
    def __init__(self, ns):
        self.ns = ns
        self.pose = PoseStamped()

        rospy.Subscriber(
            f"/{ns}/mavros/local_position/pose",
            PoseStamped,
            self._cb_pose,
            queue_size=10
        )

        self.pub = rospy.Publisher(
            f"/{ns}/mavros/setpoint_velocity/cmd_vel",
            TwistStamped,
            queue_size=10
        )

    def _cb_pose(self, msg):
        self.pose = msg

    def pos(self):
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def yaw(self):
        q = self.pose.pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def send(self, vx, vy, vz, wz):
        cmd = TwistStamped()
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = wz
        self.pub.publish(cmd)


# ===================== MATH =====================
def vec(a, b):
    return (b[0]-a[0], b[1]-a[1], b[2]-a[2])

def norm(v):
    return math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

def scale(v, k):
    return (v[0]*k, v[1]*k, v[2]*k)

def wrap_pi(a):
    while a > math.pi:
        a -= 2*math.pi
    while a < -math.pi:
        a += 2*math.pi
    return a

def rotate_offset(offset, yaw):
    ox, oy, oz = offset
    rx = math.cos(yaw)*ox - math.sin(yaw)*oy
    ry = math.sin(yaw)*ox + math.cos(yaw)*oy
    return (rx, ry, oz)


# ===================== MAIN =====================
if __name__ == "__main__":
    rospy.init_node("real_swarm_triangle_velocity_yaw")

    leader = Drone("drone1")
    f2 = Drone("drone2")
    f3 = Drone("drone3")
    drones = [leader, f2, f3]

    rospy.sleep(2.0)  # pose wait

    # ---------- USER GOAL ----------
    goal = rospy.get_param("~leader_goal", [40.0, 0.0, 6.0])

    # ---------- FORMATION (leader frame) ----------
    d = 1.6
    h = d / math.sqrt(3)
    offsets = {
        f2: (-d, +h, 0.0),
        f3: (-d, -h, 0.0),
    }

    # ---------- GAINS ----------
    k_goal = 0.8
    k_form = 1.1
    k_rep  = 0.4
    k_yaw  = 0.6        # LOW & STABLE

    d_safe = 1.2
    max_v  = 0.9
    max_w  = 0.6

    rate = rospy.Rate(20)

    rospy.loginfo("🚀 Real swarm (velocity-aligned yaw) STARTED")

    while not rospy.is_shutdown():
        # ================= LEADER =================
        lp = leader.pos()
        lyaw = leader.yaw()

        vg = vec(lp, goal)
        dist_g = norm(vg)

        if dist_g > 0.4:
            vL = scale(vg, k_goal / dist_g)
        else:
            vL = (0.0, 0.0, 0.0)

        if norm(vL) > max_v:
            vL = scale(vL, max_v / norm(vL))

        # yaw follows VELOCITY (not goal)
        if norm(vL) > 0.1:
            yaw_ref_L = math.atan2(vL[1], vL[0])
        else:
            yaw_ref_L = lyaw

        wzL = k_yaw * wrap_pi(yaw_ref_L - lyaw)
        wzL = max(-max_w, min(max_w, wzL))

        leader.send(vL[0], vL[1], vL[2], wzL)

        # ================= FOLLOWERS =================
        for f in [f2, f3]:
            fp = f.pos()
            fyaw = f.yaw()

            # formation target (rotated with leader yaw)
            off_w = rotate_offset(offsets[f], lyaw)
            target = (
                lp[0] + off_w[0],
                lp[1] + off_w[1],
                lp[2] + off_w[2],
            )

            vf = vec(fp, target)

            # repulsion
            vrep = (0.0, 0.0, 0.0)
            for other in drones:
                if other == f:
                    continue
                op = other.pos()
                diff = vec(op, fp)
                d = norm(diff)
                if 0.05 < d < d_safe:
                    rep = scale(diff, k_rep / (d*d))
                    vrep = (
                        vrep[0] + rep[0],
                        vrep[1] + rep[1],
                        vrep[2] + rep[2],
                    )

            v = (
                k_form*vf[0] + vrep[0],
                k_form*vf[1] + vrep[1],
                k_form*vf[2] + vrep[2],
            )

            if norm(v) > max_v:
                v = scale(v, max_v / norm(v))

            # yaw follows OWN velocity
            if norm(v) > 0.1:
                yaw_ref_f = math.atan2(v[1], v[0])
            else:
                yaw_ref_f = fyaw

            wzf = k_yaw * wrap_pi(yaw_ref_f - fyaw)
            wzf = max(-max_w, min(max_w, wzf))

            f.send(v[0], v[1], v[2], wzf)

        rate.sleep()
