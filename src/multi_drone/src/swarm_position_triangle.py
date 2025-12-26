#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
import tf.transformations as tft


def quat_from_yaw(yaw: float) -> Quaternion:
    q = tft.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def yaw_from_quat(q: Quaternion) -> float:
    _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def rotate_2d(x: float, y: float, yaw: float):
    """Rotate (x,y) by yaw in world frame."""
    cx = math.cos(yaw)
    sx = math.sin(yaw)
    rx = cx * x - sx * y
    ry = sx * x + cx * y
    return rx, ry


class DroneSP:
    def __init__(self, ns: str):
        self.ns = ns.strip("/")
        self.pose = PoseStamped()

        self.sub = rospy.Subscriber(
            f"/{self.ns}/mavros/local_position/pose",
            PoseStamped,
            self._cb_pose,
            queue_size=10
        )
        self.pub = rospy.Publisher(
            f"/{self.ns}/mavros/setpoint_position/local",
            PoseStamped,
            queue_size=10
        )

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg

    def pos(self):
        p = self.pose.pose.position
        return p.x, p.y, p.z

    def yaw(self):
        return yaw_from_quat(self.pose.pose.orientation)

    def publish_setpoint(self, x, y, z, yaw, frame_id="map"):
        sp = PoseStamped()
        sp.header.stamp = rospy.Time.now()
        sp.header.frame_id = frame_id
        sp.pose.position.x = float(x)
        sp.pose.position.y = float(y)
        sp.pose.position.z = float(z)
        sp.pose.orientation = quat_from_yaw(float(yaw))
        self.pub.publish(sp)


def repulse_targets(targets_xyz, min_dist=1.2, k=0.6, iters=1):
    """
    Position-space collision avoidance:
    If two targets are closer than min_dist, push them apart by adjusting targets.
    This DOES NOT output velocity; it modifies goal points (paper-style high-level tweak).
    """
    names = list(targets_xyz.keys())
    t = {n: list(targets_xyz[n]) for n in names}

    for _ in range(iters):
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                ni, nj = names[i], names[j]
                dx = t[ni][0] - t[nj][0]
                dy = t[ni][1] - t[nj][1]
                dz = t[ni][2] - t[nj][2]
                d = math.sqrt(dx*dx + dy*dy + dz*dz)
                if d < 1e-6:
                    # same point; arbitrary small split
                    dx, dy, dz, d = 1e-3, 0.0, 0.0, 1e-3
                if d < min_dist:
                    # push apart along line
                    push = k * (min_dist - d) / min_dist
                    ux, uy, uz = dx / d, dy / d, dz / d
                    # symmetric push
                    t[ni][0] += ux * push
                    t[ni][1] += uy * push
                    t[ni][2] += uz * push
                    t[nj][0] -= ux * push
                    t[nj][1] -= uy * push
                    t[nj][2] -= uz * push

    return {n: tuple(t[n]) for n in names}


if __name__ == "__main__":
    rospy.init_node("swarm_position_triangle")

    # Namespaces
    leader_ns = rospy.get_param("~leader_ns", "drone1")
    f2_ns = rospy.get_param("~follower2_ns", "drone2")
    f3_ns = rospy.get_param("~follower3_ns", "drone3")

    leader = DroneSP(leader_ns)
    f2 = DroneSP(f2_ns)
    f3 = DroneSP(f3_ns)

    rate_hz = float(rospy.get_param("~rate_hz", 20.0))
    rate = rospy.Rate(rate_hz)

    # Formation geometry (equilateral triangle in leader frame)
    # d = leader->follower distance (side length style)
    d = float(rospy.get_param("~d", 1.8))
    h = d / math.sqrt(3.0)  # gives equilateral layout behind leader

    # Offsets in LEADER frame:
    # followers are behind leader (-d) and spread +/- h
    off2_local = (-d, +h, 0.0)
    off3_local = (-d, -h, 0.0)

    # Collision parameters (goal-space)
    min_dist = float(rospy.get_param("~min_dist", 1.2))
    rep_k = float(rospy.get_param("~rep_k", 0.7))
    rep_iters = int(rospy.get_param("~rep_iters", 1))

    # Goal tolerance only for yaw behavior (not to stop publishing!)
    # IMPORTANT: We ALWAYS publish setpoints; autopilot keeps tracking.
    goal_yaw_mode = rospy.get_param("~leader_yaw_mode", "face_goal")  # or "hold"
    follower_yaw_mode = rospy.get_param("~follower_yaw_mode", "follow_leader")  # or "face_leader_goal"

    rospy.sleep(2.0)  # let poses populate

    # Pre-stream setpoints (helps some setups)
    # publish current pose as hold for a short time
    for _ in range(int(rate_hz * 1.0)):
        if rospy.is_shutdown():
            break
        lx, ly, lz = leader.pos()
        lyaw = leader.yaw()
        leader.publish_setpoint(lx, ly, lz, lyaw)
        f2.publish_setpoint(*f2.pos(), f2.yaw())
        f3.publish_setpoint(*f3.pos(), f3.yaw())
        rate.sleep()

    rospy.loginfo("Swarm position controller started (paper-style: goal points, not cmd_vel).")

    while not rospy.is_shutdown():
        # Leader goal can be changed live:
        # rosparam set /swarm_position_triangle/leader_goal "[x,y,z]"
        leader_goal = rospy.get_param("~leader_goal", [20.0, 0.0, 6.0])
        gx, gy, gz = float(leader_goal[0]), float(leader_goal[1]), float(leader_goal[2])

        lx, ly, lz = leader.pos()
        lyaw = leader.yaw()

        # Leader yaw reference
        if goal_yaw_mode == "face_goal":
            yaw_goal = math.atan2(gy - ly, gx - lx)
        else:
            yaw_goal = lyaw  # hold

        # Rotate formation offsets by leader yaw (yaw-aware triangle)
        off2_w = rotate_2d(off2_local[0], off2_local[1], lyaw)
        off3_w = rotate_2d(off3_local[0], off3_local[1], lyaw)

        t_leader = (gx, gy, gz)
        t_f2 = (lx + off2_w[0], ly + off2_w[1], lz + off2_local[2])
        t_f3 = (lx + off3_w[0], ly + off3_w[1], lz + off3_local[2])

        # Optional: goal-space collision adjustment
        targets = {
            "leader": t_leader,
            "f2": t_f2,
            "f3": t_f3,
        }
        targets = repulse_targets(targets, min_dist=min_dist, k=rep_k, iters=rep_iters)

        t_leader = targets["leader"]
        t_f2 = targets["f2"]
        t_f3 = targets["f3"]

        # Followers yaw reference
        if follower_yaw_mode == "follow_leader":
            yaw_f2 = yaw_goal
            yaw_f3 = yaw_goal
        else:
            # face leader goal direction
            yaw_f2 = yaw_goal
            yaw_f3 = yaw_goal

        # Publish setpoints continuously
        leader.publish_setpoint(t_leader[0], t_leader[1], t_leader[2], yaw_goal)
        f2.publish_setpoint(t_f2[0], t_f2[1], t_f2[2], yaw_f2)
        f3.publish_setpoint(t_f3[0], t_f3[1], t_f3[2], yaw_f3)

        rate.sleep()
