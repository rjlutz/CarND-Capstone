#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

from copy import deepcopy

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.twisted_cb)


        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # member variables we need
        self.pose = None
        self.twisted = None
        self.base_waypoints = None
        self.stopline_wp_idx = -1
        self.waypoints = None
        self.waypoints_2d = None
        self.tree = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(10) ## 50, could probably go as low as 30 Hz per video walkthrough

        while not rospy.is_shutdown():
            if self.pose and self.tree:
                closest_waypoint_idx = self.get_closest_waypoint_idx() #get closest waypoint
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.tree.query([x,y], 1)[1]

        closest_coord = self.waypoints_2d[closest_idx] # check if closest is ahead of or behind vehicle
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)     # Equation for hyperplane through closest coords
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])            # car's position

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx +1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):

        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]

        if (self.stopline_wp_idx == -1):  # Normal
            lane.waypoints = base_waypoints
        else:

            if (self.stopline_wp_idx >= farthest_idx or  # Too far or slightly past
                    self.stopline_wp_idx - 2 <= closest_idx):
                lane.waypoints = base_waypoints
            else:
                lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)  # Nellie slow down!

        return lane

    def get_current_speed(self):
        return self.twisted.twist.linear.x

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []

        cur_speed = self.twisted.twist.linear.x
        stop_idx = self.stopline_wp_idx - 2  # two waypoints back from the line so front of car stops at the line
        total_dist = self.arc_distance(waypoints, 0, stop_idx - closest_idx)

        # DEBUG
        rospy.loginfo("cur speed = {}".format(cur_speed))
        rospy.loginfo("closest_idx, stop_idx self.stopline_wp_idx = {} {} {}".format(closest_idx, stop_idx, self.stopline_wp_idx,closest_idx))

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = waypoints[i].pose

            dist = self.arc_distance(waypoints, i, stop_idx - closest_idx)
            vel = dist / total_dist * cur_speed  # linear ramp, fast to slower to zero, could use an S curve here

            if vel < 1.:
                vel = 0.

            if i >= 1:
                p.twist.twist.linear.x = vel
                temp.append(p)

        p = Waypoint()
        p.pose = waypoints[len(waypoints) - 1].pose
        p.twist.twist.linear.x = 0.
        temp.append(p)

        # DEBUG
        v_str = ""
        for j in range(len(temp)-1):
            v_str = v_str + "{00:.2f} ".format(temp[j].twist.twist.linear.x)
        rospy.loginfo("vels: {}\n".format(v_str))

        return temp

    def pose_cb(self, msg):
        self.pose = msg

    def twisted_cb(self, msg):
        self.twisted = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints    # from walkthrough
        self.waypoints = deepcopy(self.base_waypoints.waypoints)
        if not self.tree:
            self.waypoints_2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in self.waypoints]
            self.tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def arc_distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def overall_distance(self, waypoints): # convenience method for overall length
        return self.arc_distance(waypoints, 0, len(waypoints)-1)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
