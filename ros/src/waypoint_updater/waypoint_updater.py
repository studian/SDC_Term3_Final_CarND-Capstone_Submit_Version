#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from std_msgs.msg import Int32, Float32

import math
import tf
from copy import deepcopy
from scipy.interpolate import CubicSpline

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
STOP_DIST = 2.0  # (in meters) Distance from closest traffic light to decide whether to top or go through intersection
ACCEL = 1.0  # Velocity increment (m/s) to be applied at each waypoint


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.base_waypoints = None
        self.final_waypoints = None
        self.current_pose = None
        self.next_waypoint_index = None
        self.light_wp = None
        self.max_speed = None
        self.slow_dist = None
        self.current_velocity = 0 # or None ??

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        self.wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.loop()

    
    def loop(self):
        rate = rospy.Rate(5)

        # While ROS is running
        while not rospy.is_shutdown():
            is_initialized = self.base_waypoints and self.current_pose and self.light_wp

            # If we already initialized
            if is_initialized:
                wpstart_idx = self.closest_waypoint(self.current_pose.position)
                # If this waypoint is behind the current pose then update to next waypoint
                self.next_waypoint_index = self.ahead_waypoint(wpstart_idx)
                self.max_speed = self.get_waypoint_velocity(self.base_waypoints[self.next_waypoint_index])
                # rospy.logwarn("self.max_speed %s", self.max_speed)
                # Set the waypoints' speeds and publish results
                self.publish()
            rate.sleep()

    
    def publish(self):

        final_waypoints_msg = Lane()
        final_waypoints_msg.header.stamp = rospy.Time.now()

        self.set_final_waypoints()
        self.set_final_waypoints_speed()
        final_waypoints_msg.waypoints = self.final_waypoints

        self.final_waypoints_pub.publish(final_waypoints_msg)

    
    def set_final_waypoints(self):
        # Grab the next LOOKAHEAD_WPS waypoints from the base waypoints.
        self.final_waypoints = deepcopy(
            self.base_waypoints[self.next_waypoint_index: self.next_waypoint_index + LOOKAHEAD_WPS])

        # If wraparound occurs, handle it by grabbing waypoints from beginning of base waypoints.
        rem_points = LOOKAHEAD_WPS - len(self.final_waypoints)
        if rem_points > 0:
            self.final_waypoints = self.final_waypoints + deepcopy(self.base_waypoints[0:rem_points])

    
    def set_final_waypoints_speed(self):
        # Get the distance between the next waypoint and the traffic light.
        dist = self.distance(self.base_waypoints, self.next_waypoint_index, abs(self.light_wp))

        no_red_light_detected = self.light_wp < 0
        # IF no RED light,
        if no_red_light_detected:
            # rospy.logwarn("Upcoming TrafficLight %d" % -self.light_wp)
            speed = self.current_velocity
            target_speed = self.max_speed
            # rospy.logwarn("No traffic light, so accelerate to top speed! ")

            Creep_Enabled = False
            if Creep_Enabled and dist < self.slow_dist:
                target_speed = self.max_speed / 2.0
            for wp in self.final_waypoints:
                # if speed > target_speed:
                #     speed = min(target_speed, speed + ACCEL)
                # else:
                #     speed = max(target_speed, speed - ACCEL)
                speed = target_speed
                self.set_waypoint_velocity(wp, speed)
            return
        # Report message of upcoming traffic light.
        # rospy.logwarn("Next wp: %s, Next TL wp: %s, distance: %s", self.next_waypoint_index, self.light_wp, dist)
        if dist <= self.slow_dist:
            speed = max(self.current_velocity, max(dist * 0.2, 0.3))
            # speed = self.current_velocity
            if dist > STOP_DIST:
                decel = speed / (dist - STOP_DIST)
                rospy.logwarn("*** Decelerating! My next wp: %s, is close to Next TL wp: %s, So decelerate to with : %f m/s", self.next_waypoint_index, self.light_wp, decel)
            else:
                speed = 0
                decel = 0
                rospy.logwarn("!!! You are into intersection while RED light on! Vehicle Stopped")
            for wp in self.final_waypoints:
                speed = max(0, speed - decel)
                self.set_waypoint_velocity(wp, speed)
        else:
            speed = self.current_velocity
            if speed < self.max_speed:
                rospy.logwarn("~~~ Traffic light far away, so accelerate to top speed! ")
                for wp in self.final_waypoints:
                    # if speed > self.max_speed:
                    #     speed = min(self.max_speed, speed + ACCEL)
                    # else:
                    #     speed = max(self.max_speed, speed - ACCEL)
                    speed = self.max_speed
                    self.set_waypoint_velocity(wp, speed)

    
    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose

    
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints
        if self.base_waypoints:
            self.wp_sub.unregister()
            self.wp_sub = None
            # self.max_speed = self.get_waypoint_velocity(self.base_waypoints[0])  # get velocity from waypoint loader
            # rospy.logwarn("self.max_speed %s", self.max_speed)
            # self.max_speed = 12  # get velocity from waypoint loader
            # self.slow_dist = self.max_speed * 5
            self.slow_dist = max(self.get_waypoint_velocity(self.base_waypoints[0]) * 5, 12 * 3)

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.light_wp = msg.data
        # rospy.logwarn("self.light_wp = %s", self.light_wp)


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        # self.obstacle_waypoint = msg.data
        pass


    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x


    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity


    def distance(self, waypoints, wp1, wp2):
        dist, wp3 = 0.0, -1
        # In case of wraparound
        if wp2 < wp1:
            wp3, wp2 = wp2, len(waypoints) - 1
        for i in xrange(wp1, wp2):
            dist += self.dist_bw_wp(waypoints[i].pose.pose.position, waypoints[i + 1].pose.pose.position)
        for i in xrange(-1, wp3):
            dist += self.dist_bw_wp(waypoints[i].pose.pose.position, waypoints[i + 1].pose.pose.position)
        return dist


    def dist_bw_wp(self, p, q):
        dist = math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2)
        return dist


    def closest_waypoint(self, position):
        return min(xrange(len(self.base_waypoints)),
                   key=lambda p: self.dist_bw_wp(position, self.base_waypoints[p].pose.pose.position))


    def get_next_waypoint(self, pose, waypoints):
        closest_wp = self.get_closest_waypoint(pose, waypoints)
        wp_x = waypoints[closest_wp].pose.pose.position.x
        wp_y = waypoints[closest_wp].pose.pose.position.y
        heading = math.atan2((wp_y - pose.pose.position.y), (wp_x - pose.pose.position.x))
        x = pose.pose.orientation.x
        y = pose.pose.orientation.y
        z = pose.pose.orientation.z
        w = pose.pose.orientation.w
        euler_angles_xyz = tf.transformations.euler_from_quaternion([x, y, z, w])
        theta = euler_angles_xyz[-1]
        angle = math.fabs(theta - heading)
        if angle > math.pi / 4.0:
            closest_wp += 1

        return closest_wp


    def get_euler_yaw(self):
        quaternion = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]


    def ahead_waypoint(self, wp_idx):
        ahead_idx = wp_idx

        # Get coordinates of the waypoint at the specified index.
        map_wp_x = self.base_waypoints[wp_idx].pose.pose.position.x
        map_wp_y = self.base_waypoints[wp_idx].pose.pose.position.y

        # Get current position
        x, y = self.current_pose.position.x, self.current_pose.position.y

        # Get yaw angle
        yaw = self.get_euler_yaw()

        # Compute expression to determine if closest waypoint is behind us or not.
        localize_x = (map_wp_x - x) * math.cos(yaw) + (map_wp_y - y) * math.sin(yaw)

        # If the waypoint is behind us, then increment the waypoint index.
        if localize_x < 0.0:
            ahead_idx = ahead_idx + 1

        # Set the internal property
        self.ahead_waypoint_index = ahead_idx

        # And return the index as a return argument.
        return ahead_idx


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
