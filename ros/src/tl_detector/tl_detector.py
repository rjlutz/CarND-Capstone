#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import math
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):

    ### following for frame generation only
    cols_count = [0, 0, 0, 0]
    frame_count = 0
    states = ['red', 'yellow', 'green', 'none']

    def __init__(self):
        rospy.init_node('tl_detector')
        rospy.loginfo('TLDetector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        #rospy.loginfo("got waypoints, size  = " + len(waypoints))
        # rospy.loginfo("got waypoints, size waypoints.waypoints  = " + str(len(waypoints.waypoints)))
        self.waypoints = waypoints
        # rospy.loginfo("             , size self_waypoints.waypoints  = " + str(len(self.waypoints.waypoints)))

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()

        # ###
        # # the following is only for image capture
        # self.frame_count = self.frame_count + 1
        # ego = self.pose.pose.position
        # lt = self.waypoints.waypoints[light_wp].pose.pose.position
        # if (self.distance(ego.x, ego.y, lt.x, lt.y) < 120):
        #     col_idx = state
        # else:
        #     col_idx = 3
        #
        # if self.cols_count[col_idx] < 200 and self.frame_count % 10 == 0:
        #     cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #     self.cols_count[col_idx] = self.cols_count[col_idx] + 1
        #     rospy.loginfo(
        #         "dist: {} frame:{} colors:{}".format(self.distance(ego.x, ego.y, lt.x, lt.y), self.frame_count,
        #                                              self.cols_count))
        #     path = ''
        #     fn =  path + self.states[col_idx] + '-img' + str(self.frame_count) + '.jpg'
        #     rospy.loginfo("fn = {}".format(fn))
        #     cv2.imwrite(fn, cv_image)
        #
        # ###


        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            # rospy.loginfo("light_wp, state = {} {}".format(light_wp, state))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1


    def get_closest_waypoint(self, x, y):
        closest_dist = float('inf')
        closest_wp = 0
        for i in range(len(self.waypoints.waypoints)):
            wpt = self.waypoints.waypoints[i].pose.pose.position
            dist = self.distance(x, y, wpt.x, wpt.y)
            if dist < closest_dist:
                closest_dist = dist
                closest_wp = i

        return closest_wp

    # def get_closest_waypoint(self, pose):
    #     """Identifies the closest path waypoint to the given position
    #         https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
    #     Args:
    #         pose (Pose): position to match a waypoint to
    #
    #     Returns:
    #         int: index of the closest waypoint in self.waypoints
    #
    #     """
    #     #TODO implement
    #     ### from walkthrough
    #     ######closest_idx = self.waypoint_tree.query([pose.x, pose.y], 1)[1]
    #
    #     return closest_idx
    #     ###return 0

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False
        #
        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #
        # #Get classification
        # return self.light_classifier.get_classification(cv_image)
        return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        ### following walkthrough
        ###light = None
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            ###
            pos = self.pose.pose.position;
            car_wp_idx = self.get_closest_waypoint(pos.x, pos.y)
            ###car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)
        diff = len(self.waypoints.waypoints)
        for i, light in enumerate(self.lights):
            # get stop line waypoints index
            line = stop_line_positions[i]
            temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
            # find the closet stop line waypoint index
            d = temp_wp_idx - car_wp_idx
            if d >= 0 and d < diff:
                diff = d
                closest_light = light
                line_wp_idx = temp_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state

        ###self.waypoints = None

        return -1, TrafficLight.UNKNOWN

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

