#!/usr/bin/env python3

import rospy
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from f110_msgs.msg import WpntArray
from tf.transformations import euler_from_quaternion


class TimeErrorTracker:
    def __init__(self):
        rospy.init_node('tracker_node', anonymous=True)

        # Set loop rate in hertz
        self.loop_rate = 40  # rate in hertz

        # Initialize variables
        self.position = None  # current position in map frame
        self.waypoints = None  # waypoints starting at car's position in map frame

        # Subscribers
        rospy.Subscriber('/car_state/pose', PoseStamped, self.car_state_cb)
        rospy.Subscriber('/global_waypoints', WpntArray, self.waypoint_cb)

    def car_state_cb(self, data):
        """
        The callback function for the '/car_state/pose' subscriber.

        Args:
            data (PoseStamped): The message containing the current pose of the car.
        """
        x = data.pose.position.x
        y = data.pose.position.y
        theta = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y,
                                       data.pose.orientation.z, data.pose.orientation.w])[2]
        self.position = [x, y, theta]

    def waypoint_cb(self, data):
        """
        The callback function for the '/global_waypoints' subscriber.

        Args:
            data (WpntArray): The message containing the global waypoints.
        """
        self.waypoints = np.empty((len(data.wpnts), 3), dtype=np.float32)
        for i, waypoint in enumerate(data.wpnts):
            speed = waypoint.vx_mps
            self.waypoints[i] = [waypoint.x_m, waypoint.y_m, speed]

    def tracking_loop(self):
        """
        The main loop of the tracker_node.

        The lateral error and lap duration is calculated and printed every lap.
        """
        lateral_error_list = []  # list of squared lateral error
        lap_count = 0  # number of laps driven
        lap_timer = time.time()  # time between laps
        rate = rospy.Rate(self.loop_rate)

        while not rospy.is_shutdown():
            # wait for position and waypoints
            if self.position is None or self.waypoints is None:
                continue

            idx_nearest_waypoint = self.nearest_waypoint(self.position[:2], self.waypoints[:, :2])

            # if all waypoints are equal set idx_nearest_waypoint to 0
            if np.isnan(idx_nearest_waypoint):
                continue

            # print duration of a lap, calculate time it needs to pass reference waypoint (waypoint 0)
            if self.distance(self.waypoints[idx_nearest_waypoint, :2], self.waypoints[0, :2]) < 0.1:
                if lap_count > 0 and time.time() - lap_timer > 1:
                    lap_duration = time.time() - lap_timer
                    print('Lap %d: Lap duration: %.3f, Lateral error: %.3f' % (lap_count,
                                                                               lap_duration,
                                                                               (np.mean(lateral_error_list))**0.5))
                    lateral_error_list = []
                    lap_count += 1
                elif lap_count == 0:
                    lap_count = 1
                lap_timer = time.time()

            # calculate lateral error
            lateral_error = self.calc_lateral_error(self.waypoints, idx_nearest_waypoint)
            lateral_error_list.append(lateral_error**2)

            rate.sleep()

    @staticmethod
    def distance(point1, point2):
        """
        Calculates the Euclidean distance between two points.

        Args:
            point1 (tuple): A tuple containing the x and y coordinates of the first point.
            point2 (tuple): A tuple containing the x and y coordinates of the second point.

        Returns:
            float: The Euclidean distance between the two points.
        """
        return (((point2[0] - point1[0]) ** 2) + ((point2[1] - point1[1]) ** 2))**0.5

    def calc_lateral_error(self, waypoints, idx_waypoint_behind_car):
        """
        Calculates the lateral error between the car and the nearest waypoint.

        Args:
            waypoints (numpy.ndarray): An array of tuples representing the x and y coordinates of waypoints.
            idx_waypoint_behind_car (int): The index of the nearest waypoint to the car.

        Returns:
            float: The lateral error between the car and the nearest waypoint.
        """
        return self.distance(waypoints[idx_waypoint_behind_car, :2], self.position[:2])

    @staticmethod
    def nearest_waypoint(position, waypoints):
        """
        Finds the index of the nearest waypoint to a given position.

        Args:
            position (tuple): A tuple containing the x and y coordinates of the position.
            waypoints (numpy.ndarray): An array of tuples representing the x and y coordinates of waypoints.

        Returns:
            int: The index of the nearest waypoint to the position.
        """
        position_array = np.array([position]*len(waypoints))
        distances_to_position = np.linalg.norm(abs(position_array - waypoints), axis=1)
        return np.argmin(distances_to_position)


if __name__ == "__main__":
    timer_error_tracker = TimeErrorTracker()
    timer_error_tracker.tracking_loop()
