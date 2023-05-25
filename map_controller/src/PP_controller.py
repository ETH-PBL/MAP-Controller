#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from f110_msgs.msg import WpntArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Controller:
    def __init__(self):
        rospy.init_node('control_node', anonymous=True)

        # Get parameters for the PP controller
        self.param_q_pp = rospy.get_param('/control_node/q_pp')
        self.param_m_pp = rospy.get_param('/control_node/m_pp')
        self.param_t_clip_min = rospy.get_param('/control_node/t_clip_min')
        self.param_t_clip_max = rospy.get_param('/control_node/t_clip_max')

        self.wheelbase = rospy.get_param('/f1tenth_simulator/wheelbase')

        # Set loop rate in hertz
        self.loop_rate = 40

        # Initialize variables
        self.position = None  # current position in map frame [x, y, theta]
        self.waypoints = None  # waypoints in map frame [x, y, speed]
        self.ros_time = rospy.Time() 
        self.tf_listener = tf.TransformListener()

        # Publisher to publish lookahead point and steering angle
        self.lookahead_pub = rospy.Publisher('lookahead_point', Marker, queue_size=10)

        # Publisher for steering and speed command
        self.drive_pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_1',
                                         AckermannDriveStamped,
                                         queue_size=10)

        # Subscribers to get waypoints and the position of the car
        rospy.Subscriber('/car_state/pose', PoseStamped, self.car_state_cb)
        rospy.Subscriber('/global_waypoints', WpntArray, self.waypoints_cb)

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

    def waypoints_cb(self, data):
        """
        The callback function for the '/global_waypoints' subscriber.

        Args:
            data (WpntArray): The message containing the global waypoints.
        """
        self.waypoints = np.empty((len(data.wpnts), 3), dtype=np.float32)
        for i, waypoint in enumerate(data.wpnts):
            speed = waypoint.vx_mps
            self.waypoints[i] = [waypoint.x_m, waypoint.y_m, speed]

    def control_loop(self):
        """
        Control loop for the Pure Pursuit controller.
        """
        rate = rospy.Rate(self.loop_rate)

        while not rospy.is_shutdown():
            # Wait for position and waypoints
            if self.position is None or self.waypoints is None:
                continue

            # Send speed and steering commands to mux
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = self.ros_time.now()
            ack_msg.header.frame_id = 'base_link'

            if self.waypoints.shape[0] > 2:
                idx_nearest_waypoint = self.nearest_waypoint(
                    self.position[:2], self.waypoints[:, :2])

                # Desired speed at waypoint closest to car
                target_speed = self.waypoints[idx_nearest_waypoint, 2]

                # Calculate Pure Pursuit
                # Define lookahead distance as an affine function with  tuning parameter m and q
                lookahead_distance = self.param_q_pp + target_speed*self.param_m_pp
                lookahead_distance = np.clip(lookahead_distance, self.param_t_clip_min, self.param_t_clip_max)
                lookahead_point = self.waypoint_at_distance_infront_car(lookahead_distance,
                                                                        self.waypoints[:, :2],
                                                                        idx_nearest_waypoint)
                if lookahead_point.any() is not None:
                    lookahead_point_in_baselink = self.map_to_baselink(lookahead_point)
                    steering_angle = self.get_actuation(lookahead_point_in_baselink)

                    ack_msg.drive.steering_angle = steering_angle
                    ack_msg.drive.speed = np.max(target_speed, 0)  # No negative speed

                    self.visualize_lookahead(lookahead_point)
                    self.visualize_steering(steering_angle)

            # If there are no waypoints, publish zero speed and steer to STOP
            else:
                ack_msg.drive.speed = 0
                ack_msg.drive.steering_angle = 0
                rospy.logerr('[PP Controller]: Received no waypoints. STOPPING!!!')

            # Always publish ackermann msg
            self.drive_pub.publish(ack_msg)
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

    def waypoint_at_distance_infront_car(self, distance, waypoints, idx_waypoint_behind_car):
        """
        Finds the waypoint a given distance in front of a given waypoint.

        Args:
            distance (float): The distance to travel from the given waypoint.
            waypoints (numpy.ndarray): An array of tuples representing the x and y coordinates of waypoints.
            idx_waypoint_behind_car (int): The index of the waypoint behind the car.

        Returns:
            numpy.ndarray: A tuple containing the x and y coordinates of the waypoint a given distance in front of the given waypoint.
        """
        dist = 0
        i = idx_waypoint_behind_car

        while dist < distance:
            i = (i + 1) % len(waypoints)
            dist = self.distance(waypoints[idx_waypoint_behind_car], waypoints[i])

        return np.array(waypoints[i])

    def get_actuation(self, lookahead_point):
        """
        Calculates the steering angle required to reach the given point.

        Args:
            lookahead_point (np.ndarray): The position of the lookahead point in the base_link frame.

        Returns:
            float: The steering angle required to reach the lookahead point.
        """
        waypoint_y = lookahead_point[1]
        if np.abs(waypoint_y) < 1e-6:
            return 0
        radius = np.linalg.norm(lookahead_point)**2 / (2.0 * waypoint_y)
        steering_angle = np.arctan(self.wheelbase / radius)
        return steering_angle

    def map_to_baselink(self, point):
        """
        Transforms the given point from the map frame to the base_link frame.

        Args:
            point (np.ndarray): The position of the lookahead point in the map frame.

        Returns:
            np.ndarray: The position of the lookahead point in the base_link frame.
        """
        t_map = self.tf_listener.getLatestCommonTime("map", "base_link")

        posestamped_in_map = PoseStamped()
        posestamped_in_map.header.stamp = t_map
        posestamped_in_map.header.frame_id = 'map'
        posestamped_in_map.pose.position.x = point[0]
        posestamped_in_map.pose.position.y = point[1]
        posestamped_in_map.pose.position.z = 0

        pose_in_baselink = self.tf_listener.transformPose(
            "base_link", posestamped_in_map)

        x = pose_in_baselink.pose.position.x
        y = pose_in_baselink.pose.position.y
        return np.array([x, y])

    def visualize_steering(self, theta):
        """
        Publishes a visualization of the steering direction as an arrow.

        Args:
            theta (float): The steering angle in radians.
        """
        quaternions = quaternion_from_euler(0, 0, theta)

        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "base_link"
        lookahead_marker.header.stamp = self.ros_time.now()
        lookahead_marker.type = Marker.ARROW
        lookahead_marker.id = 2
        lookahead_marker.scale.x = 0.6
        lookahead_marker.scale.y = 0.05
        lookahead_marker.scale.z = 0
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.lifetime = rospy.Duration()
        lookahead_marker.pose.position.x = 0
        lookahead_marker.pose.position.y = 0
        lookahead_marker.pose.position.z = 0
        lookahead_marker.pose.orientation.x = quaternions[0]
        lookahead_marker.pose.orientation.y = quaternions[1]
        lookahead_marker.pose.orientation.z = quaternions[2]
        lookahead_marker.pose.orientation.w = quaternions[3]
        self.lookahead_pub.publish(lookahead_marker)

    def visualize_lookahead(self, lookahead_point):
        """
        Publishes a marker indicating the lookahead point on the map.

        Args:
            lookahead_point (tuple): A tuple of two floats representing the x and y coordinates of the lookahead point.
        """
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.ros_time.now()
        lookahead_marker.type = 2
        lookahead_marker.id = 1
        lookahead_marker.scale.x = 0.15
        lookahead_marker.scale.y = 0.15
        lookahead_marker.scale.z = 0.15
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = lookahead_point[0]
        lookahead_marker.pose.position.y = lookahead_point[1]
        lookahead_marker.pose.position.z = 0
        lookahead_marker.pose.orientation.x = 0
        lookahead_marker.pose.orientation.y = 0
        lookahead_marker.pose.orientation.z = 0
        lookahead_marker.pose.orientation.w = 1
        self.lookahead_pub.publish(lookahead_marker)


if __name__ == "__main__":
    controller = Controller()
    controller.control_loop()
