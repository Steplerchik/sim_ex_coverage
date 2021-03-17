#!/usr/bin/env python3
import numpy as np

import rospy
import tf2_ros
from std_msgs.msg import Bool
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker
from libs.structures import MapData, STMap
from libs.sim_ex_coverage_planner import SimExCoveragePlanner
from ros_conversions import cvt_ros_pose2point, get_transform, cvt_map2occupancy, cvt_occupancy2map
from ros_functions import publish_map, publish_marker, publish_path
import threading


class SimExCoverageNode(object):
    def __init__(self):
        rospy.init_node('sim_ex_coverage_node', anonymous=True)

        # ros params:
        self.base_frame = rospy.get_param("~base_frame", "ump")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.lethal_cost_threshold = rospy.get_param("~lethal_cost_threshold", 60)
        robot_size = rospy.get_param("~robot_size", 1.0)

        self.mutex = threading.Lock()

        # variables
        self.map = MapData()
        self.planner = SimExCoveragePlanner(robot_size)

        # objects for listening frame transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # publishers:
        self.exploration_path_publisher = rospy.Publisher("~exploration_path", Path, queue_size=1)
        self.full_path_publisher = rospy.Publisher("~full_path", Marker, queue_size=1)
        self.sp_publisher = rospy.Publisher("~spanning_tree", Marker, queue_size=1)
        self.sub_map_publisher = rospy.Publisher("~sub_map", OccupancyGrid, queue_size=1)
        self.mega_map_publisher = rospy.Publisher("~mega_map", OccupancyGrid, queue_size=1)
        self.input_map_publisher = rospy.Publisher("~input_map", OccupancyGrid, queue_size=1)
        self.spanning_tree_publisher = rospy.Publisher("~spanning_tree", Marker, queue_size=1)

        # subscribers:
        rospy.Subscriber("/clicked_point", PointStamped, self.solve_callback, queue_size=1)
        rospy.Subscriber("/start_exploration", Bool, self.explore_callback, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.explore_callback, queue_size=1)
        rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)

    def solve_callback(self, event):
        self.mutex.acquire()
        rospy.loginfo("Start solving sim_ex_coverage_problem..")
        robot_position = get_transform(self.tf_buffer, self.base_frame, self.map_frame, rospy.Time())
        if self.map.data is None:
            rospy.logerr("Map is not received, abort!")
        elif robot_position is None:
            rospy.logerr("Robot position is not received, abort!")
        else:
            self.planner.solve(robot_position, self.map)
            self.publish_all()
        self.mutex.release()

    def explore_callback(self, event):
        self.mutex.acquire()
        rospy.loginfo("Start exploring..")
        robot_position = get_transform(self.tf_buffer, self.base_frame, self.map_frame, rospy.Time())
        if not self.planner.status_running:
            print("Start solving before exploration, abort!")
        elif self.map.data is None:
            rospy.logerr("Map is not received, abort!")
        elif robot_position is None:
            rospy.logerr("Robot position is not received, abort!")
        else:
            self.planner.explore(robot_position, self.map)
            self.publish_all()
        self.mutex.release()

    def map_callback(self, data: OccupancyGrid):
        self.map = cvt_occupancy2map(data, self.lethal_cost_threshold)

    def publish_all(self):
        self.publish_st_map(self.planner.st_map)
        publish_map(self.map, self.input_map_publisher)
        self.publish_st(self.planner.st_points)
        self.publish_path(self.planner.exploration_path_points, self.planner.full_path_points)

    def publish_st_map(self, st_map: STMap):
        publish_map(st_map.mega, self.mega_map_publisher)
        publish_map(st_map.sub, self.sub_map_publisher)

    def publish_st(self, st_points: list):
        publish_marker("spanning_tree", self.map_frame, st_points, self.spanning_tree_publisher)

    def publish_path(self, exploration_path_points: list, full_path_points: list):
        publish_marker("full_path", self.map_frame, full_path_points, self.full_path_publisher,
                       color=np.array([0.5, 0.5, 1]))
        publish_path(self.map_frame, exploration_path_points, self.exploration_path_publisher)


if __name__ == '__main__':
    try:
        sim_ex_coverage_node = SimExCoverageNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
