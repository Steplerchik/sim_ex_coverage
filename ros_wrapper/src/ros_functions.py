import rospy
from structures import MapData
from ros_conversions import cvt_map2occupancy, cvt_points2marker, cvt_points2path
import numpy as np


def publish_map(map: MapData, publisher: rospy.Publisher):
    map_msg = cvt_map2occupancy(map)
    publisher.publish(map_msg)


def publish_marker(name: str, frame_id: str, points: list, publisher: rospy.Publisher, color=np.array([0, 0, 0]), scale=0.1):
    marker_msg = cvt_points2marker(name, frame_id, points, color, scale)
    publisher.publish(marker_msg)


def publish_path(frame_id: str, points: list, publisher: rospy.Publisher):
    path_msg = cvt_points2path(frame_id, points)
    publisher.publish(path_msg)
