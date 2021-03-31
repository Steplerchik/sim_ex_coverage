from geometry_msgs.msg import Pose, Transform, Quaternion, Point, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, OccupancyGrid
import tf_conversions
import tf2_ros
import numpy as np
import rospy
from libs.structures import MapData
from libs.conversions import FREE, OCCUPIED, UNKNOWN

def cvt_angle2quaternion(angle: np.array) -> Quaternion:
    quaternion = Quaternion()
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, angle)
    quaternion.x = q[0]
    quaternion.y = q[1]
    quaternion.z = q[2]
    quaternion.w = q[3]
    return quaternion


def cvt_quaternion2angle(quaternion: Quaternion) -> float:
    q = [quaternion.x,
         quaternion.y,
         quaternion.z,
         quaternion.w]
    _, _, angle = tf_conversions.transformations.euler_from_quaternion(q)
    return angle


def cvt_ros_pose2point(pose: Pose) -> np.array:
    x = pose.position.x
    y = pose.position.y
    a = cvt_quaternion2angle(pose.orientation)
    return np.array([x, y, a])


def cvt_point2ros_pose(point: np.array) -> Pose:
    pose = Pose()
    pose.position.x = point[0]
    pose.position.y = point[1]
    pose.orientation = cvt_angle2quaternion(point[2])
    return pose


def cvt_ros_transform2point(transform: Transform) -> np.array:
    x = transform.translation.x
    y = transform.translation.y
    a = cvt_quaternion2angle(transform.rotation)
    return np.array([x, y, a])


def get_transform(buffer_, child_frame, parent_frame, stamp):
    try:
        t = buffer_.lookup_transform(parent_frame, child_frame, stamp)
        return cvt_ros_transform2point(t.transform)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
        rospy.logwarn(str(msg))
        return None


def cvt_point2ros_transform(point: np.array) -> Transform:
    transform = Transform()
    transform.translation.x = point[0]
    transform.translation.y = point[1]
    transform.rotation = cvt_angle2quaternion(point[2])
    return transform


def cvt_map2occupancy(map: MapData) -> OccupancyGrid:
    map_msg = OccupancyGrid()
    map_msg.info.resolution = map.res
    map_msg.info.width = map.w
    map_msg.info.height = map.h
    map_msg.info.origin = cvt_point2ros_pose(map.origin)
    map_msg.header.frame_id = map.frame_id
    map_msg.header.stamp = rospy.Time.now()
    map_msg.data = tuple(map.data.astype('int').T.reshape(map.h * map.w))
    return map_msg


def cvt_occupancy2map(map_msg: OccupancyGrid, lethal_cost_threshold: int) -> MapData:
    map_data = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width).T
    map_data[(map_data < lethal_cost_threshold) * (map_data >= 0)] = FREE
    map_data[map_data >= lethal_cost_threshold] = OCCUPIED
    map_data[map_data == -1] = UNKNOWN
    map = MapData()
    map.data = map_data
    map.w = map_msg.info.width
    map.h = map_msg.info.height
    map.res = map_msg.info.resolution
    map.origin = cvt_ros_pose2point(map_msg.info.origin)
    map.frame_id = map_msg.header.frame_id
    return map


def cvt_points2marker(name: str, frame_id: str, points: list, color=np.array([0, 0, 0]), scale=0.1, type=Marker.LINE_STRIP) -> Marker:
    marker_msg = Marker()
    marker_msg.header.frame_id = frame_id
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.ns = name
    marker_msg.id = 0
    marker_msg.type = type
    marker_msg.action = 0
    marker_msg.scale.x = scale
    marker_msg.color.r = color[0]
    marker_msg.color.g = color[1]
    marker_msg.color.b = color[2]
    marker_msg.color.a = 1.0
    marker_msg.frame_locked = False
    marker_points = []
    for point in points:
        new_point = Point()
        new_point.x = point[0]
        new_point.y = point[1]
        marker_points.append(new_point)
    marker_msg.points = marker_points
    return marker_msg


def cvt_points2path(frame_id: str, points: list) -> Path:
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    path_msg.header.stamp = rospy.Time.now()
    for point in points:
        pose = PoseStamped()
        pose.pose = cvt_point2ros_pose(point)
        path_msg.poses.append(pose)
    return path_msg
