import numpy as np
from libs.structures import MapData
import cv2
from math import ceil, atan2

FREE = 0
OCCUPIED = 100
UNKNOWN = 120


def cvt_local2global(local_point: np.array, src_point: np.array):
    """
    Convert points from local frame to global
    :param local_point: A local point or array of local points that must be converted 1-D np.array or 2-D np.array
    :param src_point: A
    :return:
    """
    size = local_point.shape[-1]
    x, y, a = 0, 0, 0
    if size == 3:
        x, y, a = local_point.T
    elif size == 2:
        x, y = local_point.T
    X, Y, A = src_point.T
    x1 = x * np.cos(A) - y * np.sin(A) + X
    y1 = x * np.sin(A) + y * np.cos(A) + Y
    a1 = (a + A + np.pi) % (2 * np.pi) - np.pi
    if size == 3:
        return np.array([x1, y1, a1]).T
    elif size == 2:
        return np.array([x1, y1]).T
    else:
        return


def cvt_global2local(global_point: np.array, src_point: np.array):
    size = global_point.shape[-1]
    x1, y1, a1 = 0, 0, 0
    if size == 3:
        x1, y1, a1 = global_point.T
    elif size == 2:
        x1, y1 = global_point.T
    X, Y, A = src_point.T
    x = x1 * np.cos(A) + y1 * np.sin(A) - X * np.cos(A) - Y * np.sin(A)
    y = -x1 * np.sin(A) + y1 * np.cos(A) + X * np.sin(A) - Y * np.cos(A)
    a = (a1 - A + np.pi) % (2 * np.pi) - np.pi
    if size == 3:
        return np.array([x, y, a]).T
    elif size == 2:
        return np.array([x, y]).T
    else:
        return


def cvt_point2map_point(point, map: MapData) -> np.array:
    map_point = cvt_global2local(point[:2], map.origin)
    map_point = np.array([int(map_point[0] / map.res), int(map_point[1] / map.res)])
    if 0 <= map_point[0] < map.w and 0 <= map_point[1] < map.h:
        return map_point
    else:
        return None

def cvt_map_point2point(map_point: np.array, map: MapData) -> np.array:
    point = map_point * map.res + np.ones(2) * map.res / 2.
    return cvt_local2global(point, map.origin)


def resize_map(map: MapData, target_res: float) -> MapData:
    resized_map = MapData()
    resized_map.res = target_res
    resized_map.h = ceil(float(map.h * map.res) / target_res)
    resized_map.w = ceil(float(map.w * map.res) / target_res)

    diff_h = ceil(float(resized_map.h) * resized_map.res / map.res) - map.h
    diff_w = ceil(float(resized_map.w) * resized_map.res / map.res) - map.w

    map_data = map.data

    for i in range(diff_w):
        map_data = np.vstack((map_data, map_data[-1, :][np.newaxis]))
    for j in range(diff_h):
        map_data = np.hstack((map_data, map_data[:, -1][np.newaxis].T))

    resized_map.origin = map.origin
    resized_map.data = cv2.resize(map_data.astype(np.uint8), (resized_map.h, resized_map.w), interpolation=cv2.INTER_LINEAR)
    # resized_map.data = cv2.resize(map_data.astype(np.uint8), (resized_map.h, resized_map.w), interpolation=cv2.INTER_AREA)
    resized_map.data[(resized_map.data < (FREE + OCCUPIED) / 2) * (resized_map.data >= FREE)] = FREE
    resized_map.data[(resized_map.data >= (FREE + OCCUPIED) / 2) * (resized_map.data <= (OCCUPIED + UNKNOWN) / 2)] = OCCUPIED
    resized_map.data[resized_map.data > (OCCUPIED + UNKNOWN) / 2] = UNKNOWN
    resized_map.frame_id = map.frame_id
    return resized_map

def cvt_sub2mega(sub_map: MapData) -> MapData:
    mega_map = MapData()
    mega_map.res = sub_map.res * 2
    mega_map.h = int(sub_map.h / 2) + sub_map.h % 2
    mega_map.w = int(sub_map.w / 2) + sub_map.w % 2
    mega_map.origin = sub_map.origin
    mega_map.frame_id = sub_map.frame_id

    map_data = sub_map.data
    if sub_map.w % 2 != 0:
        map_data = np.vstack((map_data, map_data[-1, :][np.newaxis]))
    if sub_map.h % 2 != 0:
        map_data = np.hstack((map_data, map_data[:, -1][np.newaxis].T))

    temp = np.maximum(map_data[::2, ::2], map_data[1::2, 1::2])
    temp = np.maximum(temp, map_data[::2, 1::2])
    mega_map.data = np.maximum(temp, map_data[1::2, ::2])
    return mega_map

def cvt_direction2angle(begin:np.array, end:np.array) -> float:
    direction = end - begin
    return atan2(direction[1], direction[0])
