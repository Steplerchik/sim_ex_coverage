import numpy as np
import cv2
from libs.structures import MapData, STCell, STMap
from libs.graph import Graph
from libs.conversions import cvt_point2map_point, resize_map, cvt_map_point2point, cvt_direction2angle, cvt_sub2mega
from libs.conversions import FREE, UNKNOWN, OCCUPIED


class SimExCoveragePlanner(object):
    def __init__(self, robot_size=1.0):
        self._robot_size = robot_size
        self._st_map = STMap()
        self._start_cell = STCell()
        self._st = Graph()
        self._full_path_points = []
        self._exploration_path_points = []
        self._status_started = False
        self._status_running = False
        self._status_finished = False


    def solve(self, robot_position: np.array, map: MapData):
        self._status_started = True
        self._status_running = True
        self._status_finished = False
        self.update_st_map(map)
        self._start_cell = self.get_st_cell(robot_position, self._st_map)
        self._st = Graph()
        self._full_path_points = []
        try:
            self.explore(robot_position, map)
        except Exception as ex:
            print(ex)

    def explore(self, robot_position: np.array, map: MapData):
        self.update_st_map(map)
        cell = self.get_st_cell(robot_position, self._st_map)
        self.update_st(cell)
        self.update_path(cell, robot_position)

    def update_st(self, cell: STCell):
        self.visit(cell.mega, self._st_map.mega)

    def update_path(self, cell: STCell, robot_position: np.array):
        current_cell = cell
        self._exploration_path_points = [robot_position[:2], cvt_map_point2point(current_cell.sub, self._st_map.sub)]
        neighbours = []
        while (len(self._exploration_path_points) < 5 and (current_cell.mega == cell.mega).all()) or \
            (len(neighbours) == 0 and not (current_cell.sub == self._start_cell.sub).all()):
            neighbours = self.get_neighbours(current_cell.mega, self._st_map.mega)
            neighbours = [x for x in neighbours if self._st_map.mega.data[x[0], x[1]] == UNKNOWN]
            current_cell = self.get_next_cell(current_cell)
            self._exploration_path_points.append(cvt_map_point2point(current_cell.sub, self._st_map.sub))

        for i in range(len(self._exploration_path_points) - 1):
            self._exploration_path_points[i] = np.append(self._exploration_path_points[i],
                                                         cvt_direction2angle(self._exploration_path_points[i], self._exploration_path_points[i + 1]))
        self._exploration_path_points[-1] = np.append(self._exploration_path_points[-1], self._exploration_path_points[-2][2])
        self._full_path_points.extend(self._exploration_path_points)
        if (current_cell.sub == self._start_cell.sub).all():
            self._status_started = False
            self._status_running = False
            self._status_finished = True
            print("Solving sim_ex_coverage_problem is DONE!")

    def visit(self, map_point: np.array, map: MapData):
        neighbours = self.get_neighbours(map_point, map)
        for neighbour_point in neighbours:
            if tuple(neighbour_point) not in self._st.nodes and map.data[neighbour_point[0], neighbour_point[1]] == FREE:
                self._st.add_edge(map_point, neighbour_point, map)
                self.visit(neighbour_point, map)

    def get_neighbours(self, map_point: np.array, map: MapData):
        neighbours = map_point + np.array([[0, 1], [1, 0], [-1, 0], [0, -1]])
        neighbours = np.array([x for x in neighbours if self.map_point_in_map(x, map)])
        return neighbours

    def map_point_in_map(self, map_point, map):
        return 0 <= map_point[0] < map.w and 0 <= map_point[1] < map.h

    def get_st_cell(self, robot_position: np.array, st_map: STMap) -> STCell:
        cell = STCell()
        cell.mega = cvt_point2map_point(robot_position, st_map.mega)
        cell.sub = cvt_point2map_point(robot_position, st_map.sub)
        return cell

    def update_st_map(self, map: MapData):
        self._st_map.sub = resize_map(map, self._robot_size)
        self._st_map.mega = cvt_sub2mega(self._st_map.sub)
        # self._st_map.mega = resize_map(self._st_map.sub, 2 * self._robot_size, interpolation=cv2.INTER_NEAREST)

    def get_next_cell(self, cell: STCell) -> STCell:
        sub_cells = [(1 + cell.mega) * 2 - 1,
                     (1 + cell.mega) * 2 - 1 - np.array([1, 0]),
                     (1 + cell.mega) * 2 - 1 - np.array([1, 1]),
                     (1 + cell.mega) * 2 - 1 - np.array([0, 1])]

        index = [tuple(x) for x in sub_cells].index(tuple(cell.sub))

        st_directions = np.array([[0, 1], [-1, 0], [0, -1], [1, 0]])
        mega_neighbours = [tuple(x) for x in list(cell.mega + st_directions)]

        cell_directions = np.array([[-1, 0], [0, -1], [1, 0], [0, 1]])

        next_cell = STCell()
        if tuple(cell.mega) in self._st.edges and mega_neighbours[index] in self._st.edges[tuple(cell.mega)]:
            next_cell.sub = cell.sub + st_directions[index]
            next_cell.mega = cell.mega + st_directions[index]
        else:
            next_cell.sub = cell.sub + cell_directions[index]
            next_cell.mega = cell.mega
        return next_cell

    @property
    def st_map(self):
        return self._st_map

    @property
    def st_points(self):
        return self._st.points

    @property
    def full_path_points(self):
        return self._full_path_points

    @property
    def exploration_path_points(self):
        return self._exploration_path_points

    @property
    def status_running(self):
        return self._status_running
