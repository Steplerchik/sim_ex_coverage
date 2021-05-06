import numpy as np
import cv2
from libs.structures import MapData, STCell, STMap
from libs.graph import Graph
from libs.conversions import cvt_point2map_point, resize_map, cvt_map_point2point, cvt_direction2angle, cvt_sub2mega, cvt_map2map_to_resize, cvt_input2sub
from libs.conversions import FREE, UNKNOWN, OCCUPIED


class SimExCoveragePlanner(object):
    def __init__(self, robot_size=1.0):
        self._robot_size = robot_size
        self._st_map = STMap()
        self._start_cell = STCell()
        self._st = Graph()
        self._full_path_points = []
        self._full_path_points_tuple = []
        self._exploration_path_points = []
        self._status_started = False
        self._status_running = False
        self._status_finished = False

        self.previous_sub_map_origin = None


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
        while (not ((current_cell.sub == self._start_cell.sub).all() and (tuple(self._start_cell.sub) in self._full_path_points_tuple))) and ((len(self._exploration_path_points) < 5 and (current_cell.mega == cell.mega).all()) or \
            (len(neighbours) == 0 and not (current_cell.sub == self._start_cell.sub).all())):
            neighbours = self.get_neighbours(current_cell.mega, self._st_map.mega)
            neighbours = [x for x in neighbours if self._st_map.mega.data[x[0], x[1]] == UNKNOWN]
            current_cell = self.get_next_cell(current_cell)
            self._exploration_path_points.append(cvt_map_point2point(current_cell.sub, self._st_map.sub))

        exploration_path_points_tuple = [tuple(x) for x in self._exploration_path_points]

        for i in range(len(self._exploration_path_points) - 1):
            self._exploration_path_points[i] = np.append(self._exploration_path_points[i],
                                                         cvt_direction2angle(self._exploration_path_points[i], self._exploration_path_points[i + 1]))
        self._exploration_path_points[-1] = np.append(self._exploration_path_points[-1], self._exploration_path_points[-2][2])
        self._full_path_points.extend(self._exploration_path_points)
        self._full_path_points_tuple.extend(exploration_path_points_tuple)
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
        # self._st_map.sub = cvt_input2sub(map, self._robot_size, int(10 / self._robot_size), int(10 / self._robot_size), np.array([-5, -5, 0]))
        self._st_map.sub = cvt_input2sub(map, self._robot_size, int(20 / self._robot_size), int(10 / self._robot_size), np.array([-4.0, -14.5, 0]))
        self._st_map.mega = cvt_sub2mega(self._st_map.sub)

        # self._st_map.sub = resize_map(map_to_resize, self._robot_size)

        # map_to_resize = cvt_map2map_to_resize(self._st_map.sub, self.previous_sub_map_origin, 2 * self._robot_size)
        # if self.previous_sub_map_origin is not None:
        #     shift_mega = self.previous_sub_map_origin - map_to_resize.origin
        #     mega_res = 2 * self._robot_size
        #     shift_mega_cells_x = int(shift_mega[0] / mega_res)
        #     shift_mega_cells_y = int(shift_mega[1] / mega_res)
        #     sub_res = self._robot_size
        #     shift_sub_cells_x = int(shift_mega[0] / sub_res)
        #     shift_sub_cells_y = int(shift_mega[1] / sub_res)
        #
        #     new_st_nodes = set()
        #     for node in self._st.nodes:
        #         new_node = (node[0] + shift_mega_cells_x, node[1] + shift_mega_cells_y)
        #         # new_st_nodes.add(tuple(map(sum, zip(node, (shift_mega_cells_x, shift_mega_cells_y)))))
        #         new_st_nodes.add(new_node)
        #     self._st.nodes = new_st_nodes
        #
        #     new_st_edges = {}
        #     for key in self._st.edges:
        #         new_edge_list = []
        #         for node in self._st.edges[key]:
        #             new_node = (node[0] + shift_mega_cells_x, node[1] + shift_mega_cells_y)
        #             new_edge_list.append(new_node)
        #         new_key = (key[0] + shift_mega_cells_x, key[1] + shift_mega_cells_y)
        #         new_st_edges[new_key] = new_edge_list
        #     self._st.edges = new_st_edges
        #
        #         # self._st.edges[key][i] = tuple(map(sum, zip(self._st.edges[key][i], (shift_mega_cells_x, shift_mega_cells_y))))
        #         # self._st.edges[key][i] = self._st.edges[key][i]
        #
        #     self._start_cell.mega += np.array([shift_mega_cells_x, shift_mega_cells_y])
        #     self._start_cell.sub += np.array([shift_sub_cells_x, shift_sub_cells_y])
        #
        # self.previous_sub_map_origin = map_to_resize.origin
        # self._st_map.mega = cvt_sub2mega(map_to_resize)





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
