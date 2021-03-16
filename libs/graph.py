import numpy as np
from structures import MapData
from conversions import cvt_map_point2point

class Graph(object):
    def __init__(self):
        self.nodes = set()
        self.edges = {}
        self.points = []

    def add_edge(self, parent: np.array, child: np.array, map: MapData):
        self.nodes.add(tuple(parent))
        self.nodes.add(tuple(child))
        self.edges.setdefault(tuple(parent),[]).append(tuple(child))
        self.edges.setdefault(tuple(child),[]).append(tuple(parent))
        self.points.append(cvt_map_point2point(parent, map))
        self.points.append(cvt_map_point2point(child, map))