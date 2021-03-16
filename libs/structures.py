class MapData(object):
    def __init__(self):
        self.data = None
        self.w = None
        self.h = None
        self.res = None
        self.origin = None
        self.frame_id = None

class STCell(object):
    def __init__(self):
        self.mega = None
        self.sub = None

class STMap(object):
    def __init__(self):
        self.mega = MapData()
        self.sub = MapData()
