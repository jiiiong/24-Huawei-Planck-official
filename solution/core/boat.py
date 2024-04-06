from path_planing import Point
from path_planing import Boat_Direction

cmd_boatDirection_map = {
    0: Boat_Direction.RIGHT,
    1: Boat_Direction.LEFT,
    2: Boat_Direction.UP,
    3: Boat_Direction.DOWN
}

class Boat:
    def __init__(self, boat_id=0, x=0, y=0, dir=0, num=0, status=0):
        self.boat_id = boat_id
        self.goods_num = num
        self.pos = Point(x,y)
        self._dir = cmd_boatDirection_map[dir]
        self.status = status

    @property
    def dir(self):
        return self._dir
    
    @dir.setter
    def dir(self, value: int):
        self._dir = cmd_boatDirection_map[value]

    @property
    def sVec(self):
        return (self.pos, self.dir)

    @property
    def x(self):
        return self.pos.x
    @x.setter
    def x(self, value):
        self.pos.x = value
    @property
    def y(self):
        return self.pos.y
    @y.setter
    def y(self, value):
        self.pos.y = value

def transfer_direction(boat_dir):
    if boat_dir == 0:
        return Boat_Direction.RIGHT
    elif boat_dir == 1:
        return Boat_Direction.LEFT
    elif boat_dir == 2:
        return Boat_Direction.UP
    elif boat_dir == 3:
        return Boat_Direction.DOWN
    else:
        return ""

