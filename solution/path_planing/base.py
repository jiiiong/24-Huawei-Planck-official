from __future__ import annotations
from enum import Enum
from dataclasses import dataclass
from cfg import N
Nm1 = N - 1

@dataclass
class Pixel_Attrs():
    is_ground: bool = False
    is_ocean: bool= False
    is_free_ground: bool = False  
    is_free_ocean: bool = False


class Point():
    def __init__(self, x = 0, y = 0) -> None:
        self.x = x
        self.y = y

    def __add__(self, b: Point):
        return Point(self.x + b.x, self.y + b.y)
    
    def __sub__(self, b: Point):
        return Point(self.x - b.x, self.y - b.y)
    
    def is_in_grid(self) -> bool:
        return (0<=self.x and self.x<N) and (0<=self.y and self.y<N)
    
    def __eq__(self, b):
        return self.x == b.x and self.y == b.y
    def __hash__(self) -> int:
        return hash((self.x, self.y))
    def __repr__(self) -> str:
        return f"({self.x}, {self.y})"
    def __neg__(self):
        return Point(-self.x, -self.y)
    def __lt__(self, b):
        return True
    def distance(self, b: Point):
        return abs(self.x - b.x) + abs(self.y - b.y)


    
INFINIT_COST = (2**31) -1
UNREACHABLE_POS = Point(-2, -2)

class Robot_Move():
    UP = Point(-1, 0)
    DOWN = Point(1, 0)
    LEFT = Point(0, -1)
    RIGHT = Point(0, 1)
    HOLD = Point(0, 0)

    BAD_MOVE = Point(-2, -2)

    def __iter__(self):
        yield self.UP
        yield self.DOWN
        yield self.LEFT
        yield self.RIGHT
        yield self.HOLD

class Boat_Action():
    SHIP = "ship"
    ROTATE1 = "rotate1"
    ROTATE0 = "rotate0"
    STAY = "stay"

class Boat_Direction(Enum):
    RIGHT = 0
    LEFT = 1
    UP = 2
    DOWN = 3

class sVec():
    def __init__(self, pos: Point = Point(), dir: Boat_Direction = Boat_Direction.RIGHT):
        self.pos = pos
        self.dir = dir

    def __hash__(self) -> int:
        return hash((self.pos, self.dir))
    def __eq__(self, value) -> bool:
        return (self.pos == value.pos and self.dir == value.dir)
    def __lt__(self, b):
        return True
    
    @property
    def dir(self):
        return self._dir
    @dir.setter
    def dir(self, value):
        if isinstance(value, int):
            self._dir = Boat_Direction(value)
        elif isinstance(value, Boat_Direction):
            self._dir = value
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