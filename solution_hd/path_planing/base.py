from __future__ import annotations
from dataclasses import dataclass
from cfg import N
Nm1 = N - 1

@dataclass
class Pixel_Attrs():
    is_ground: bool = False
    is_ocean: bool= False
    is_free_ground: bool = False  
    is_free_ocean: bool = False

    is_berth: bool = False                  # 该像素点是否为港口
    is_purchase_ship_position: bool = False    # 该像素点是否为 轮船生成点
    is_main_channel: bool = False           # 是否为主航道
    is_target: bool = False                 # 交货点
    is_mooring: bool = False                # 靠泊区





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

class Boat_Direction():
    DOWN = "D"
    RIGHT = "R"
    UP = "U"
    LEFT = "L"
    ANYWHERE = "A"
