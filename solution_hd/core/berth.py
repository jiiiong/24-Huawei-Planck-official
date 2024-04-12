from __future__ import annotations
from typing import List


from log import logger
from cfg import TYPE_CHECKING, N
if TYPE_CHECKING:
    from .env import Env
from path_planing import Point, Point, Boat_Direction

class Berth:
    def __init__(self, berth_id: int, env: Env, x=0, y=0, loading_speed=0):
        self.berth_id: int = berth_id
        self.pos = Point(x, y)
        self.loading_speed = loading_speed
        self.env: Env = env
        #self.gds_priority_queue = PriorityQueue()

        # 统计港口收集的货物用
        self.cur_num_gds = 0
        self.total_num_gds = 0
        self.total_earn = 0

        #self.total_value_of_allocated_goods = 0
        self.robot_cost_grid: List[List[int]]   = []
        self.robot_move_grid: List[List[Point]] = []

        self.increase_rate = 0

        # 该港口的停泊位置
        self.berth_area: List[Point] = []
        # 该港口的靠泊位置
        self.moor_area: List[Point] = []
        # 船在该港口停泊的可能方向，有且只有一种方向
        self.berth_direction = Boat_Direction.ANYWHERE
    
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


    # 通过测试我们知道，当船舶执行berth命令后，船舶的核心点会直接移动到港口的x,y处
    # 方向为当核心点为 x, y 时， 港口刚好能放下船的方向
    # 所以我们使用 berth_direction 记录 船在港口停靠时的方向
    def get_berth_direction(self):
        def can_place_down(x, y):
            if x + 2 >= N or y - 1 < 0 or x + 1 >= N:
                return False
            for p in range(x, x + 2 + 1):
                for q in range(y - 1, y + 1):
                    if self.env.attrs_grid[p][q].is_berth == False:
                        return False
            return True

        def can_place_up(x, y):
            if y + 1 >= N or x - 2 < 0 or x - 1 < 0:
                return False
            for p in range(x - 2, x + 1):
                for q in range(y, y + 1 + 1):
                    if self.env.attrs_grid[p][q].is_berth == False:
                        return False
            return True

        def can_place_left(x, y):
            if x - 1 < 0 or y - 1 < 0 or y - 2 < 0:
                return False
            for p in range(x - 1, x + 1):
                for q in range(y - 2, y + 1):
                    if self.env.attrs_grid[p][q].is_berth == False:
                        return False
            return True

        def can_place_right(x, y):
            if x + 1 >= N or y + 1 >= N or y + 2 >= N:
                return False
            for p in range(x, x + 1 + 1):
                for q in range(y, y + 2 + 1):
                    if self.env.attrs_grid[p][q].is_berth == False:
                        return False
            return True



        if self.env.attrs_grid[self.x][self.y].is_berth:
            if can_place_down(self.x, self.y): self.berth_direction = Boat_Direction.DOWN
            if can_place_right(self.x, self.y): self.berth_direction = Boat_Direction.RIGHT
            if can_place_up(self.x, self.y): self.berth_direction =Boat_Direction.UP
            if can_place_left(self.x, self.y): self.berth_direction = Boat_Direction.LEFT

