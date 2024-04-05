from __future__ import annotations
from typing import List


from log import logger
from cfg import TYPE_CHECKING
if TYPE_CHECKING:
    from .env import Env
from path_planing import Point, Point

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

