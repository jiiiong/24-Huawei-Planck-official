from typing import List

from path_planing import Point, UNREACHABLE_POS

class Goods:
    def __init__(self, gen_zhen:int, global_zhen_ref: List[int], pos: Point, price: int):
        self.pos = pos
        self.price = price

        self.cost = -1
        self.fetched = False

        self.gen_zhen = gen_zhen
        self.global_zhen_ref = global_zhen_ref
    @property
    def elapsed_zhen(self):
        return self.global_zhen_ref[0] - self.gen_zhen
    
    @property
    def left_zhen(self):
        return 1000 - self.global_zhen_ref[0] + self.gen_zhen

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

    def __repr__(self) -> str:
        return f"gds: {self.pos} {self.price}"
    
    def __lt__(self, b):
        return True