from typing import Tuple, List

from cfg import TYPE_CHECKING
from log import main_logger
from path_planing import Point, sVec
from path_planing import Boat_Direction

if TYPE_CHECKING:
    from .env import Env

int_boatDirection_map = {
    0: Boat_Direction.RIGHT,
    1: Boat_Direction.LEFT,
    2: Boat_Direction.UP,
    3: Boat_Direction.DOWN
}


class Boat:
    def __init__(self, boat_id, env: 'Env', x=0, y=0, dir=0, num=0, status=0):
        self.boat_id = boat_id
        self.env: 'Env' = env
        self.goods_num = num
        self.pos = Point(x,y)
        self._dir = int_boatDirection_map[dir]
        self.status = status
        self.actions: List[str] = []

    @property
    def dir(self):
        return self._dir
    @dir.setter
    def dir(self, value: int):
        self._dir = int_boatDirection_map[value]

    @property
    def sVec(self):
        return sVec(self.pos, self.dir)

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

    def ship_from_A_to_B(self, start_sVec: 'sVec', end_sVec: 'sVec'):
        if start_sVec == self.sVec:
            if ((start_sVec, end_sVec) in self.env.boat_route_dict):
                self.actions = self.env.boat_route_dict[(start_sVec, end_sVec)]
            else:
                main_logger.error(f"boat{self.boat_id} 尝试在 {self.sVec} 从 {start_sVec} 出发到 {end_sVec}，路径不存在")
        else:
            main_logger.error(f"boat{self.boat_id} 尝试在 {self.sVec} 从 {start_sVec} 出发到 {end_sVec}，初始位置错误")
    
    def boat_execute(self):
        if self.status == 0:
            if len(self.actions) > 0:
                action = self.actions.pop(0)
                if action == "ship":
                    print("ship", self.boat_id)
                elif action == "rot 0":
                    print("rot", self.boat_id, 0)
                elif action == "rot 1":
                    print("rot", self.boat_id, 1)

