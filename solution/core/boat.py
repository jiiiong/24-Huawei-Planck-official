from __future__ import annotations
from typing import Tuple, List, Set
from queue import Queue
from copy import deepcopy

from cfg import TYPE_CHECKING
from log import main_logger, boat_logger
from path_planing import Point, sVec
from path_planing import Boat_Direction, boat_actions_to_poses, boat_next_sVec

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
        self._dir: Boat_Direction = int_boatDirection_map[dir]
        self.status = status
        self.actions: List[str] = []

    @property
    def dir(self):
        return self._dir
    @dir.setter
    def dir(self, value):
        if isinstance(value, int):
            self._dir = int_boatDirection_map[value]
        elif isinstance(value, Boat_Direction):
            self._dir = value

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
                self.actions = deepcopy(self.env.boat_route_dict[(start_sVec, end_sVec)])
            else:
                main_logger.error(f"boat{self.boat_id} 尝试在 {self.sVec} 从 {start_sVec} 出发到 {end_sVec}，路径不存在")
        else:
            main_logger.error(f"boat{self.boat_id} 尝试在 {self.sVec} 从 {start_sVec} 出发到 {end_sVec}，初始位置错误")
    
    def boat_execute(self):
        if self.status == 0:
            if len(self.actions) > 0 and self.action_valid_check(self.actions[0]):
                action = self.actions.pop(0)
                if action == "ship":
                    print("ship", self.boat_id)
                elif action == "rot 0":
                    print("rot", self.boat_id, 0)
                elif action == "rot 1":
                    print("rot", self.boat_id, 1)

    def action_valid_check(self, action: str):
        okk, cur_owned_locks, next_owned_locks = self.lock_check(action)
        boat_logger.error(f"{self.boat_id}, {self.sVec},{okk} {cur_owned_locks} {next_owned_locks}")
        if okk:
            if self.topology_graph_valid_check(action):
                if self.direction_valid_check(action):
                    self.get_lock(cur_owned_locks, next_owned_locks)
                    return True
        return False

    def lock_check(self, action: str) -> Tuple[bool, Set[int], Set[int]]: 
        cur_owned_locks = set()
        lt_pos, rb_pos = self.sVec.proj()
        for x in range(lt_pos.x, rb_pos.x+1):
            for y in range(lt_pos.y, rb_pos.y+1):
                cur_owned_locks.add(self.env.lock_grid[x][y].unique_route_id)

        next_sVec = boat_next_sVec(self.sVec, action)
        next_owned_locks = set()
        lt_pos, rb_pos = next_sVec.proj()
        for x in range(lt_pos.x, rb_pos.x+1):
            for y in range(lt_pos.y, rb_pos.y+1):
                if (self.env.lock_grid[x][y].unique_route_id not in cur_owned_locks):
                    if self.env.lock_dict[self.env.lock_grid[x][y].unique_route_id].check_available() == False:
                        return False, cur_owned_locks, next_owned_locks
                    else:
                        next_owned_locks.add(self.env.lock_grid[x][y].unique_route_id)
                else:
                    next_owned_locks.add(self.env.lock_grid[x][y].unique_route_id)

        return True, cur_owned_locks, next_owned_locks

    def get_lock(self, cur_owned_locks: Set[int], next_owned_locks: Set[int]):
        # boat_logger.info(f"{self.boat_id} release lock_id {cur_owned_locks - next_owned_locks}")
        # boat_logger.info(f"{self.boat_id} get lock_id {next_owned_locks - cur_owned_locks}")

        for id in cur_owned_locks:
            self.env.lock_dict[id].locked = False
        for id in next_owned_locks:
            self.env.lock_dict[id].locked = True

    def topology_graph_valid_check(self, action: str):
        return True

    def direction_valid_check(self, action: str):
        return True

class Boat_Route_Lock:

    def __init__(self) -> None:
        self.route_id: str = "A"
        self.unique_route_id: int = -1
        self.times: int = 0
        self.num_shared_roads: int = 0
        self.locked: bool = False
        self.acquire_lock_queue = Queue()

    def __str__(self):
        return str(self.unique_route_id)

    def update(self, unique_route_id: str):
        self.route_id = self.route_id + unique_route_id
        self.num_shared_roads += 1

    def check_available(self):
        return (not self.locked)
    
    def release_lock(self):
        self.locked = False