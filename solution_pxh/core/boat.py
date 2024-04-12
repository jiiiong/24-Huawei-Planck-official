from __future__ import annotations
from typing import Tuple, List, Set, Dict
from queue import Queue
from copy import deepcopy

from cfg import TYPE_CHECKING
from log import main_logger, boat_logger
from path_planing import Point, sVec
from path_planing import Boat_Direction, boat_actions_to_poses, boat_next_sVec, Boat_Ship_Offset, rotate_one_move_no_test

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
        self.last_action: str = ''
        self.supposed_sVec:sVec = self.sVec
        self.last_locks_id_set: Set[int] = set()
        
        self.route_allocated: bool = False
        self.checked_run: bool = False

        self.route_id: Tuple[sVec, sVec]
        self.required_lock_ids: List[int] = []
        # self.my_locks: Set[int] = set()
        self.my_locks: Dict[int, int] = {}

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

    @property
    def next_sVec(self):
        if len(self.actions) > 0:
            action = self.actions[0]
            if action == "ship":
                cur_sVec = sVec(self.pos + Boat_Ship_Offset[self.dir], self.dir)
            elif action == "rot 0":
                cur_sVec = rotate_one_move_no_test(self.sVec, 0)
            elif action == "rot 1":
                cur_sVec = rotate_one_move_no_test(self.sVec, 1)
            else:
                cur_sVec = self.sVec
            return cur_sVec
        else:
            return self.sVec

    def ship_from_A_to_B(self, start_sVec: 'sVec', end_sVec: 'sVec'):
        if start_sVec == self.sVec:
            if ((start_sVec, end_sVec) in self.env.boat_route_dict):
                self.actions = deepcopy(self.env.boat_route_dict[(start_sVec, end_sVec)])
                self.route_id = (start_sVec, end_sVec)
                self.route_allocated = True
            else:
                main_logger.error(f"boat{self.boat_id} 尝试在 {self.sVec} 从 {start_sVec} 出发到 {end_sVec}，路径不存在")
        else:
            main_logger.error(f"boat{self.boat_id} 尝试在 {self.sVec} 从 {start_sVec} 出发到 {end_sVec}，初始位置错误")
    
    def collision_recovery(self):
        if self.sVec != self.supposed_sVec:
            self.actions.insert(0, self.last_action)
        # boat_logger.error(f"{self.boat_id}, {self.last_locks_id_set}")
        # 如果任务还未完成，即action还有东西
        if self.route_allocated == True:
            if  len(self.actions) > 0 and self.checked_run:
                # 需要释放已经走过的路段
                cur_locks_id_set: Set[int] = set()
                lt_pos, rb_pos = self.sVec.proj()
                for x in range(lt_pos.x, rb_pos.x+1):
                    for y in range(lt_pos.y, rb_pos.y+1):
                        cur_locks_id_set.add(self.env.lock_grid[x][y].unique_route_id)
                boat_logger.error(f"{self.boat_id} {self.last_locks_id_set}, {cur_locks_id_set}")
                for lock_id in self.last_locks_id_set:
                    if lock_id not in cur_locks_id_set:
                        if 1:#self.env.lock_dict[lock_id].free_lock is False:
                            self.my_locks[lock_id] -= 1
                            if self.my_locks[lock_id] <= 0:
                                self.env.lock_dict[lock_id].release_lock()
                            # self.my_locks.remove(lock_id)
                self.last_locks_id_set = cur_locks_id_set
            # 结束一次任务
            elif len(self.actions) == 0:
                self.route_allocated = False
                self.checked_run = False

                cur_locks_id_set: Set[int] = set()
                lt_pos, rb_pos = self.sVec.proj()
                for x in range(lt_pos.x, rb_pos.x+1):
                    for y in range(lt_pos.y, rb_pos.y+1):
                        cur_locks_id_set.add(self.env.lock_grid[x][y].unique_route_id)
                # boat_logger.error(f"{self.boat_id} {self.last_locks_id_set}, {cur_locks_id_set}")
                for lock_id in self.last_locks_id_set:
                    if lock_id not in cur_locks_id_set:
                        if 1:#self.env.lock_dict[lock_id].free_lock is False:
                            self.my_locks[lock_id] -= 1
                            if self.my_locks[lock_id] <= 0:
                                self.env.lock_dict[lock_id].release_lock()
                            
                            # self.my_locks.remove(lock_id)
                self.last_locks_id_set = cur_locks_id_set
            # self.release_all_locks()
            # lt_pos, rb_pos = self.sVec.proj()
            # for x in range(lt_pos.x, rb_pos.x+1):
            #     for y in range(lt_pos.y, rb_pos.y+1):
            #         self.env.lock_grid[x][y].get_lock(self.boat_id)
        # boat_logger.error(f"{self.boat_id} {self.my_locks}")

    def boat_execute(self):
        # 当前能够运行
        if self.status == 0 or self.status == 2:
            # 存在任务
            if self.route_allocated:
                boat_logger.debug(f"{self.boat_id} {self.my_locks}")
                # 如果还没检查过路径是否能够分配，则检查
                if not self.checked_run:
                    # 如果检查能够并获取所需要的所有lock。则则表示检查完成
                    if self.action_valid_check_once(self.actions[0]):
                        self.checked_run = True
                # 如果已经获取所有的锁
                if self.checked_run:
                    self.supposed_sVec = self.next_sVec
                    action = self.actions.pop(0)
                    if action == "ship":
                        print("ship", self.boat_id)
                    elif action == "rot 0":
                        print("rot", self.boat_id, 0)
                    elif action == "rot 1":
                        print("rot", self.boat_id, 1)
                    else:
                        pass
                    self.last_action = action

    def action_valid_check_once(self, action: str):
        okk = self.all_lock_check()
        if okk:
            okk = self.collision_check()
            if okk:
                self.get_all_locks()
        return okk
        okk, cur_owned_locks, next_owned_locks = self.lock_check(action)
        boat_logger.error(f"{self.boat_id}, {self.sVec},{okk} {cur_owned_locks} {next_owned_locks}")
        if okk:
            if self.topology_graph_valid_check(action):
                if self.direction_valid_check(action):
                    self.get_lock(cur_owned_locks, next_owned_locks)
                    return True
        return False

    def collision_check(self):
        return True
        for boat in self.env.boats:
            if boat.boat_id != self.boat_id:
                if boat.pos.distance(self.pos) <= 10:
                    lt_pos_self, rb_pos_self = self.sVec.proj()
                    
                    lt_pos_oth, rb_pos_oth = boat.sVec.proj()
                    horizontal_overlap = 1

    def all_lock_check(self) -> bool: 
        '''检查是否能够获得所有lock，保证进入前route_id已经设置'''
        for lock_id in self.env.boat_route_all_locks_id_map[self.route_id]:
            if self.env.lock_dict[lock_id].check_available(self.boat_id) == False:
                return False
        return True
    
    def get_all_locks(self): 
        '''检查是否能够获得所有lock，保证进入前route_id已经设置'''
        self.my_locks = deepcopy(self.env.boat_route_all_locks_id_map[self.route_id])
        for lock_id in self.env.boat_route_all_locks_id_map[self.route_id]:
            
            self.env.lock_dict[lock_id].get_lock(self.boat_id)
            # self.my_locks.add(lock_id)
            
            # self.last_locks_id_set = set()
            # lt_pos, rb_pos = self.sVec.proj()
            # for x in range(lt_pos.x, rb_pos.x+1):
            #     for y in range(lt_pos.y, rb_pos.y+1):
            #         self.last_locks_id_set.add(self.env.lock_grid[x][y].unique_route_id)

    def release_all_locks(self): 
        '''检查是否能够获得所有lock，保证进入前route_id已经设置'''
        for lock_id in self.my_locks:
            self.env.lock_dict[lock_id].release_lock()
        self.my_locks = {}

    def lock_check(self, action: str) -> Tuple[bool, Set[int], Set[int]]: 
        '''检查是否能够获得lock'''
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
                    if self.env.lock_dict[self.env.lock_grid[x][y].unique_route_id].check_available(self.boat_id) == False:
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
        self.free_lock: bool = False
        self.acquire_lock_queue = Queue()
        self.cur_owner: int = -1

    def __str__(self):
        return str(self.unique_route_id)

    def update(self, unique_route_id: str):
        self.route_id = self.route_id + unique_route_id
        self.num_shared_roads += 1

    def check_available(self, boat_id: int):
        if self.cur_owner == boat_id:
            return True
        else:
            return (not self.locked) or (self.free_lock)
    
    def release_lock(self):
        self.locked = False

    def get_lock(self, boat_id):
        if (self.check_available(boat_id)):
            self.locked = True
            self.cur_owner = boat_id


# from typing import Tuple, List
# from copy import deepcopy

# from cfg import TYPE_CHECKING
# from log import main_logger
# from path_planing import Point, sVec
# from path_planing import Boat_Direction

# if TYPE_CHECKING:
#     from .env import Env

# int_boatDirection_map = {
#     0: Boat_Direction.RIGHT,
#     1: Boat_Direction.LEFT,
#     2: Boat_Direction.UP,
#     3: Boat_Direction.DOWN
# }


# class Boat:
#     def __init__(self, boat_id, env: 'Env', x=0, y=0, dir=0, num=0, status=0):
#         self.boat_id = boat_id
#         self.env: 'Env' = env
#         self.goods_num = num
#         self.pos = Point(x,y)
#         self._dir: Boat_Direction = int_boatDirection_map[dir]
#         self.status = status
#         self.actions: List[str] = []

#     @property
#     def dir(self):
#         return self._dir
#     @dir.setter
#     def dir(self, value):
#         if isinstance(value, int):
#             self._dir = int_boatDirection_map[value]
#         elif isinstance(value, Boat_Direction):
#             self._dir = value

#     @property
#     def sVec(self):
#         return sVec(self.pos, self.dir)

#     @property
#     def x(self):
#         return self.pos.x
#     @x.setter
#     def x(self, value):
#         self.pos.x = value
#     @property
#     def y(self):
#         return self.pos.y
#     @y.setter
#     def y(self, value):
#         self.pos.y = value

#     def ship_from_A_to_B(self, start_sVec: 'sVec', end_sVec: 'sVec'):
#         if start_sVec == self.sVec:
#             if ((start_sVec, end_sVec) in self.env.boat_route_dict):
#                 self.actions = deepcopy(self.env.boat_route_dict[(start_sVec, end_sVec)])
#                 main_logger.error("actions:%s",self.actions)
#             else:
#                 main_logger.error(f"boat{self.boat_id} 尝试在 {self.sVec} 从 {start_sVec} 出发到 {end_sVec}，路径不存在")
#         else:
#             main_logger.error(f"boat{self.boat_id} 尝试在 {self.sVec} 从 {start_sVec} 出发到 {end_sVec}，初始位置错误")
    
#     def boat_execute(self):
#         if self.status == 0 or self.status == 2:
#             if len(self.actions) > 0:
#                 action = self.actions.pop(0)
#                 if action == "ship":
#                     print("ship", self.boat_id)
#                 elif action == "rot 0":
#                     print("rot", self.boat_id, 0)
#                 elif action == "rot 1":
#                     print("rot", self.boat_id, 1)

