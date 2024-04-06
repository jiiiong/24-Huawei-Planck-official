from __future__ import annotations
import sys
import random
from enum import Enum
from typing import List, Dict
from queue import LifoQueue, PriorityQueue, Queue
import copy
import time

from cfg import TYPE_CHECKING, N
from log import  robot_logger, root_logger, func_timer
from path_planing import Point, Pixel_Attrs
from path_planing import one_move_avoidance

if TYPE_CHECKING:
    from .env import Env
from .goods import Goods
from .utils import enum_stk_and_recover, enum_stk_and_empty, enum_stk

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

robot_action_value_to_cmd = {
    Robot_Move.RIGHT : 0,
    Robot_Move.LEFT  : 1,
    Robot_Move.UP    : 2,
    Robot_Move.DOWN  : 3,
    Robot_Move.HOLD  : 4,
}

class Robot_Extended_Status(Enum):
    Uninitialized = -1
    BackBerthAndPull = 2
    UnableBackBerth = 21
    OnBerth = 1
    GotoFetchFromBerth = 3
    GotGoods = 4
    CollisionAvoidance = 5
    Recovery = 6



# priority_for_robot_extended_status = {
#     Robot_Extended_Status.UnableBackBerth : -100,
#     Robot_Extended_Status.Recovery: -50,
#     Robot_Extended_Status.CollisionAvoidance: 0,
#     Robot_Extended_Status.GotGoods: 100,
#     Robot_Extended_Status.OnBerth: 200,
#     Robot_Extended_Status.BackBerthAndPull: 300,
#     Robot_Extended_Status.GotoFetchFromBerth: 400,
#     Robot_Extended_Status.Uninitialized : 500
# }

priority_for_robot_extended_status = {
    Robot_Extended_Status.UnableBackBerth : -100,
    Robot_Extended_Status.Recovery: -50,
    Robot_Extended_Status.CollisionAvoidance: 0,
    Robot_Extended_Status.GotGoods: 100,
    Robot_Extended_Status.BackBerthAndPull: 200,
    Robot_Extended_Status.GotoFetchFromBerth: 300,
    Robot_Extended_Status.OnBerth: 400,
    Robot_Extended_Status.Uninitialized : 500
}

class Robot():

    def __init__(self, robot_id: int, env: 'Env', startX=0, startY=0, goods=0):
        self.robot_id = robot_id
        self.env= env
        self.pos = Point(x = startX, y = startY)
        self.goods = goods
        
        # 在第一帧开始前初始化的内容
        self.berth_id = -1
        self.target_gds: Goods = Goods(0, [0], UNREACHABLE_POS, 0)

        # 状态，每个状态经过规划、避障、执行；状态更新在第二帧调度前进行
        self.extended_status = Robot_Extended_Status.Uninitialized
        # 路径规划用，定义为接下来要经过的位置，空代表呆在原地
        self.paths_stk: LifoQueue[Point] = LifoQueue()

        # 避障用的变量
        self.alarming_area_size = 2
        self.surronding_robots_with_priority: Dict[int, int]= dict()
        self.collision_robots_id: List[int] = []
        # 进入避障状态时保存原有状态的变量 
        self.original_extended_status = Robot_Extended_Status.Uninitialized
        self.original_paths_stk = LifoQueue()
        # 解决单行道问题
        self.master_robot_id: int = -1
        # 避障计时
        self.count_collision_avoidance_time: int = 0

        # 碰撞后恢复paths_stk用
        self.suppose_pos = Point(x = startX, y = startY)
        self.empty_paths = True
        

    # 兼容原有代码用
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
    def berth(self):
        return self.env.berths[self.berth_id]

    def convert_extended_status(self, value: Robot_Extended_Status):
        valid = True
        last_status = self.extended_status

        if (last_status == Robot_Extended_Status.CollisionAvoidance):

            if self.original_extended_status == Robot_Extended_Status.GotGoods:
                self.paths_stk = LifoQueue()

            # Onberth恢复时特殊处理，如果避让stk非空，则将原来的状态设置为回港
            elif self.original_extended_status  == Robot_Extended_Status.OnBerth:
                if (not self.paths_stk.empty()):
                    # 将原来的状态设置为回港
                    self.original_extended_status = Robot_Extended_Status.BackBerthAndPull
            
            tmp_stk = LifoQueue()
            while not self.paths_stk.empty():
                tmp_stk.put(self.paths_stk.get())
            self.paths_stk = self.original_paths_stk
            while not tmp_stk.empty():
                self.paths_stk.put(tmp_stk.get())
 
            self.original_paths_stk = LifoQueue()
            
            self.extended_status = self.original_extended_status
            # 不需要path_update()
            return True

        elif (value == Robot_Extended_Status.BackBerthAndPull):
            # 转入条件：
            #   目标港口非不可达
            # 状态要求：
            #   priority, _extended_status, paths_stk
            if (self.berth.robot_cost_grid[self.x][self.y] != INFINIT_COST):
                self.extended_status = value
                self.path_update()
            else:
                valid = False
        
        elif (value == Robot_Extended_Status.OnBerth):
            # 转入条件：
            #   当前处于港口
            # 状态要求：
            #   当前处于港口
            #   paths_stk为空
            #   priority, _extended_status, 
            if (self.pos == self.env.berths[self.berth_id].pos):
                self.extended_status = value
                self.path_update()
            elif (self.berth.robot_cost_grid[self.x][self.y] != INFINIT_COST):
                self.extended_status = Robot_Extended_Status.BackBerthAndPull
                self.path_update()
            else:
                valid = False
            
        elif (value == Robot_Extended_Status.GotoFetchFromBerth):
            # 转入条件：
            #   当前处于港口
            #   由go_to_fetch_gds_from_berth启动，设置target_gds
            # 状态要求：
            #   priority, _extended_status, paths_stk, target_gds
            if (self.pos == self.env.berths[self.berth_id].pos):
                self.extended_status = value
                self.path_update()       
            else:
                valid = False

        elif (value == Robot_Extended_Status.GotGoods):
            # 转入条件：
            #   必须由GotoFetchFromBerth转入
            # 状态要求：
            #   priority, _extended_status, paths_stk
            if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
                self.extended_status = value
                self.path_update()
            else:
                valid = False

        elif (value == Robot_Extended_Status.UnableBackBerth):
            # 转入条件：
            #   目标港口不可达
            # 状态要求：
            #   priority, _extended_status, paths_stk
            if (self.berth.robot_cost_grid[self.x][self.y] == INFINIT_COST):
                self.extended_status = value
                self.path_update()
            else:
                valid = False

        elif (value == Robot_Extended_Status.CollisionAvoidance):
            # 转入条件：
            #   发生碰撞，且为低优先级
            # 状态要求：
            #   需要为负责对象让路
            #   priority, _extended_status, paths_stk
            #   responsible_robot_id
                self.extended_status = value
                self.path_update()

        if valid:
            robot_logger.info(f"Stat Conv by id: {self.robot_id} at {self.pos}, from {last_status} to {value}")
            return True
        else:
            robot_logger.error(f"Stat Conv by id: {self.robot_id} at {self.pos}, from {last_status} to {value}, Invalid")
            return False

    # api：从绑定的港口出发，去某地取货
    def go_to_fetch_gds_from_berth(self, target_gds: Goods):
        if self.berth_id == -1:
            return False
        
        self.target_gds = target_gds
        return self.convert_extended_status(Robot_Extended_Status.GotoFetchFromBerth)
        
    # api: 返回绑定的港口
    def back_berth(self):
        return self.convert_extended_status(Robot_Extended_Status.BackBerthAndPull)
        
    # api：返回一个新港口，若不可达，则不操作
    def back_new_berth(self, new_berth_id: int):
        robot_move_grid = self.berth.robot_move_grid
        if (robot_move_grid[self.x][self.y] != UNREACHABLE_POS):
            self.berth_id = new_berth_id
            self.paths_stk = LifoQueue()
            return self.convert_extended_status(Robot_Extended_Status.BackBerthAndPull)
        else:
            root_logger.error(f"new berth_id {new_berth_id} unreachable by id: {self.robot_id}")
            return False

    def check_A_is_ancestor(self, A_robot:Robot):
        robots = self.env.robots
        cur_robot = robots[self.master_robot_id]
        while (cur_robot.extended_status == Robot_Extended_Status.CollisionAvoidance):
            if (A_robot.robot_id == cur_robot.robot_id):
                return True
            else:
                cur_robot = robots[cur_robot.master_robot_id]
        # 到达祖先
        if (A_robot.robot_id == cur_robot.robot_id):
            return True
        else:
            return False

    def ancestors(self) -> List[int]:
        ancestors: List[int] = []

        if self.extended_status != Robot_Extended_Status.CollisionAvoidance:
            return ancestors
        
        robots = self.env.robots
        cur_robot  = robots[self.master_robot_id]
        while (cur_robot.extended_status == Robot_Extended_Status.CollisionAvoidance):
            ancestors.append(cur_robot.robot_id)
            cur_robot = robots[cur_robot.master_robot_id]
        ancestors.append(cur_robot.robot_id)
        
        return ancestors

    def get_root_id(self):
        '''要求本身处于避障状态'''
        robots = self.env.robots
        cur_robot  = self
        while (cur_robot.extended_status == Robot_Extended_Status.CollisionAvoidance):
            cur_robot = robots[cur_robot.master_robot_id]
        return cur_robot.robot_id
    
    # 所有状态：
    # 1. 是否为避障链中
    # 2. 是否为直系祖先
    # 此为获取相对的优先级
    def get_priority_for_A(self, A_robot: Robot):
        # 如果自己处在避障链中
        if (self.extended_status == Robot_Extended_Status.CollisionAvoidance):
            # 如果A是祖先，则Ａ拥有高优先级；如果A是其他避障树的非根，修改A可能对破坏那颗避障树
            if self.check_A_is_ancestor(A_robot) is True:
                return 600 + self.robot_id
            else:
                return -100 + self.robot_id
        # 如果自己是root节点，A的优先级低于自己
        else:
            # A为避障节点
            if (A_robot.extended_status == Robot_Extended_Status.CollisionAvoidance):
                return -100 + self.robot_id
            else: 
                return priority_for_robot_extended_status[A_robot.extended_status] + A_robot.robot_id

    # 考虑避障时看到原来的路径
    def next_n_pos(self, n:int = 1) -> List[Point]:
        poses: List[Point] = []
        count = 0
        final_pos = Point(self.x, self.y)
        
        for stk in [self.paths_stk, self.original_paths_stk]:
            for next_pos in enum_stk_and_recover(stk):
                if count < n:
                    poses.append(next_pos)
                    final_pos = next_pos
                    count += 1
                else:
                    break
        while count < n:
            poses.append(final_pos)
            count += 1

        return poses
    
    def collision_check_and_update(self):

        # 每一帧都初始化 附近的机器人 and 会碰撞的机器人
        # {robot_id : priority, ...}
        self.surronding_robots_with_priority = {self.robot_id: priority_for_robot_extended_status[self.extended_status] + self.robot_id}
        # [robot_id, ...]
        self.collision_robots_id = [self.robot_id]

        # 检查警戒范围内是否有其他机器人 
        robots = self.env.robots               
        for robot in robots:
            if (robot.robot_id != self.robot_id):
            # 如果警戒范围内存在其他机器人
                if  (self.pos.distance(robot.pos) <= self.alarming_area_size):
                    # 将机器人加入surrounding字典，并让其value为优先级
                    self.surronding_robots_with_priority[robot.robot_id] = self.get_priority_for_A(robot)
                    # 如果那个机器人下一帧会与自己碰撞，则加入碰撞机器人队列
                    pos1 = self.next_n_pos(1)[0]
                    pos2 = robot.next_n_pos(1)[0]
                    # 
                    if self.env.attrs_grid[pos1.x][pos1.y].is_free_ground == False:
                        if (pos1 == pos2 
                            or (pos1 == robot.pos and pos2 == self.pos)):
                            self.collision_robots_id.append(robot.robot_id)

                    # # 1. 前往相同位置且目标位置非主干道
                    # if (pos1 == pos2 and self.env.attrs_grid[pos1.x][pos1.y].is_free_ground == False):
                    #     self.collision_robots_id.append(robot.robot_id)
                    # # 2. 互相前往对方位置, 且没有任意一方为主干道
                    # # 实际上，前往非主干道的一方会碰撞，但前往主干道的一方不会；
                    # elif  (pos1 == robot.pos and pos2 == self.pos
                    #        and self.env.attrs_grid[pos1.x][pos1.y].is_free_ground == False
                    #        and self.env.attrs_grid[pos2.x][pos2.y].is_free_ground == False):
                    #     self.collision_robots_id.append(robot.robot_id)

        if (len(self.collision_robots_id) > 1):
            return True
        else:
            return False

    def collision_avoid(self):
        robots = self.env.robots
        okk = True
        # 如果存在会碰撞的机器人
        if (self.collision_check_and_update()):
            # 找出碰撞车中最高优先级的
            max_id = max(self.collision_robots_id, key = (lambda id: self.surronding_robots_with_priority[id]))
            
            # # 如果自己是碰撞集合中最高优先级的车
            # if (self.robot_id == max_id):
            #     ordered_collision_robots_id = [ (robot_id, self.pos.distance(robots[robot_id])) for robot_id in self.collision_robots_id]
            #     ordered_collision_robots_id.sort(key=lambda x:x[1])
            #     # 按顺序激活其他车辆的避障
            #     for id, _ in ordered_collision_robots_id:
            #     # for id in self.collision_robots_id:
            #         # 排除自己
            #         if (id != max_id):
            #             # 如果非避障，其可能是别人的root，或者普通
            #             if (robots[id].extended_status != Robot_Extended_Status.CollisionAvoidance):
            #                 robot_logger.info(f"at avoidance id: {self.robot_id}")
            #                 robots[id].enable_collision_avoidance(max_id)
            #             else:
            #                 # self优先级高于避障，则此时不可能为普通状态
            #                 self_root_id = self.get_root_id()
            #                 a_root_id = robots[id].get_root_id()
            #                 if a_root_id != self_root_id:
            #                     robots[id].master_robot_id = max_id
            #                     robots[id].path_update()
            #                 else:
            #                     robots[a_root_id].enable_collision_avoidance(max_id)

            # # 备选方案
            if (self.robot_id != max_id):
                if (self.extended_status != Robot_Extended_Status.CollisionAvoidance):
                    robot_logger.info(f"at avoidance id: {self.robot_id}")
                    self.enable_collision_avoidance(max_id)
                else:
                    self.path_update()
            else:
                ordered_collision_robots_id = [ (robot_id, self.pos.distance(robots[robot_id].pos)) for robot_id in self.collision_robots_id]
                ordered_collision_robots_id.sort(key=lambda x:x[1])
                for id, _ in ordered_collision_robots_id:
                # for id in self.collision_robots_id:
                    # 排除自己
                    if (id != max_id):
                        # 如果还未进入避障状态，启动避障状态，并重新计算路径
                        if (robots[id].extended_status != Robot_Extended_Status.CollisionAvoidance):
                            robot_logger.info(f"at avoidance id: {self.robot_id}")
                            robots[id].enable_collision_avoidance(max_id)
                        else:
                            # self优先级高于避障，则此时不可能为普通状态
                            self_root_id = self.get_root_id()
                            a_root_id = robots[id].get_root_id()
                            if a_root_id == self_root_id:
                                robots[id].master_robot_id = max_id
                                robots[id].path_update()
                            else:
                                robots[a_root_id].enable_collision_avoidance(max_id)
                    
            # for id in self.collision_robots_id:
            #     # 当前robot非优先级最高
            #     # ！！！！！！！！！！！！！计算的相对优先级，此处存在逻辑问题
            #     if (id != max_id#): # and priority(max_id) > priority(id)
            #         and priority_for_robot_extended_status[robots[id].extended_status] + robots[id].robot_id < robots[id].get_priority_for_A(robots[max_id])):
            #         # 如果还未进入避障状态，启动避障状态，并重新计算路径
            #         if (robots[id].extended_status != Robot_Extended_Status.CollisionAvoidance):
            #             robot_logger.info(f"at avoidance id: {self.robot_id}")
            #             robots[id].enable_collision_avoidance(max_id)
            #         else:
            #             robots[id].path_update()
        return okk

    def try_find_avoidance_path(self):
        robots = self.env.robots
        success = False
        avoidance_paths_stk = LifoQueue()
        # collision_check同时会更新self.surrounding, self.collision_robots_ud

        if (self.collision_check_and_update() == False):
            success = True
            return success, avoidance_paths_stk
        
        avoidance_grid_len = 5 # = alarming_side + predict_steps = 2 + 3 = 5
        # 获得障碍图
        l = self.pos.y - avoidance_grid_len
        r = self.pos.y + avoidance_grid_len
        t = self.pos.x - avoidance_grid_len
        b = self.pos.x + avoidance_grid_len
        if l < 0: l = 0
        if r > 199:  r = 199
        if t < 0: t = 0
        if b > 199: b = 199

        avoidance_grid: List[List[Pixel_Attrs]] = []
        col = -1
        for y in range(t, b+1):
            avoidance_grid.append([])
            col += 1
            for x in range(l, r+1):
                avoidance_grid[col].append(self.env.attrs_grid[x][y])
        
        # 1. 尝试针对所有周围的对象进行避障
        predict_steps = 3
        list_avoidance_paths = []
        while (success == False and predict_steps > 0):
            ins_avoidance_grid = copy.deepcopy(avoidance_grid)
            # 将机器人的位置和其路径视为障碍物
            for robot_id in self.surronding_robots_with_priority:
                # 不考虑自己
                if robot_id == self.robot_id:
                    continue
                robot = self.env.robots[robot_id]
                poses: List[Point] = []
                # 如果对撞，则不可以前往对方的位置!!
                if (robots[robot_id].next_n_pos(1)[0] == self.pos):
                    poses.append(Point(robot.x, robot.y))
                poses += robot.next_n_pos(predict_steps)
                for pos in poses:
                    p_a = ins_avoidance_grid[pos.x-t][pos.y-l]
                    # 值得注意的是，如果目前的位置不是主干道，对撞会导致对方碰撞，但只眩晕1帧，貌似开销不大。
                    p_a.is_ground = (False or p_a.is_free_ground)

            # 尝试一条避障路径
            list_avoidance_paths, success = one_move_avoidance(ins_avoidance_grid,
                                                            Point(self.pos.x-l, self.pos.y-t))
            predict_steps -= 1
        

        # 2. 如果无法避障成功, 尝试避开master robot
        robots = self.env.robots
        if success == False:
            success_second = False
            predict_steps = 2
            list_avoidance_paths = []
            ancestors = self.ancestors()
            while (success_second == False and predict_steps > 0):
                ins_avoidance_grid = copy.deepcopy(avoidance_grid)
                # 只考虑master robot
                for robot_id in ancestors: 
                    robot = self.env.robots[robot_id]
                    poses: List[Point] = []
                    if (robot.next_n_pos(1)[0] == self.pos):
                        poses.append(Point(x = robot.x, y = robot.y))
                    poses += robot.next_n_pos(predict_steps)
                    for pos in poses:
                        p_a = ins_avoidance_grid[pos.x-t][pos.y-l]
                        p_a.is_ground = (False or p_a.is_free_ground)
                # 尝试一条避障路径
                list_avoidance_paths, success_second = one_move_avoidance(ins_avoidance_grid,
                                                            Point(self.pos.x-l, self.pos.y-t))
                predict_steps -= 1

        for item in list_avoidance_paths:
            item = Point(item.x + l, item.y + t)
            avoidance_paths_stk.put(item)

        return success, avoidance_paths_stk

        # 尝试规划避让路径

    def gen_recoverable_paths(self, following_stk: LifoQueue):
        # 如果呆在原地，则会导致生成两步原地？？？？？？？？？？  
        cur_pos = Point(self.pos.x, self.pos.y)
        reverse_stk = LifoQueue()

        if (following_stk.empty()):
            return

        # 构造回溯位置，包括原始位置
        for item in enum_stk_and_recover(following_stk):
            reverse_stk.put(item)
        # 对避障路径为原点进行特殊处理，
        stay = False
        if (reverse_stk.qsize() == 1):
            for item in enum_stk_and_recover(reverse_stk):
                if item == cur_pos:
                    stay = True
        if not stay:
            reverse_stk.put(cur_pos)

        following_stk.get() # 去除一个重复的终点
        # 将新的回归路径拼接到原来的回归路径上
        last_index = -1
        for i, item in enumerate(enum_stk_and_recover(self.paths_stk)):
            if (item == cur_pos):
                last_index = i
        if (last_index != -1):
            for i, item in enumerate(enum_stk(self.paths_stk)):
                if i == last_index:
                    break

        for item in enum_stk(reverse_stk):
            self.paths_stk.put(item)
        for item in enum_stk(following_stk):
            self.paths_stk.put(item)

    # 启动避障状态，该状态会通过 搜索算法 尽可能远离其他机器人的位置，并记录移动的路径
    # 启动时会直接重新计算路径
    def enable_collision_avoidance(self, master_robot_id: int):
        self.master_robot_id = master_robot_id
        self.original_extended_status = self.extended_status
        self.original_paths_stk = self.paths_stk
        self.paths_stk = LifoQueue()
        robot_logger.info(f"Enab Avoi by id: {self.robot_id} master_id: {master_robot_id}")
        self.convert_extended_status(Robot_Extended_Status.CollisionAvoidance)
    
    # 退出避障
    # 将collision期间的paths附加到原来状态的paths上
    def try_disable_collision_avoidance(self):
        self.convert_extended_status(self.original_extended_status)
        robot_logger.info(f"Disa Avoi by id: {self.robot_id} at {self.pos}, status: {self.extended_status}, next_3_pos: {self.next_n_pos(3)}")

    @func_timer
    def path_update(self):  
            move_grid = self.berth.robot_move_grid
            berths = self.env.berths

            # 更新路径为到目标点的路径
            if self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
                target_pos = self.target_gds.pos
                if (move_grid[target_pos.x][target_pos.y] == UNREACHABLE_POS):
                    self.convert_extended_status(Robot_Extended_Status.OnBerth)
                elif (berths[self.berth_id].pos == target_pos): 
                    self.paths_stk = LifoQueue()
                else:
                    self.paths_stk = LifoQueue()
                    cur_pos = target_pos
                    self.paths_stk.put(cur_pos)
                    cur_pos = cur_pos + move_grid[cur_pos.x][cur_pos.y]
                    while (cur_pos != berths[self.berth_id].pos):
                        self.paths_stk.put(cur_pos)
                        cur_pos = cur_pos + move_grid[cur_pos.x][cur_pos.y]
                
            elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
               
                if (self.pos == berths[self.berth_id].pos):
                    self.paths_stk = LifoQueue()
                else:
                    self.paths_stk = LifoQueue()
                    tmp_stk = LifoQueue()
                    cur_pos = self.pos
                    cur_pos = cur_pos + move_grid[cur_pos.x][cur_pos.y]
                    while (cur_pos != berths[self.berth_id].pos): # 此时self.pos就是在berth上
                        tmp_stk.put(cur_pos)
                        #logger.info("debug %s | %s", cur_pos, berths[berth_id].pos)
                        cur_pos = cur_pos + move_grid[cur_pos.x][cur_pos.y]
                    tmp_stk.put(cur_pos)
                    
                    while not tmp_stk.empty():
                        self.paths_stk.put(tmp_stk.get())
                
            elif self.extended_status == Robot_Extended_Status.CollisionAvoidance:
                success, tmp_paths_stk = self.try_find_avoidance_path()
                # 尝试能够回溯的避障路径
                self.gen_recoverable_paths(tmp_paths_stk)

                if success is False:
                    robots = self.env.robots
                    # 针对新规划的路径（只对负责的对象避障）进行避障
                    self.collision_check_and_update()
                    ordered_surronding_robots_id = [ (robot_id, self.pos.distance(robots[robot_id].pos)) for robot_id in self.surronding_robots_with_priority]
                    ordered_surronding_robots_id.sort(key=lambda x:x[1])
                    for robot_id, _ in ordered_surronding_robots_id:
                        if self.surronding_robots_with_priority[robot_id] < self.surronding_robots_with_priority[self.robot_id]:
                            if (robots[robot_id].extended_status != Robot_Extended_Status.CollisionAvoidance):
                                robot_logger.info(f"at pathupdate id: {self.robot_id}")
                                robots[robot_id].enable_collision_avoidance(self.robot_id)
                            else:
                                robots[robot_id].path_update()

                    success, tmp_paths_stk = self.try_find_avoidance_path()
                    # 尝试能够回溯的避障路径
                    if success:
                        self.gen_recoverable_paths(tmp_paths_stk)

            elif self.extended_status in [Robot_Extended_Status.OnBerth, 
                                          Robot_Extended_Status.GotGoods,
                                          Robot_Extended_Status.UnableBackBerth]:
                self.paths_stk = LifoQueue()
            
            robot_logger.info(f"Path Upda by id: {self.robot_id} at {self.pos}, status: {self.extended_status}, next3pos:{self.next_n_pos(3)}")

            return True

    def paths_execution(self):
        # 取消当前避障状态
        # 避障避免中可能会依赖 原路径 实现不碰撞
        # 可能会因为未及时退出避障状态而使得，原路径中下一步无法被执行
        # 值得注意的是，取消避障被放在外层，是因为防止部分robot因为出栈导致原来不碰撞的路径现在检测为碰撞
        if (self.extended_status == Robot_Extended_Status.CollisionAvoidance):
            self.count_collision_avoidance_time += self.paths_stk.qsize()
            self.try_disable_collision_avoidance()

        if not self.paths_stk.empty():
            self.empty_paths = False
            next_pos = self.paths_stk.get(False)
            action = next_pos - self.pos
            if (action not in robot_action_value_to_cmd):
                robot_logger.error(f"invalid execution by id: {self.robot_id} at pos {self.pos}, next_pos: {next_pos}")
                return ""
        else:
            self.empty_paths = True
            next_pos = self.pos
            action = Robot_Move.HOLD
        
        self.suppose_pos = next_pos
        
        if (action != Robot_Move.HOLD):
            return (f"move {self.robot_id} {robot_action_value_to_cmd[action]}\n")
        else:
            return ""

    # 根据状态的执行结果改变状态，区别于run中的状态变化（如何区别？）
    def update_extended_status(self):
        '''在规划、执行后进行，在下一帧进行更新；
            当前帧的状态更新放在下一帧的最开始，
            是因为机器人的状态，下一帧才能被更新'''
        berths = self.env.berths
        # 避免碰撞时self.path_stks错误，根据status恢复
        if (self.pos != self.suppose_pos and self.empty_paths is False):
            self.paths_stk.put(self.suppose_pos)
        
        elif self.extended_status == Robot_Extended_Status.GotoFetchFromBerth:
            if (self.paths_stk.empty()):
                # 可能无法取到物品 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                # 取不到物品该如何处理????????????????????????????????????????????????????????????????????????????????????????????????
                print("get", self.robot_id)
        
                self.convert_extended_status(Robot_Extended_Status.GotGoods)
        elif self.extended_status == Robot_Extended_Status.BackBerthAndPull:
            if (self.paths_stk.empty()):
                self.convert_extended_status(Robot_Extended_Status.OnBerth)
                if self.goods == 1:
                    print("pull", self.robot_id)
                    berths[self.berth_id].cur_num_gds += 1
                    berths[self.berth_id].total_num_gds += 1
                    berths[self.berth_id].total_earn += self.target_gds.price
        
        # # 如果发生碰撞，则不会移动，所以不需要担心避障时碰撞导致无法记录路径
        # elif self.extended_status == Robot_Extended_Status.CollisionAvoidance:
        #     if (self.collision_check() == False):
        #             self.try_disable_collision_avoidance()


        