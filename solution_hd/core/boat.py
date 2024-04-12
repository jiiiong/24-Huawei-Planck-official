from __future__ import annotations
from asyncio import LifoQueue
from typing import List, Dict, Tuple
from path_planing import Point, Pixel_Attrs
from path_planing import Boat_Direction, Boat_Action
from path_planing import ship_one_move, rotate_one_move, route_a_star
from enum import Enum
from log import logger, boat_logger

from cfg import TYPE_CHECKING, N

import copy

if TYPE_CHECKING:
    from .env import Env

cmd_boatDirection_map = {
    0: Boat_Direction.RIGHT,
    1: Boat_Direction.LEFT,
    2: Boat_Direction.UP,
    3: Boat_Direction.DOWN,
    4: Boat_Direction.ANYWHERE # 当知道 start point, start direction 和 end point 但不需要规定 end direction 时 使用
}

# 额外的状态，用于状态切换
class Boat_Extended_Status(Enum):
    Uninitialized = -1
    OnBerth = 1
    BackBerthAndLoad = 3
    fromBerth2Delivery = 4
    CollisionAvoidance = 6
    Recovery = 7
    OnDeliveryPoint = 8

# 每一种状态的优先级
priority_for_boat_extended_status = {
    Boat_Extended_Status.Uninitialized: 500,
    Boat_Extended_Status.OnBerth: 400,
    Boat_Extended_Status.CollisionAvoidance: 0,
    Boat_Extended_Status.Recovery: -50,
    Boat_Extended_Status.BackBerthAndLoad: 300,
    Boat_Extended_Status.fromBerth2Delivery: 450,
    Boat_Extended_Status.OnDeliveryPoint: 350
}




class Boat:
    def __init__(self, env: Env, boat_id=0, x=0, y=0, dir=0, num=0,  status=0):
        self.boat_id = boat_id
        self.goods_num = num
        self.pos = Point(x,y)
        self._dir = cmd_boatDirection_map[dir]
        self.status = status
        self.env = env

        # 当前目标港口
        self.target_berth_id = 0
        # 当前目标运输点
        self.target_delivery_point_id = 0

        # 解决碰撞用的变量
        self.extended_status = Boat_Extended_Status.Uninitialized
        #  路径规划用的，动作栈
        self.actions_stack = []

        # 两个核心点要距离多近才会触发警报?
        self.alarming_area_size = 8
        self.surrounding_boats_with_priority: Dict[int, int] = dict()
        self.collision_boats_id: List[int] = []

        # 进入避障状态时保存原有状态的变量
        self.original_extended_status = Boat_Extended_Status.Uninitialized
        self.original_actions_stack = []

        # 解决单行道问题
        self.master_boat_id: int = -1

        # 避障计时
        self.count_collision_avoidance_time: int = 0

        # 碰撞后恢复action_stk
        # self.suppose_action =
        self.empty_actions = True

        # 用于避免碰撞后恢复到的 位置和方向
        self.collision_avoidance_pos: Point
        self.collision_avoidance_direction: Boat_Direction

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


    def convert_extended_status(self, value: Boat_Extended_Status):
        valid = True
        last_status = self.extended_status
        logger.info(f"BOAT {self.boat_id} FUNC: convert_extended_status > self.status: {last_status}")
        logger.info(f"BOAT {self.boat_id} FUNC: convert_extended_status > target status: {value}")
        # 如果当前状态处于避障状态
        if last_status == Boat_Extended_Status.CollisionAvoidance:

            # OnBerth 恢复时特殊处理，如果避让stk非空，则将原来的状态设置为回港
            if self.original_extended_status == Boat_Extended_Status.OnBerth:
                if not self.actions_stack.empty():
                    # 将原来的状态设置为回港
                    self.original_extended_status = Boat_Extended_Status.BackBerthAndLoad

            # tmp_stk = LifoQueue()
            # while not self.actions_stack.empty():
            #     tmp_stk.put(self.actions_stack.get())
            # self.actions_stack = self.original_actions_stack
            # while not tmp_stk.empty():
            #     self.actions_stack.put(tmp_stk.get())
            #
            # self.original_paths_stk = LifoQueue()
            #
            # self.extended_status = self.original_extended_status
            # # 不需要path_update()
            valid = True

        elif value == Boat_Extended_Status.BackBerthAndLoad:
            # 转入条件：
            #   船的核心点在泊口处 or 船的核心点在生成点 or 船的核心点在运输点
            transfer_ok = False
            if transfer_ok is False:
                for berth in self.env.berths:
                    if self.x == berth.x and self.y == berth.y:
                        self.extended_status = value
                        self.action_update()
                        transfer_ok = True
                        break
            if transfer_ok is False:
                for purchase in self.env.boat_purchase_point:
                    if self.x == purchase.x and self.y == purchase.y:
                        self.extended_status = value
                        self.action_update()
                        transfer_ok = True
                        break

            if transfer_ok is False:
                for t in self.env.delivery_point:
                    if self.x == t.x and self.y == t.y:
                        self.extended_status = value
                        self.action_update()
                        transfer_ok = True
                        break

            if transfer_ok is True:
                valid = True
            else:
                valid = False



        elif value == Boat_Extended_Status.OnBerth:
            # 转入条件：
            #   当前处于港口
            # 状态要求：
            #   当前处于港口
            #   actions_stack 为空

            # 没有处理港口不可达的问题
            can_OnBerth = False
            for berth in self.env.berths:
                if self.x == berth.x and self.y == berth.y and len(self.actions_stack) == 0:
                    can_OnBerth = True
                    self.extended_status = value
                    self.action_update()
                    break
            if can_OnBerth is True:
                valid = True
            else:
                valid = False

        elif value == Boat_Extended_Status.fromBerth2Delivery:
            # 转入条件：
            #   当前处于港口，
            #   状态为OnBerth状态，
            #   action_stack 为空
            can_FromBerth = False
            for berth in self.env.berths:
                if self.x == berth.x and self.y == berth.y and len(self.actions_stack) == 0 \
                    and self.extended_status == Boat_Extended_Status.OnBerth:
                    self.extended_status = value
                    self.action_update()
                    can_FromBerth = True
                    break

            if can_FromBerth is True:
                valid = True
            else:
                valid = False

        elif value == Boat_Extended_Status.OnDeliveryPoint:
            # 转入条件，船只的核心点在运输点上
            can_OnDeliveryPoint = False
            for delivery_point in self.env.delivery_point:
                if self.x == delivery_point.x and self.y == delivery_point.y and len(self.actions_stack) == 0:
                    self.extended_status = value
                    self.action_update()
                    can_OnDeliveryPoint = True
                    break

                if can_OnDeliveryPoint is True:
                    valid = True
                else:
                    valid = False

        elif value == Boat_Extended_Status.CollisionAvoidance:
            # 转入条件:
            #   发生碰撞，且为低优先级
            # 状态要求:
            #   需要为负责对象让路

            self.extended_status = value
            self.action_update()

        if valid is True:
            logger.info(f"BOAT {self.boat_id} At zhen: {self.env.global_zhen} Boat Stat Conv by id: {self.boat_id} at {self.pos}, from {last_status} to {value}")
            return True
        else:
            logger.info(f"BOAT {self.boat_id} At zhen: {self.env.global_zhen} Boat Stat Conv by id: {self.boat_id} at {self.pos}, from {last_status} to {value}, Invalid")
            return False


    # api：
    # 从生成点出发，去某个港口, 接收参数，目标港口的id
    def go_to_berth_from_purchase_point(self, berth_index):
        if berth_index > len(self.env.berths) or berth_index < 0:
            return False
        on_bpp = False
        for i, bpp in enumerate(self.env.boat_purchase_point):
            if self.x == bpp.x or self.y== bpp.y:
                on_bpp = True
                break
        # 更新 目标港口
        if on_bpp:
            self.target_berth_id = berth_index
            return self.convert_extended_status(Boat_Extended_Status.BackBerthAndLoad)
        else:
            return False

    # api 在港口之间移动
    #   要求当前处于港口，传入目标港口的id:
    def ship_between_berths(self, target_berth_id):
        if target_berth_id > len(self.env.berths) or target_berth_id < 0:
            return False
        on_bpp = False

        for i, berth in enumerate(self.env.berths):
            if self.x == berth.x or self.y == berth.y:
                logger.info(f"BOAT {self.boat_id} At zhen: {self.env.global_zhen} Func: ship_between_berths failed: boat not at berth")
                on_bpp = True
                break
        if on_bpp:
            self.target_berth_id = target_berth_id
            return self.convert_extended_status(Boat_Extended_Status.BackBerthAndLoad)
        else:
            return False

    # api： 从运输点返回到指定的港口
    #      要求当前船在运输点
    def go_to_berth_from_delivery_point(self, berth_id):
        if berth_id > len(self.env.berths) or berth_id < 0:
            return False

        on_dp = False
        for i, dp in enumerate(self.env.delivery_point):
            if self.x == dp.x or self.y == dp.y:
                on_dp = True
                break

        if on_dp:
            self.target_berth_id = berth_id
            return self.convert_extended_status(Boat_Extended_Status.BackBerthAndLoad)
        else:
            return False

    # api: 从港口 出发到指定的运输点
    #   要求：当前船在港口上
    def go_to_delivery_point(self, delivery_point_id):
        if delivery_point_id > len(self.env.delivery_point) or delivery_point_id < 0:
            return False

        on_berth = False
        for i, berth in enumerate(self.env.berths):
            if self.x == berth.x or self.y == berth.y:
                on_berth = True
                break
        if on_berth:
            self.target_delivery_point_id = delivery_point_id
            return self.convert_extended_status(Boat_Extended_Status.fromBerth2Delivery)
        else:
            return False

    def check_A_is_ancestor(self, A_boat: Boat):
        boats = self.env.boats
        cur_boat = boats[self.master_boat_id]
        while cur_boat.extended_status == Boat_Extended_Status.CollisionAvoidance:
            if A_boat.boat_id == cur_boat.boat_id:
                return True
            else:
                cur_boat = boats[cur_boat.master_boat_id]

        # 到达祖先
        if A_boat.boat_id == cur_boat.boat_id:
            return True
        else:
            return False

    def ancestors(self) -> List[int]:
        ancestors: List[int] = []
        if self.extended_status != Boat_Extended_Status.CollisionAvoidance:
            return ancestors

        boats = self.env.boats
        cur_boat = boats[self.master_boat_id]
        while cur_boat.extended_status == Boat_Extended_Status.CollisionAvoidance:
            ancestors.append(cur_boat.master_boat_id)
            cur_boat = boats[cur_boat.master_boat_id]
        ancestors.append(cur_boat.boat_id)

        return ancestors

    def get_root_id(self):
        # 要求本身处于避障状态
        boats = self.env.boats
        cur_boat = self
        while cur_boat.extended_status == Boat_Extended_Status.CollisionAvoidance:
            cur_boat = boats[cur_boat.master_boat_id]
        return cur_boat.boat_id

    # 所有状态：
    # 1. 是否为避障链中
    # 2. 是否为直系祖先
    # 此为获取相对的优先级
    def get_priority_for_A(self, A_boat: Boat):
        # 如果自己处在避障链中
        if self.extended_status == Boat_Extended_Status.CollisionAvoidance:
            # 如果 A 是祖先，A 拥有最高优先级；如果A是其他避免碰撞树，修改A可能会破坏那颗树
            if self.check_A_is_ancestor(A_boat) is True:
                return 600 + self.boat_id
            else:
                return -100 + self.boat_id
        # 如果自己是root节点，A的优先级低于自己
        else:
            # A 为避障节点
            if A_boat.extended_status == Boat_Extended_Status.CollisionAvoidance:
                return -100 + self.boat_id
            else:
                return priority_for_boat_extended_status[A_boat.extended_status] + A_boat.boat_id

    # 返回下n帧，船的核心点所在的位置
    def next_n_pose(self, n:int = 1) -> List[Point]:
        poses: List[Point] = []
        count = 0
        final_pos = Point(self.x, self.y)
        # logger.info(f"func: next_n_pose > self.pos: {self.pos}")
        # logger.info(f"func: next_n_pose > self.dir: {self.dir}")

        i = 0
        # 如果 i 小于 self.actions_stack 的长度，证明还没有遍历完当前的actions_stack
        while i < len(self.actions_stack):
            action = self.actions_stack[i]
            if count < n:
                if action == "ship":
                    cur_sVec = (self.pos, transfer_direction(self.dir))
                    okk, sVec = ship_one_move(cur_sVec, self.env.boat_direction_grid)
                    poses.append(sVec[0])

                elif action == "rot 0":
                    cur_sVec = (self.pos, transfer_direction(self.dir))
                    okk, sVec = rotate_one_move(cur_sVec, 0, self.env.boat_direction_grid, self.env.attrs_grid)
                    poses.append(sVec[0])

                elif action == "rot 1":
                    cur_sVec = (self.pos, transfer_direction(self.dir))
                    okk, sVec = rotate_one_move(cur_sVec, 1, self.env.boat_direction_grid, self.env.attrs_grid)
                    poses.append(sVec[0])

                # 如果在主航道，则为每个动作还要多增加一帧的原地恢复时间
                if self.env.attrs_grid[sVec[0].x][sVec[0].y].is_free_ocean:
                    poses.append(sVec[0])
                    count += 1
                i += 1
                count += 1
            else:
                return poses

        i = 0
        while i < len(self.original_actions_stack):
            action = self.original_actions_stack[i]
            if count < n:
                if action == "ship":
                    cur_sVec = Tuple[self.pos, transfer_direction(self.dir)]
                    okk, sVec = ship_one_move(cur_sVec, self.env.boat_direction_grid)
                    poses.append(sVec[0])

                elif action == "rot 0":
                    cur_sVec = Tuple[self.pos, transfer_direction(self.dir)]
                    okk, sVec = rotate_one_move(cur_sVec, 0, self.env.boat_direction_grid, self.env.attrs_grid)
                    poses.append(sVec[0])

                elif action == "rot 1":
                    cur_sVec = Tuple[self.pos, transfer_direction(self.dir)]
                    okk, sVec = rotate_one_move(cur_sVec, 1, self.env.boat_direction_grid, self.env.attrs_grid)
                    poses.append(sVec[0])

                # 如果在主航道，则为每个动作还要多增加一帧的原地恢复时间
                if self.env.attrs_grid[sVec[0][0]][sVec[0][1]].is_free_ocean:
                    poses.append(sVec[0])
                    count += 1
                i += 1
                count += 1
            else:
                return poses

        while count < n:
            poses.append(final_pos)
            count += 1
        return poses

    def collision_check_and_update(self):
        # 每一帧都初始化 附近的船只 and 会碰撞的船只
        # { boat_id: priority }
        self.surrounding_boats_with_priority = {self.boat_id: priority_for_boat_extended_status[self.extended_status] + self.boat_id}
        # [boat_id]
        self.collision_boats_id = [self.boat_id]

        boats = self.env.boats
        # 检查警戒范围内是否有其他船只
        for boat in boats:
            if boat.boat_id == self.boat_id:
                continue
            if self.pos.distance(boat.pos) <= self.alarming_area_size:
                # 将船加入surrounding 字典，并让其value为优先级
                self.surrounding_boats_with_priority[boat.boat_id] = self.get_priority_for_A(boat)

                # 判断三种情况，
                # 1. 如果 A, B 同时不处于主航道，则正常处理
                # 交给上层处理
                # 2. 如果 A，B 同时在主航道，
                #   先不做处理？
                # 3. 如果 A在主航道，B不在主航道，让A通过depth 命令回到主航道并等待10帧（给B充分的时间进入主航道或者离开）
                # 交给上层处理
                # 逻辑为，当 A 进入避障状态且在主航道上时，进入等待

                if self.env.attrs_grid[self.x][self.y].is_free_ocean and self.env.attrs_grid[boat.x][boat.y].is_free_ocean:
                    continue
                # 获取下两帧在的位置
                self_pos_list = self.next_n_pose(2)
                a_pos_list = boat.next_n_pose(2)

                self_set_1 = set(self.boat_cover_area(self_pos_list[0].x, self_pos_list[0].y))
                self_set_2 = set(self.boat_cover_area(self_pos_list[1].x, self_pos_list[1].y))

                a_set_1 = set(boat.boat_cover_area(a_pos_list[0].x, a_pos_list[0].y))
                a_set_2 = set(boat.boat_cover_area(a_pos_list[1].x, a_pos_list[1].y))

                intersection1 = self_set_1.intersection(a_set_1)
                intersection2 = self_set_2.intersection(a_set_2)
                if intersection1 or intersection2:
                    self.collision_boats_id.append(boat.boat_id)

        if len(self.collision_boats_id) > 1:
            return True
        return False

    # 返回一个point的列表，元素是船占的六个格子
    def boat_cover_area(self, x, y):
        cover_list = []
        if transfer_direction(self.dir) == Boat_Direction.DOWN:
            for p in range(x, x + 2+1):
                for q in range(y - 1, y+1):
                    cover_list.append(Point(p, q))
        elif transfer_direction(self.dir) == Boat_Direction.UP:
            for p in range(x - 2, x+1):
                for q in range(y, y + 1+1):
                    cover_list.append(Point(p, q))
        elif transfer_direction(self.dir) == Boat_Direction.LEFT:
            for p in range(x - 1, x + 1):
                for q in range(y - 2, y + 1):
                    cover_list.append(Point(p, q))
        elif transfer_direction(self.dir) == Boat_Direction.RIGHT:
            for p in range (x , x + 1+1):
                for q in range (y, y + 2+1):
                    cover_list.append(Point(p, q))

        return cover_list


    def enable_collision_avoidance(self, master_boat_id: int):
        self.master_boat_id = master_boat_id
        self.original_extended_status = self.extended_status
        self.original_actions_stack = self.actions_stack.copy()
        self.actions_stack = []
        logger.info(f"BOAT {self.boat_id} At zhen: {self.env.global_zhen} enable_collision_avoidance id: {self.boat_id} master_id: {master_boat_id}")
        self.convert_extended_status(Boat_Extended_Status.CollisionAvoidance)

    def try_disable_collision_avoidance(self):
        self.convert_extended_status(self.original_extended_status)
        logger.info(f"BOAT {self.boat_id} At zhen: {self.env.global_zhen} try_disable_collision_avoidance by id: {self.boat_id} at {self.pos}, status: {self.extended_status}, next_3_pos: {self.next_n_pose(3)}")


    def collision_avoidance(self):
        boats = self.env.boats
        okk = True
        # 如果存在会碰撞的船
        if self.collision_check_and_update():
            max_id = max(self.collision_boats_id, key = (lambda id: self.surrounding_boats_with_priority[id]))

            # 如果优先级最高的不是自己，且自己不处于避障状态，自己进入避免碰撞状态
            #                       如果自己已经处于避障状态，更新动作栈
            if self.boat_id != max_id:
                if self.extended_status != Boat_Extended_Status.CollisionAvoidance:
                    logger.info(f"At zhen: {self.env.global_zhen} boat at avoidance: {self.boat_id}")
                    self.enable_collision_avoidance(max_id)
                else:
                    self.action_update()
            # 如果优先级最高的是自己
            else:
                ordered_collision_boats_id = [(boat_id, self.pos.distance(boats[boat_id].pos)) for boat_id in self.collision_boats_id]
                ordered_collision_boats_id.sort(key = lambda x:x[1])
                for id, _ in ordered_collision_boats_id:
                    # 排除自己
                    if id != max_id:
                        # 如果还没进入避免碰撞状态，启动避免碰撞，重新计算路径
                        if boats[id].extended_status != Boat_Extended_Status.CollisionAvoidance:
                            logger.info(f"At zhen: {self.env.global_zhen} boat at avoidance: {id}")
                            logger.info(f"At zhen: {self.env.global_zhen} boat at avoidance: {id}")
                            boats[id].enable_collision_avoidance(max_id)
                        # 如果已经处于避障状态, 该boat可能在避让self 看不见的boat
                        else:
                            # 找到 self 正在避让的船的id
                            self_root_id = self.get_root_id()
                            a_root_id = boats[id]
                            # 如果self 和 a_boat 都处于避让状态，且都避让一个boat，则直接执行避让规划
                            if a_root_id == self_root_id:
                                boats[id].master_boat_id = max_id
                                boats[id].action_update()
                            else:
                                # 否则a正在避让的船也要避让max_id
                                boats[a_root_id].enable_collision_avoidance(max_id)

        return okk


    def try_find_avoidance_action(self):
        boats = self.env.robots
        success = False
        avoidance_actions = []
        # 在进行寻找避让动作时，先更新，self.surrounding, self.collision_boat_id
        if self.collision_check_and_update() == False:
            success = True
            return success, avoidance_actions

        # 避免碰撞时要记录两个变量
        #   当前所在的位置
        #   当前所在的方向
        # 用于恢复避免碰撞后恢复当前状态使用
        self.collision_avoidance_pos = Point(self.x, self.y) # 记录下当前的位置
        self.collision_avoidance_direction = transfer_direction(self.dir) # 记录下当前的方向

        # 如果当前船只在主航道，直接使用 dept 命令让其返回最近的航道，
        # 然后在寻找一条返回的路
        # 寻找这条路可能需要在下一次恢复到正常状态后寻找
        if self.env.attrs_grid[self.x][self.y].is_free_ocean:
            avoidance_actions += ["dept"]
            return success, avoidance_actions

        # 如果当前船只不在主航道
        else:
            avoidance_grid_len = 10 # = alarming_side + predict_steps = 2 + 8 = 10

            # 尝试针对所有周围的对象进行避免碰撞
            predict_steps = 3
            # 这个set用来记录周边不能放的区域
            oqupied_area = set()
            # 将船的位置和其路径视为障碍物
            for boat_id in self.surrounding_boats_with_priority:
                # 不考虑自己
                if boat_id == self.boat_id:
                    continue
                boat = self.env.boats[boat_id]

                # 周围船占的位置
                cover_list = boat.boat_cover_area(boat.x, boat.y)
                for cover in cover_list:
                    oqupied_area.add(cover) # 加入到list里面

            # 遍历当前船的上下左右10个位置，让船驶向最远的位置
            for i in range(boat.x - 10, boat.x + 10):
                for j in range(boat.y - 10, boat.y + 10):
                    if boat.pos.distance(Point(i, j)) > 5 and Point(i, j) not in oqupied_area:
                        avoidance_actions, direction = route_a_star(self.env.attrs_grid, self.env.boat_direction_grid,
                                                                    [boat.pos, transfer_direction(boat.dir)], [Point(i, j), Boat_Direction.ANYWHERE])
                        if direction is not None:
                            success = True
                            break

            return success, avoidance_actions


    def update_extended_status(self):
        '''在规划、执行后进行，在下一帧进行更新；
            当前帧的状态更新放在下一帧的最开始，
            是因为船的状态，下一帧才能被更新'''
        boats = self.env.boats
        # 如果已经到达运输点
        # if self.extended_status == Boat_Extended_Status.OnDeliveryPoint:
        #     # 且动作栈为空，则将状态转换为回港状态
        #     if len(self.actions_stack) == 0:
        #         self.extended_status = Boat_Extended_Status.BackBerthAndLoad
        # 如果正在回港状态，且动作栈为空，则转换为 OnBerth 状态
        if self.extended_status == Boat_Extended_Status.BackBerthAndLoad:
            # 从 BackBerthAndLoad 转移到 OnBerth 状态 要求
            #   当前动作栈为空 and 当前 船的坐标和目标港口的坐标相同
            if len(self.actions_stack) == 0 and self.pos == self.env.berths[self.target_berth_id].pos:
                self.convert_extended_status(Boat_Extended_Status.OnBerth)
        # 如果正在去向运输点的状态，且动作栈为空，则转换为 OnDeliveryPoint 状态
        elif self.extended_status == Boat_Extended_Status.fromBerth2Delivery:
            if len(self.actions_stack) == 0:
                # OnDeliveryPoint 只能自动转入，而转出需要手动操作，这是因为当到达 DeliveryPoint 后需要 人工选择船取到的港口
                self.convert_extended_status(Boat_Extended_Status.OnDeliveryPoint)


    def action_update(self):

        logger.info(f"BOAT {self.boat_id} At zhen {self.env.global_zhen} FUNC: action_update > boat extended status: {self.extended_status}")
        logger.info(f"BOAT {self.boat_id} At zhen {self.env.global_zhen} FUNC: boat_id: {self.boat_id} boat dir: {self.dir}")
        # 如果当前船的状态为 从港口到运输点,
        # 找到船当前在的港口的id
        if self.extended_status == Boat_Extended_Status.fromBerth2Delivery:
            berth_id = -1
            target_delivery_point_id = self.target_delivery_point_id
            for i, berth in enumerate(self.env.berths):
                if self.x == berth.x and self.y == berth.y:
                    berth_id = i
                    break
            # 获取从港口 berth_id 到目标点 Delivery point 的动作
            logger.info(f"BOAT {self.boat_id} berth_id: {berth_id} target_delivery_point_id {target_delivery_point_id}")

            actions = self.env.route_from_berth_to_target[berth_id][target_delivery_point_id]
            logger.info(actions)

            # 将路线添加到动作栈中
            self.actions_stack = actions.copy()

        # 如果船的状态为回港口状态
        # 找到船的当前港口 or 当前出生点 or 当前运输点
        elif self.extended_status == Boat_Extended_Status.BackBerthAndLoad:
            target_berth_id = self.target_berth_id
            berth_id = -1
            purchase_point_id = -1
            delivery_point_id = -1
            for i, berth in enumerate(self.env.berths):
                if self.x == berth.x and self.y == berth.y:
                    berth_id = i
                    break

            if berth_id == -1:
                for j, purchase_point in enumerate(self.env.boat_purchase_point):
                    if self.x == purchase_point.x and self.y == purchase_point.y:
                        purchase_point_id = j
                        break

            if berth_id == -1 and purchase_point_id == -1:
                for k, delivery_point in enumerate(self.env.delivery_point):
                    if self.x == delivery_point.x and self.y == delivery_point.y:
                        delivery_point_id = k
                        break

            # logger.info(f"target_berth_id: {target_berth_id}, berth_id: {berth_id}, purchase_point_id: {purchase_point_id}, delivery_point_id {delivery_point_id}")
            # 从 港口到港口的航线
            if berth_id != -1:
                actions = self.env.route_between_each_berth[berth_id][target_berth_id]
                self.actions_stack = actions.copy()
                # for action in reversed(actions):
                #     self.actions_stack.put(action)
            # 从购买点 到港口
            elif purchase_point_id != -1:
                actions = self.env.route_from_boat_purchase_point_to_each_berth[purchase_point_id][target_berth_id]
                self.actions_stack = actions.copy()
                # for action in reversed(actions):
                #     self.actions_stack.put(action)

                # logger.info("action_update > purchase_point_id > size of actions %d", len(self.actions_stack))

            # 从交货点到港口
            elif delivery_point_id != -1:
                actions = self.env.route_from_target_to_berth[delivery_point_id][target_berth_id]
                self.actions_stack = actions.copy()
                # for action in reversed(actions):
                #     self.actions_stack.put(action)

        # 还没有处理路径规划问题
        elif self.extended_status == Boat_Extended_Status.CollisionAvoidance:
            # 尝试能够回溯的避免碰撞路径
            success, tmp_actions_list = self.try_find_avoidance_action()

            # 如果没有找到避免路径，则新路径的规划只针对负责的对象进行避免
            if not success:
                boats = self.env.boats
                self.collision_check_and_update()
                ordered_surrounding_boats_id = [(boat_id, self.pos.distance(boats[boat_id].pos)) for boat_id in self.surrounding_boats_with_priority]
                ordered_surrounding_boats_id.sort(key=lambda x:x[1])
                for boat_id, _ in ordered_surrounding_boats_id:
                    if self.surrounding_boats_with_priority[boat_id] < self.surrounding_boats_with_priority[self.boat_id]:
                        if boats[boat_id].extended_status != Boat_Extended_Status.CollisionAvoidance:
                            logger.info(f"At zhen: {self.env.global_zhen} at action update id: {self.boat_id}")
                            boats[boat_id].enable_collision_avoidance(self.boat_id)
                        else:
                            boats[boat_id].action_update()
                success, tmp_actions_list = self.try_find_avoidance_action()
                # 尝试能够回溯的避免碰撞路径
                if success:
                    self.gen_recoverable_path(tmp_actions_list)
        elif self.extended_status == Boat_Extended_Status.OnBerth:
            self.actions_stack = []

        elif self.extended_status == Boat_Extended_Status.OnDeliveryPoint:
            self.actions_stack = []


        logger.info(f"BOAT {self.boat_id} At zhen: {self.env.global_zhen} Func: action_update > by id: {self.boat_id} at {self.pos}, status: {self.extended_status}, next3pos: {self.next_n_pose(3)}")


        return True

def transfer_direction(boat_dir):
    if boat_dir == "R":
        return Boat_Direction.RIGHT
    elif boat_dir == "L":
        return Boat_Direction.LEFT
    elif boat_dir == "U":
        return Boat_Direction.UP
    elif boat_dir == "D":
        return Boat_Direction.DOWN
    else:
        return ""

