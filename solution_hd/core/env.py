from __future__ import annotations
import multiprocessing
import concurrent.futures
from typing import List, Tuple
from dataclasses import dataclass
from queue import PriorityQueue

from cfg import N
from log import func_timer, main_logger, route_logger, logger
from path_planing.base import Point, Point, Pixel_Attrs
from path_planing import Boat_Direction, Boat_Action
from path_planing import robot_bfs, boat_bfs, route_a_star

from .robot import Robot
from .berth import Berth
from .boat import Boat, cmd_boatDirection_map

from concurrent.futures import ProcessPoolExecutor

@dataclass
class Env:

    boat_price:int =8000
    robot_price:int =2000

    robot_num:int  = 0

    money:int = 0
    boat_capacity:int = 0

    
    def __init__(self):
        # self.interact_stream = My_Stream( out_file= 'output.txt')

        # 各种类型的grid，字符，属性，物品
        self.ch_grid: List[List[str]] = [[' ' for _ in range(N)] for _ in range(N)]
        self.attrs_grid: List[List[Pixel_Attrs]] = [[Pixel_Attrs() for _ in range(N)] for _ in range(N)]
        self.gds_grid:List[List[int]] = [[0 for _ in range(N)] for _ in range(N)]
        self.boat_direction_grid:List[List[ (List[str]) ]] = [[ [] for _ in range(N)] for _ in range(N)]

        # 机器人购买点、船购买点、运输点、
        self.robot_purchase_point: List[Point] = []
        self.boat_purchase_point: List[Point] = []  # 轮船购买点的位置
        self.delivery_point: List[Point] = []       # 运输点的位置


        
        self.berths:List[Berth] = [] # 港口的位置
        self.robots:List[Robot] = []
        self.boats:List[Boat] = []

        # 轮船购买点 到 港口之间的航线
        # 是一个 二维数组
        # route_between_boat_purchase_point_and_each_berth[i][j] 中存储的是 第 i 个生成点 到 第 j 个港口的航线，航线表示为相应的动作列表
        # 初始时由于不知道具体的数量，我们暂时不做处理
        self.route_from_boat_purchase_point_to_each_berth = []

        # 每个港口之间的航线
        # route_between_each_berth[i][j] 表示 第 i 个 港口 到 第 j 个港口的动作列表
        self.route_between_each_berth = []

        # 每个港口到运输点的航线
        # route_from_berth_to_target[i][j] 表示第 i 个港口到 第 j 个目标点的动作列表
        self.route_from_berth_to_target = []

        # 每个运输点到每个港口的航线
        # route_from_target_to_berth[i][j] 表示第 i 个目标点到 第 j 个港口的动作列表
        self.route_from_target_to_berth = []

        # 全局帧
        self.global_zhen_ref:List[int] = [0]

    @property
    def global_zhen(self):
        return self.global_zhen_ref[0]
    @global_zhen.setter
    def global_zhen(self, value):
        self.global_zhen_ref[0] = value
    
    @property
    def left_zhen(self):
        '''剩余可以操作的帧数'''
        return 15001 - self.global_zhen_ref[0]
        # 计算方式：15000 - 当前帧 + 1，
    
    def process_map_input(self):
        '''
        用处：处理判题器的初始输入地图，生成字符网格和属性网格（每个元素表示某个像素持有的属性）
        '''
        for x in range(N):
            line: str = input()
            for y, ch in enumerate(line):
                
                # 处理字符网格
                self.ch_grid[x][y] = ch

                # 处理属性网格
                if ch == '.':
                    self.attrs_grid[x][y].is_ground = True
                elif ch == '>' or ch == 'R':
                    self.attrs_grid[x][y].is_ground = True
                    self.attrs_grid[x][y].is_free_ground = True
                elif ch == '*':
                    self.attrs_grid[x][y].is_ocean = True
                elif ch == '~' or ch == 'S' or ch == 'K' or ch == 'T':
                    self.attrs_grid[x][y].is_ocean = True
                    self.attrs_grid[x][y].is_free_ocean = True
                    if ch == '~': # 如果是主航道
                        self.attrs_grid[x][y].is_main_channel = True
                    elif ch == 'S': # 如果是船舶购买地块
                        self.attrs_grid[x][y].is_purchase_ship_position = True
                        self.attrs_grid[x][y].is_main_channel = True
                    elif ch == 'K': # 如果是靠泊区域
                        self.attrs_grid[x][y].is_mooring = True
                        self.attrs_grid[x][y].is_main_channel = True
                    elif ch == 'T': # 如果是交货点
                        self.attrs_grid[x][y].is_target = True
                        self.attrs_grid[x][y].is_main_channel = True
                elif ch == 'B' or ch == 'c':
                    self.attrs_grid[x][y].is_ground = True
                    self.attrs_grid[x][y].is_free_ground = True
                    self.attrs_grid[x][y].is_ocean = True
                    self.attrs_grid[x][y].is_free_ocean = True
                    if ch == 'B': # 如果是泊位
                        self.attrs_grid[x][y].is_main_channel = True
                        self.attrs_grid[x][y].is_berth = True
                    elif ch == 'c':
                        self.attrs_grid[x][y].is_main_channel = True


                elif ch == 'C':
                    self.attrs_grid[x][y].is_ground = True
                    self.attrs_grid[x][y].is_ocean = True
                else: # '#'
                    pass
                
                # 
                if ch == 'R':
                    self.robot_purchase_point.append(Point(x, y))
                elif ch == 'S':
                    self.boat_purchase_point.append(Point(x, y))
                elif ch == 'T':
                    self.delivery_point.append(Point(x, y))
    
    def robot_bfs(self):
        for berth in self.berths:
            p = multiprocessing.Process(target=robot_bfs, args=(self.attrs_grid, berth.pos))
            berth.robot_move_grid, berth.robot_cost_grid  = robot_bfs(self.attrs_grid, berth.pos)
            # from path_planing import apply_move_grid_to_ch_grid, save_grid_to_file
            # save_grid_to_file(apply_move_grid_to_ch_grid(self.ch_grid, berth.robot_move_grid), 'move')
    
    @func_timer
    def robot_bfs_multi_core(self):
        results: List[Tuple[ (List[List[Point]]), (List[List[int]])]] = []
        with concurrent.futures.ProcessPoolExecutor(2) as executor:
            future_results = [executor.submit(robot_bfs, self.attrs_grid, berth.pos) for berth in self.berths]
            results = [future.result() for future in concurrent.futures.as_completed(future_results)]
        for i, berth in enumerate(self.berths):
            berth.robot_move_grid, berth.robot_cost_grid = results[i]
            # from path_planing import apply_move_grid_to_ch_grid, save_grid_to_file
            # save_grid_to_file(apply_move_grid_to_ch_grid(self.ch_grid, berth.robot_move_grid), 'move')

    @func_timer
    def gen_boat_direction_grid(self):
        def can_place_down(x, y):
            if x + 2 >= N or y - 1 < 0 or x + 1 >= N:
                return False
            for p in range(x, x + 2+1):
                for q in range(y - 1, y+1):
                    if self.attrs_grid[p][q].is_ocean == False:
                        return False
            return True
        def can_place_up(x, y):
            if y + 1 >= N or x - 2 < 0 or x - 1 < 0:
                return False
            for p in range(x - 2, x+1):
                for q in range(y, y + 1+1):
                    if self.attrs_grid[p][q].is_ocean == False:
                        return False
            return True
        def can_place_left(x, y):
            if x - 1 < 0 or y - 1 < 0 or y - 2 < 0:
                return False
            for p in range(x - 1, x+1):
                for q in range(y - 2, y+1):
                    if self.attrs_grid[p][q].is_ocean == False:
                        return False
            return True
        def can_place_right(x, y):
            if x + 1 >= N or y + 1 >= N or y + 2 >= N:
                return False
            for p in range (x , x + 1+1):
                for q in range (y, y + 2+1):
                    if self.attrs_grid[p][q].is_ocean == False:
                        return False
            return True
        
        for i in range(N):
            for j in range(N):
                directions: List[str] = []
                if self.attrs_grid[i][j].is_ocean:
                    if can_place_down(i, j): directions.append(Boat_Direction.DOWN)
                    if can_place_right(i, j): directions.append(Boat_Direction.RIGHT)
                    if can_place_up(i, j): directions.append(Boat_Direction.UP)
                    if can_place_left(i, j): directions.append(Boat_Direction.LEFT)
                self.boat_direction_grid[i][j] = directions

    @func_timer
    def boat_bfs(self):
        self.gen_boat_direction_grid()
        T = Point(93, 71)
        D = Boat_Direction.DOWN
        self.boat0_actions = boat_bfs(self.attrs_grid, self.boat_direction_grid, 
                                      (self.boat_purchase_point[0], cmd_boatDirection_map[0]), (T, D))
        main_logger.error(self.boat0_actions)

    @func_timer
    def init_route(self):



        # ``` 首先初始化 轮船购买点 到 各个港口的航线 ```
        # 确保外层列表有足够的空间存储所有轮船购买点到港口的航线
        # logger.info("func init_route: size of boat_purchase_point is %d", len(self.boat_purchase_point))
        # logger.info("func init_route: size of berths is %d", len(self.berths))
        @func_timer
        def init_route_from_boat_purchase_point_to_each_berth():
            for bpp_index, bpp in enumerate(self.boat_purchase_point):
                # 如果当前索引超出现有列表的长度，添加一个新的空列表
                if bpp_index >= len(self.route_from_boat_purchase_point_to_each_berth):
                    self.route_from_boat_purchase_point_to_each_berth.append([])

                for berth_index, berth_position in enumerate(self.berths):
                    target_position = Point(berth_position.x, berth_position.y)
                    # logger.info("func init_route: boat_purchase_point: %d %d, target: %d %d", bpp.x, bpp.y, target_position.x, target_position.y)
                    # 轮船在购买点的方向始终向右，轮船在港口的方向任意
                    actions, target_direction = route_a_star(self.attrs_grid, self.boat_direction_grid,
                                           (bpp, cmd_boatDirection_map[0]),
                                           (target_position, cmd_boatDirection_map[4]))
                    logger.info("from purchase %d to berth %d", bpp_index, berth_index)
                    logger.info(actions)
                    # 为当前轮船购买点到港口的航线添加动作列表
                    # 由于可能存在多个港口，确保每个购买点的列表也有足够的空间
                    if berth_index >= len(self.route_from_boat_purchase_point_to_each_berth[bpp_index]):
                        self.route_from_boat_purchase_point_to_each_berth[bpp_index].append(actions)
                    else:
                        self.route_from_boat_purchase_point_to_each_berth[bpp_index][berth_index] = actions

        # 接着初始化各个港口之间的航线
        @func_timer
        def init_route_between_each_berth():
            num_berths = len(self.berths)

            # 初始化二维数组，先将所有的元素设置为None或空列表
            self.route_between_each_berth = [[None for _ in range(num_berths)] for _ in range(num_berths)]

            for i, start_berth in enumerate(self.berths):
                for j, end_berth in enumerate(self.berths):
                    if i == j:
                        # 同一个港口，没有航线，设为空
                        self.route_between_each_berth[i][j] = []
                        continue

                    # 获取起始和结束港口的位置
                    start_position = Point(start_berth.x, start_berth.y)
                    end_position = Point(end_berth.x, end_berth.y)

                    # 调用路径规划函数来计算从一个港口到另一个港口的航线
                    actions, target_direction = route_a_star(self.attrs_grid, self.boat_direction_grid,
                                           (start_position, start_berth.berth_direction),  # 这里我们使用船在泊位上的方向
                                           (end_position, cmd_boatDirection_map[4]))  # 这里我们不规定最终的方向


                    # logger 一下
                    # logger.info("from berth %d to berth %d", i, j)
                    # logger.info(actions)

                    self.route_between_each_berth[i][j] = actions

        # 最后初始化 运输点 与 港口之间的航线
        @func_timer
        def init_route_between_berth_and_target():

            # 确定港口和运输点的数量
            num_berths = len(self.berths)
            num_targets = len(self.delivery_point)

            # 初始化二维数组以存储每个港口到每个运输点的航线
            self.route_from_berth_to_target = [[None for _ in range(num_targets)] for _ in range(num_berths)]
            self.route_from_target_to_berth = [[None for _ in range(num_berths)] for _ in range(num_targets)]

            # 遍历所有港口
            for i, berth in enumerate(self.berths):
                # 获取港口位置
                start_position = Point(berth.x, berth.y)

                # 遍历所有运输点
                for j, target in enumerate(self.delivery_point):
                    # 获取运输点位置
                    end_position = Point(target.x, target.y)

                    # 调用路径规划函数来计算航线
                    # 注意：这里我们假设船在港口的起始方向和在运输点的结束方向是已知的
                    # 如果这些方向不是固定的，你需要根据实际情况进行调整
                    actions_to_target, to_target_direction = route_a_star(self.attrs_grid, self.boat_direction_grid,
                                                             (start_position, berth.berth_direction),
                                                             (end_position, Boat_Direction.ANYWHERE))

                    # 将计算出的航线添加到二维数组中
                    self.route_from_berth_to_target[i][j] = actions_to_target


                    actions_to_berth, to_berth_direction = route_a_star(self.attrs_grid, self.boat_direction_grid,
                                                             (end_position, to_target_direction),
                                                             (start_position, Boat_Direction.ANYWHERE))

                    # 从第ij个 target 到 第 i 个 berth 之间的航线
                    self.route_from_target_to_berth[j][i] = actions_to_berth

                    # 如果需要，可以记录航线信息
                    # logger.info(f"Route from berth {i} to target {j}: {actions_to_target}")
                    # logger.info(f"Route from target {j} to berth {i}: {actions_to_berth}")




        # 首先初始化船的方向格子
        self.gen_boat_direction_grid()

        # 接着初始化轮船生成点到各个港口之间的航线
        init_route_from_boat_purchase_point_to_each_berth()

        # 初始化各个港口之间的航线
        init_route_between_each_berth()

        # 初始化 港口和运输点之间的航线
        init_route_between_berth_and_target()

    @func_timer
    def multicore_init_route_between_berth_and_target(self):
        num_berths = len(self.berths)
        num_targets = len(self.delivery_point)

        # 初始化二维数组以存储每个港口到每个运输点的航线
        self.route_from_berth_to_target = [[None for _ in range(num_targets)] for _ in range(num_berths)]
        self.route_from_target_to_berth = [[None for _ in range(num_berths)] for _ in range(num_targets)]

        tasks = []

        # 构建任务列表
        for i, berth in enumerate(self.berths):
            for j, target in enumerate(self.delivery_point):
                tasks.append((i, j, berth, target))

        # 使用进程池执行任务
        with ProcessPoolExecutor(max_workers=2) as executor:
            futures = {executor.submit(self.calculate_route, task): task for task in tasks}

            for future in concurrent.futures.as_completed(futures):
                i, j, _, _ = futures[future]
                actions_to_target, actions_to_berth = future.result()
                self.route_from_berth_to_target[i][j] = actions_to_target
                self.route_from_target_to_berth[j][i] = actions_to_berth

    def calculate_route(self, task):
        i, j, berth, target = task
        start_position = Point(berth.x, berth.y)
        end_position = Point(target.x, target.y)

        # 调用路径规划函数来计算航线
        actions_to_target, to_target_direction = route_a_star(self.attrs_grid, self.boat_direction_grid,
                                                              (start_position, berth.berth_direction),
                                                              (end_position, Boat_Direction.ANYWHERE))

        actions_to_berth, to_berth_direction = route_a_star(self.attrs_grid, self.boat_direction_grid,
                                                            (end_position, to_target_direction),
                                                            (start_position, Boat_Direction.ANYWHERE))

        return actions_to_target, actions_to_berth


    @func_timer
    def init_env(self):

        # 初始化各种 字符网格和属性网格
        self.process_map_input()

        # 初始化 berths
        self.berth_num = int(input())
        for _ in range(self.berth_num):
            berth_id, x, y, loading_speed = map(int, input().split())
            berth = Berth(berth_id, self, x, y, loading_speed)
            self.berths.append(berth)
            self.berths[berth_id].get_berth_direction()
            # logger.info("berth %d: %d %d, direction: %s", berth_id, self.berths[berth_id].x, self.berths[berth_id].y, self.berths[berth_id].berth_direction )

        # 初始化 船只容量
        self.boat_capacity = int(input())


        # 初始化输入结束
        okk = input()
        
        self.robot_bfs_multi_core()
        # self.boat_bfs()

        # 初始化航线
        self.init_route()
        self.multicore_init_route_between_berth_and_target()
        self.boat0_actions = self.route_from_boat_purchase_point_to_each_berth[0][1]
        # logger.info("berth id %d: %d %d", 1, self.berths[1].x, self.berths[1].y)

    
    #@func_timer
    def input(self):
        # 帧、金币更新
        self.global_zhen, self.money = map(int, input().split())

        # 物品更新
        self.goods_num = int(input())
        for i in range(self.goods_num):
            x, y, val = map(int, input().split())
            self.gds_grid[x][y] = val
        
        # 机器人更新
        self.robot_num = int(input())
        # 动态扩充新买的机器人
        while (len(self.robots) < self.robot_num):
            self.robots.append(Robot(-1, self))
        for robot in self.robots:
            robot.robot_id, robot.goods, robot.x, robot.y = map(int, input().split())

        # 船只更新
        self.boat_num = int(input())
        while (len(self.boats) < self.boat_num):
            self.boats.append(Boat(self))
        for boat in self.boats:
            boat.boat_id, boat.goods_num, boat.x, boat.y, boat.dir, boat.status = map(int, input().split())
        
        # 判题器更新结束
        okk = input()


                    
# class My_Stream():

#     def __init__(self, in_file = None, out_file = None) -> None:

#         if in_file:
#             self.in_stream = open(in_file, 'r')
#         else:
#             self.in_stream = sys.stdin
        
#         if out_file:
#             self.out_stream = open(out_file, 'w+')
#         else:
#             self.out_stream = sys.stdout

#     def input(self):
#         # return input()
#         return self.in_stream.readline().rstrip('\n')

#     def print(self, value: str):
#         if self.out_stream:
#             print(value, file = self.out_stream)
#         print(value)

#     def flush(self):
#         sys.stdout.flush()