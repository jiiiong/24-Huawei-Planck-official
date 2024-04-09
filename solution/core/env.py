from __future__ import annotations
import multiprocessing
import concurrent.futures
from typing import List, Tuple, Dict, Callable, Set
from dataclasses import dataclass
from queue import PriorityQueue
import copy

from cfg import N
from log import func_timer, main_logger
from path_planing import Point, Point, Pixel_Attrs, sVec
from path_planing import Boat_Direction, Boat_Action
from path_planing import robot_bfs, boat_bfs, boat_actions_to_sVecs, save_grid_to_file, boat_actions_to_poses, boat_actions_to_poses_per_action

from .robot import Robot
from .berth import Berth
from .boat import Boat, int_boatDirection_map, Boat_Route_Lock

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
        self.boat_valid_grid:List[List[ (List[Boat_Direction]) ]] = [[ [] for _ in range(N)] for _ in range(N)]

        # 机器人购买点、船购买点、运输点、
        self.robot_purchase_point: List[Point] = []
        self.boat_purchase_sVec_list: List[sVec] = []
        self.delivery_point: List[Point] = []
        self.delivery_sVec_list: List[sVec] = []
        
        self.berths:List[Berth] = []
        self.robots:List[Robot] = []
        self.boats:List[Boat] = []

        self.boat_route_dict: Dict[Tuple[sVec, sVec], List[str]] = {}
        self.lock_dict: Dict[int, Boat_Route_Lock] = {}
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
                elif ch == 'B' or ch == 'c':
                    self.attrs_grid[x][y].is_ground = True
                    self.attrs_grid[x][y].is_free_ground = True
                    self.attrs_grid[x][y].is_ocean = True
                    self.attrs_grid[x][y].is_free_ocean = True
                elif ch == 'C':
                    self.attrs_grid[x][y].is_ground = True
                    self.attrs_grid[x][y].is_ocean = True
                else: # '#'
                    pass
                
                # 
                if ch == 'R':
                    self.robot_purchase_point.append(Point(x, y))
                elif ch == 'S':
                    self.boat_purchase_sVec_list.append(sVec(Point(x, y), Boat_Direction.RIGHT))
                elif ch == 'T':
                    self.delivery_point.append(Point(x, y))
    
    # def robot_bfs(self):
    #     for berth in self.berths:
    #         p = multiprocessing.Process(target=robot_bfs, args=(self.attrs_grid, berth.pos))
    #         berth.robot_move_grid, berth.robot_cost_grid  = robot_bfs(self.attrs_grid, berth.pos)
    #         # from path_planing import apply_move_grid_to_ch_grid, save_grid_to_file
    #         # save_grid_to_file(apply_move_grid_to_ch_grid(self.ch_grid, berth.robot_move_grid), 'move')
    
    @func_timer
    def robot_bfs_multi_core(self):
        with concurrent.futures.ProcessPoolExecutor(2) as executor:
            future_results = [(executor.submit(robot_bfs, berth.berth_id, self.attrs_grid, berth.pos)) for berth in self.berths]
            results = [ future.result() for future in concurrent.futures.as_completed(future_results)]
        for result in results:
            berth_id, robot_move_grid, robot_cost_grid = result
            for berth in self.berths:
                if berth_id == berth.berth_id:
                    berth.robot_cost_grid = robot_cost_grid
                    berth.robot_move_grid = robot_move_grid

    @func_timer
    def gen_boat_valid_grid(self):
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
                directions: List[Boat_Direction] = []
                if self.attrs_grid[i][j].is_ocean:
                    if can_place_down(i, j): directions.append(Boat_Direction.DOWN)
                    if can_place_right(i, j): directions.append(Boat_Direction.RIGHT)
                    if can_place_up(i, j): directions.append(Boat_Direction.UP)
                    if can_place_left(i, j): directions.append(Boat_Direction.LEFT)
                self.boat_valid_grid[i][j] = directions

    @func_timer
    def boat_bfs_multi_core(self):
        self.gen_boat_valid_grid()

        # 处理交货！！！！！！！！！！！！！！！！！！！！！！！！可能导致无法离开
        for pos in self.delivery_point:
            self.delivery_sVec_list.append(sVec(pos, self.boat_valid_grid[pos.x][pos.y][0]))

        self.route_ids: List[Tuple[sVec, sVec]] = []
        # 生成S到B的路径
        for S_sVec in self.boat_purchase_sVec_list:
            for berth in self.berths:
                self.route_ids.append((S_sVec, berth.sVec))
                break
            break
        # for berth_A in self.berths:
        #     for berth_B in self.berths:
        #         self.route_ids.append((berth_A.sVec, berth_B.sVec))
        #         self.route_ids.append((berth_B.sVec, berth_A.sVec))
        #         # break
        #     # break
        for T_sVec in self.delivery_sVec_list:
            for berth in self.berths:
                self.route_ids.append((T_sVec, berth.sVec))
                self.route_ids.append((berth.sVec, T_sVec))
                break
            break

        # for id_a, T_sVec in enumerate(self.delivery_sVec_list):
        #     for id_b, berth in enumerate(self.berths):
        #         if id_a == 0 and id_b == 3:
        #             # task_ids.append((T_sVec, berth.sVec))
        #             task_ids.append((berth.sVec, T_sVec))
        
        self.lock_grid: List[List[Boat_Route_Lock]] = [[ Boat_Route_Lock() for _ in range(N)] for _ in range(N)]
        for i, id in enumerate(self.route_ids):
            _, self.boat_route_dict[id] = boat_bfs(id, self.attrs_grid, self.boat_valid_grid, id[0], id[1], self.lock_grid)
            # sVec_list = boat_actions_to_sVecs(id[0], self.boat_route_dict[id])
            # 划分航道
            self.update_lock_grid(id, i)
        # 划分为区块
        main_logger.error(f"''{self.union_lock_grid()}")
        
        # 生成拓扑图，采用邻接矩阵
        self.gen_boat_route_topylogy_graph()
        main_logger.error(self.boat_route_topology_graph)
        
        # 使用matplot打印可视化的道路网格
        self.visualize_lock_grid()

        self.test_route = self.route_ids

        # # 打印lock_grid
        # test_grid = copy.deepcopy(self.ch_grid)
        # for x, line in enumerate(self.lock_grid):
        #     for y, lock in enumerate(line):
        #         if lock.num_shared_roads > 0:
        #             main_logger.error(f"{x} {y} {lock.route_id}")

        # 尝试打印航道
        # self.boat_route_grid: List[List[List[int]]] = []
        # for id in task_ids:
        #     # 产生path
        #     self.print_boat_route_id(id)

        
        # 多进程版本
        # with concurrent.futures.ProcessPoolExecutor(1) as executor:
        #     future_results = [executor.submit(boat_bfs, id, self.attrs_grid, self.boat_valid_grid, id[0], id[1]) for id in task_ids]
        #     results = [ future.result() for future in concurrent.futures.as_completed(future_results
        # for result in results:
        #     id, actions = result
        #     self.boat_route_dict[id] = actions
        # self.test_route = task_ids
        
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

        # 初始化 船只容量
        self.boat_capacity = int(input())

        # 初始化输入结束
        okk = input()
        
        self.robot_bfs_multi_core()
        self.boat_bfs_multi_core()
    
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
            self.boats.append(Boat(-1, self))
        for boat in self.boats:
            boat.boat_id, boat.goods_num, boat.x, boat.y, boat.dir, boat.status = map(int, input().split())
        
        # 判题器更新结束
        okk = input()

    @func_timer
    def print_boat_route_id(self, id: Tuple[sVec, sVec]):
        _ch_grid = copy.deepcopy(self.ch_grid)
        poses = boat_actions_to_poses(id[0], self.boat_route_dict[id])
        for pos in poses:
            _ch_grid[pos.x][pos.y] = '@'
        save_grid_to_file(_ch_grid, "boat_route")

    @func_timer
    def update_lock_grid(self, id: Tuple[sVec, sVec], id_index: int):
        poses = boat_actions_to_poses(id[0], self.boat_route_dict[id])
        # 一段路占据的不重复的位置
        for pos in poses:
            self.lock_grid[pos.x][pos.y].update(str(100+id_index))

    @func_timer
    def union_lock_grid(self):
        # 解决道路被切割的问题
        for route_id in self.route_ids:
            previous_block_ids: Dict[str, int] = {}
            last_block_ids: Set[str] = set()
            poses_per_action_list = boat_actions_to_poses_per_action(route_id[0], self.boat_route_dict[route_id])
            # 遍历每一次action后所占的位置
            for poses_per_action in poses_per_action_list:
                cur_block_ids: Set[str] = set()
                # 求出这些位置使用的block id
                for pos in poses_per_action:
                    cur_block_ids.add(self.lock_grid[pos.x][pos.y].route_id)
                for cur_block_id in cur_block_ids:
                    # 如果还占据当前区块
                    if cur_block_id in last_block_ids:
                        pass
                    else:
                        # 如果到达一个全新的区块
                        if cur_block_id not in previous_block_ids:
                            previous_block_ids[cur_block_id] = 0
                        # 如果之前到达过
                        else:
                            previous_block_ids[cur_block_id] += 1

                for pos in poses_per_action:
                    route_id = self.lock_grid[pos.x][pos.y].route_id
                    if route_id in previous_block_ids:
                        self.lock_grid[pos.x][pos.y].times = previous_block_ids[route_id]
                
                last_block_ids = cur_block_ids
                # to_new_block = False
                # for cur_block_id in cur_block_ids:
                #     if cur_block_id in last_block_ids:
                #         continue
                #     else:
                #         to_new_block = True
                #         break

                # if to_new_block:
                #     for cur_block_id in cur_block_ids:
                #         if cur_block_id not in previous_block_ids:
                #             previous_block_ids[cur_block_id] = 0
                #         # 如果之前到达过
                #         else:
                #             previous_block_ids[cur_block_id] += 1

                # for pos in poses_per_action:
                #     route_id = self.lock_grid[pos.x][pos.y].route_id
                #     if to_new_block:
                #         self.lock_grid[pos.x][pos.y].times = previous_block_ids[route_id]
                # last_block_ids = cur_block_ids


        for x, line in enumerate(self.lock_grid):
            for y, lock in enumerate(line):
                self.lock_grid[x][y].route_id = self.lock_grid[x][y].route_id + 'T' * self.lock_grid[x][y].times
            
                
        lock_dict: Dict[str, Boat_Route_Lock] = {}
        for x, line in enumerate(self.lock_grid):
            for y, lock in enumerate(line):
                route_id = self.lock_grid[x][y].route_id
                if route_id in lock_dict:
                    self.lock_grid[x][y] = lock_dict[route_id]
                else:
                    lock_dict[route_id] = lock

        return len(lock_dict)

    @func_timer
    def gen_boat_route_topylogy_graph(self):
        # 重新规定编号
        unique_id: int = 0
        for line in self.lock_grid:
            for lock in line:
                if lock.unique_route_id == -1:
                    lock.unique_route_id = unique_id
                    self.lock_dict[unique_id] = lock
                    unique_id += 1
                    
        save_grid_to_file(self.lock_grid)
        self.boat_route_topology_graph = [[ False for _ in range(unique_id)] for _ in range(unique_id)]
        for x in range(unique_id):
            self.boat_route_topology_graph[x][x] = True


        for route_id in self.route_ids:
            last_unique_route_ids: Set[int] = set()
            poses_per_action_list = boat_actions_to_poses_per_action(route_id[0], self.boat_route_dict[route_id])
            # 遍历每一次action后所占的位置
            for poses_per_action in poses_per_action_list:
                cur_unique_route_ids: Set[int] = set()
                # 求出这些位置使用的block id
                for pos in poses_per_action:
                    cur_unique_route_ids.add(self.lock_grid[pos.x][pos.y].unique_route_id)

                for cur_unique_route_id in cur_unique_route_ids:
                    if cur_unique_route_id in last_unique_route_ids:
                        pass
                    else:
                        for last_uni_route_id in last_unique_route_ids:
                            self.boat_route_topology_graph[last_uni_route_id][cur_unique_route_id] = True
                last_unique_route_ids = cur_unique_route_ids
        # save_grid_to_file(self.boat_route_topology_graph)

    def visualize_lock_grid(self):
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
        colors = ['blue', 'green', 'red', 'cyan', 'magenta', 'yellow', 'orange', 'purple', 'brown', 'black']



        # 创建 ListedColormap 对象
        cmap = ListedColormap(colors)

        unique_id = 1
        id_int_map = {}
        grid = [[0 for _ in range(N)] for _ in range(N)]
        for x, line in enumerate(self.lock_grid):
            for y, lock in enumerate(line):
                route_id = lock.route_id
                if 1:#'100' in route_id:
                    if route_id in id_int_map:
                        grid[x][y] = id_int_map[route_id]
                    else:
                        id_int_map[route_id] = unique_id
                        grid[x][y] = id_int_map[route_id]
                        unique_id += 1
        plt.imshow(grid, cmap=cmap, interpolation='nearest')
        plt.colorbar()  # 添加颜色条
        plt.savefig('fg.png')

        
    
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