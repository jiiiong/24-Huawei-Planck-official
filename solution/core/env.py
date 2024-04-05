from __future__ import annotations
import multiprocessing
import concurrent.futures
from typing import List, Tuple
from dataclasses import dataclass
from queue import PriorityQueue

from cfg import N
from log import func_timer, main_logger
from path_planing import Point, Point, Pixel_Attrs
from path_planing import Boat_Direction, Boat_Action
from path_planing import robot_bfs, boat_bfs

from .robot import Robot
from .berth import Berth
from .boat import Boat, transfer_direction
        
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
        self.boat_purchase_point: List[Point] = []
        self.delivery_point: List[Point] = []
        
        self.berths:List[Berth] = []
        self.robots:List[Robot] = []
        self.boats:List[Boat] = []
        
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
        T = Point(2, 195)
        self.boat0_actions = boat_bfs(self.attrs_grid, self.boat_direction_grid, self.boat_purchase_point[0], transfer_direction(0), T)
        main_logger.error(self.boat0_actions)

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
        self.boat_bfs()
    
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
            self.boats.append(Boat())
        for boat in self.boats:
            boat.id, boat.goods_num, boat.x, boat.y, boat.dir, boat.status = map(int, input().split())
        
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