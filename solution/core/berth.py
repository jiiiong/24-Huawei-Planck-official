from __future__ import annotations
from typing import List,Tuple
from queue import PriorityQueue



from log import scheduler_logger,goods_logger

from core import Goods
from log import logger
from cfg import TYPE_CHECKING
if TYPE_CHECKING:
    from .env import Env
from path_planing import Point, sVec
from path_planing import Boat_Direction

class Berth:
    def __init__(self, berth_id: int, env: Env, x=0, y=0, loading_speed=0):
        self.berth_id: int = berth_id
        self.pos = Point(x, y)
        self.loading_speed = loading_speed
        self.env: Env = env
        self.gds_priority_queue = PriorityQueue()

        # 统计港口收集的货物用
        self.cur_num_gds = 0
        self.total_num_gds = 0
        self.num_allocated_gds = 0
        self.num_losed_gds = 0
        self.total_earn = 0
        self.desired_value = 0
        self.earn_when_n: List[int] = [0, 0, 0]
        self.earn_when_n_comsuming_time: List[int] = [0, 0, 0, 0]

        #self.total_value_of_allocated_goods = 0
        self.robot_cost_grid: List[List[int]]   = []
        self.robot_move_grid: List[List[Point]] = []

        self.increase_rate = 0
        self.friend_berths: List[Berth] = []
        
        # 初始化计算港口的dir
        potential_oppsite_points = {Boat_Direction.UP: Point(-2, 1),
                                    Boat_Direction.RIGHT: Point(1, 2),
                                    Boat_Direction.DOWN: Point(2, -1),
                                    Boat_Direction.LEFT: Point(-1, -2),
                                    }
        for kv in potential_oppsite_points.items():
            oppsite_point = self.pos + kv[1]
            if oppsite_point.is_in_grid() and self.env.ch_grid[oppsite_point.x][oppsite_point.y] == 'B':
                self.dir = kv[0]
                break
        
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
    def lost_rate(self):
        return self.num_losed_gds / (self.num_allocated_gds + 1)

    def clear_queue(self):
        pq = self.gds_priority_queue
        elements = []
        total_cost_available_goods = 0
        while not pq.empty():
            item: Tuple[float, Goods] = pq.get()
            if (item[1].elapsed_zhen < 1000):
                elements.append(item)
                total_cost_available_goods += item[1].cost
            else:
                self.num_losed_gds += 1
        self.total_cost_available_goods = total_cost_available_goods
        for item in elements:
            pq.put(item)
            
        # # 计算该队列在只有n个机器人时能够赚多少
        for i, _ in enumerate(self.earn_when_n):
            self.earn_when_n[i] = self.cal_earn_berfore_end_when_n_robots(elements, i+1)

    def add_goods(self, goods: Goods):
        cost = self.robot_cost_grid[goods.x][goods.y]
        goods.cost = cost
        self.gds_priority_queue.put((-goods.price/(2 * cost), goods))
        self.desired_value += goods.price/(2 * cost + 1)
        # self.total_value_of_allocated_goods += goods.price
        # self.total_cost_available_goods +=  cost

    def fetch_goods(self) -> Tuple[bool, Goods] : 
        success = False
        fetched_goods: Goods = Goods(-1, [0], Point(), 0) # 无效的，只是为了注释正确
        robot_cost_grid = self.robot_cost_grid
        # 尝试在自己的优先级队列中取物品
        if not self.gds_priority_queue.empty():
            fetched_goods= self.gds_priority_queue.get(False)[1]
            cost = robot_cost_grid[fetched_goods.x][fetched_goods.y]
            while ( (fetched_goods.fetched == True 
                     or (fetched_goods.left_zhen < cost + 1)
                     or (cost > (248 - (1/(8 * self.cal_increase_rate())))))
                   and not self.gds_priority_queue.empty()):
                if fetched_goods.elapsed_zhen >= 1000:
                    self.num_losed_gds += 1
                fetched_goods = self.gds_priority_queue.get(False)[1]
            # 如果能取到还未被取的
            if fetched_goods.fetched == False:
                success = True

        # 如果没有在当前队列取到货，则尝试帮朋友港口取会过期的货
        if success == False:
            # 用于筛选最优的friend berths会丢失的货
            best_losing_gds_queue: PriorityQueue = PriorityQueue()
            
            # 用于暂存所有friend berths的优先级队列的队列
            pq_list_list: List[List[Tuple[float, Goods]]] = []            
            for i, friend_berth in enumerate(self.friend_berths):
                # 当前friend berth的优先级队列
                pq_list: List[Tuple[float, Goods]] = []
                pq = friend_berth.gds_priority_queue
                while not pq.empty():
                    item: Tuple[float, Goods] = pq.get()
                    if item[1].elapsed_zhen < 1000: # 丢弃超时的
                        pq_list.append(item)
                    else:
                        friend_berth.num_losed_gds += 1
                # 加入pq_list_list用于恢复
                pq_list_list.append(pq_list)

                # 寻找friend berth无法拿到的货物
                # elapsed_time = 0 # 受到friend berth当前小车取货状态的影响，一般来说大于0
                elapsed_time = int(1/friend_berth.cal_increase_rate()) # 受到friend berth当前小车取货状态的影响，一般来说大于0
                for j, item in enumerate(pq_list):
                    tmp_gds = item[1]
                    # 哪些货物能够被friend berth取到
                    go_time = elapsed_time + tmp_gds.cost + 1
                    go_back_time =  elapsed_time + 2 * tmp_gds.cost + 2
                    if (go_time < tmp_gds.left_zhen # 物品消失前能取到
                        ): #and go_back_time < self.env.left_zhen - friend_berth.transport_time - 5): # 能赶上最后一趟
                        elapsed_time = go_back_time # 取货来回需要两倍，额外考虑避让的时间
                    # 对于无法被取到的物品，如果没被取过，并且自己能够取到
                    else: 
                        cost = robot_cost_grid[tmp_gds.pos.x][tmp_gds.pos.y]
                        if (tmp_gds.fetched == False 
                            and (cost + 1 < tmp_gds.left_zhen)
                            and cost < (249 - (1/(8 * friend_berth.cal_increase_rate())))):
                            losing_gds = (-tmp_gds.price/(2 * cost), tmp_gds, i, j)
                            best_losing_gds_queue.put(losing_gds) # 需要保存对应的friend_berth和item的索引
            
            # 如果可以拿到一个别人拿不到的货物
            if not best_losing_gds_queue.empty():
                (_, fetched_goods, i_friend_berth, j_item) = best_losing_gds_queue.get()
                pq_list_list[i_friend_berth][j_item][1].fetched = True
                success = True
            
            # 恢复friend_berth的队列
            for i, friend_berth in enumerate(self.friend_berths):
                pq_list = pq_list_list[i]
                for item in pq_list:
                    friend_berth.gds_priority_queue.put(item)
            
        return success, fetched_goods
    
    def cal_increase_rate(self): # 最开始几帧
        return (self.total_num_gds+1) / self.env.global_zhen

    def cal_earn_berfore_end_when_n_robots(self, pq: List[Tuple[float, Goods]], n:int):
        earn = 0
        limit_time = 15000 - self.env.global_zhen
        ordered_n_elapsed_time = [0 for _ in range(n)]
        for item in pq:
            ordered_n_elapsed_time.sort()
            goods = item[1]
            if (goods.left_zhen > (ordered_n_elapsed_time[0] + goods.cost + 1)
                and goods.fetched == False): # 表示货物在经过elapsed_time后，还能被取到
                ordered_n_elapsed_time[0] += 2*goods.cost + 1   # 表示取货需要花费2*goods.cost + 5长的时间
                if ordered_n_elapsed_time[0] > limit_time:   # 如果超时
                    break
                earn += goods.price
        self.earn_when_n_comsuming_time[n-1] = ordered_n_elapsed_time[0]
        alpha = 0.7
        earn = self.earn_when_n[n-1] * alpha + earn * (1-alpha)
        return int(earn)

    # def fetch_goods(self) -> Tuple[bool, Goods] : 
    #     success = False
    #     goods: Goods = Goods(-1,[0],Point(-1,-1), -1) # 无效的，只是为了注释正确
    #     if not self.gds_priority_queue.empty():
    #         goods= self.gds_priority_queue.get(False)[1]
    #         if self.berth_id==1:
    #             scheduler_logger.info("zhen:%s fetched goods: %s",self.env.global_zhen, goods)
            
    #         while ( (goods.fetched == True or (1000 - (goods.elapsed_zhen) < self.robot_cost_grid[goods.x][goods.y] + 5))
    #                and not self.gds_priority_queue.empty()):
    #             goods = self.gds_priority_queue.get(False)[1]
    #             if self.berth_id==1:
    #                 scheduler_logger.info("zhen:%s fetched goods: %s",self.env.global_zhen, goods)
    #         # 如果能取到还未被取的
    #         if goods.fetched == False:
    #             success = True
    #     # if self.berth_id==1:
    #     #     scheduler_logger.info("zhen:%s fetched goods: %s",self.env.global_zhen, goods)
    #     #     for item in self.gds_priority_queue:
    #     #         scheduler_logger.info("priQ-item:%s",item)
    #     # logger.info("fetched goods: %s", goods)
    #     return success, goods

