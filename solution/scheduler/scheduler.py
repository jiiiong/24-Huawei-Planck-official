from typing import List, Tuple, Set, Dict
from queue import LifoQueue, Queue, PriorityQueue
import random

from log import scheduler_logger, main_logger, robot_logger
from core import Env
from core import Robot, Robot_Extended_Status
from core import Berth, Goods, Boat
from path_planing import Point, UNREACHABLE_POS, INFINIT_COST

class Scheduler:
    def __init__(self, env: Env) -> None:
        self.env = env   

    def init_robots(self):
        berths = self.env.berths
        robots = self.env.robots
                
        # 分配robot到港口
        for i, robot in enumerate(robots):
            robot.robot_id = i
            robot.robot_id = 0
            robot.env = self.env
            robot.suppose_pos = robot.pos

        # main_logger.info("robots initialization starts")

        # berths = self.env.berths
        # robots = self.env.robots
        # cost_matrix_list = self.env.cost_matrix_list
        # # 初始化robot

        # for i, robot in enumerate(robots):
        #     robot.robot_id = i
        #     robot.env = self.env
        #     robot.suppose_pos = robot.pos
        #     robot.last_pos = robot.pos

        # # 计算robot最近的港口
        # distance: List[Tuple[int, int, int]] = []
        # for robot in robots:
        #     for berth in berths:
        #         distance.append((cost_matrix_list[berth.berth_id][robot.y][robot.x], robot.robot_id, berth.berth_id))
        # distance.sort()

        # allocated_berth = set()
        # robot_berth_init_map = {}
        # for item in distance:
        #     (_, robot_id, berth_id) = item
        #     if robot_id not in robot_berth_init_map and berth_id not in allocated_berth: 
        #         robot_berth_init_map[robot_id] = berth_id
        #         allocated_berth.add(berth_id)

        # # 分配robot到港口
        # for i, robot in enumerate(robots):
        #     robot.berth_id = robot_berth_init_map[robot.robot_id]
        #     # robot.berth_id = 0
        # # scheduler logger 记录机器人与港口的分配情况 
        # scheduler_logger.info("robots information:")
        # for i, robot in enumerate(robots):
        #     scheduler_logger.info(f"robots_id: {robot.robot_id}, berth_id: {robot.berth_id}")
        # scheduler_logger.info(" ")

        # main_logger.info("robots initialization ends\n")

    def init_berths(self):
        pass
        #     main_logger.info("berths initialization starts")

        #     # 初始化所有港口的id，并配置env方便使用
        #     for berth_id, berth in enumerate(self.env.berths):
        #         berth.berth_id = berth_id
        #         berth.env = self.env
            
        #     # scheduler logger记录每个港口的基本信息
        #     scheduler_logger.info("berth information: ")
        #     for berth_id, berth in enumerate(self.env.berths):
        #         scheduler_logger.info(f"berth_id: {berth_id} at {berth.pos}, loading speed: {berth.loading_speed}")
        #     scheduler_logger.info("")
                
        #     for berth_id, berth in enumerate(self.env.berths): 
        #         # 计算可支援的港口
        #         distance_ordered_tuple: List[Tuple[Berth, int]] = []
        #         robot_cost_grid = berth.robot_cost_grid
        #         for friend_berth in self.env.berths:
        #             cost = robot_cost_grid[friend_berth.x][friend_berth.y]
        #             if (friend_berth.berth_id != berth_id) and (cost != INFINIT_COST):
        #                 distance_ordered_tuple.append((friend_berth, cost))
        #         distance_ordered_tuple.sort(key=lambda tup: tup[1])
        #         distance_ordered_friends_berths = [friend_berth for friend_berth, cost in distance_ordered_tuple]

        #         num_friend_berths = min(9, len(distance_ordered_friends_berths))
        #         for order in range(num_friend_berths):
        #             #scheduler_logger.info("friend_berth for %s is %s", berth_id, distance_ordered_friends_berths[order].berth_id)
        #             berth.friend_berths.append(distance_ordered_friends_berths[order])
        #     main_logger.info("berths initialization ends\n")

    def init_boats(self):
        pass
        # main_logger.info("boats initialization starts")

        # boats = self.env.boats
        # berths = self.env.berths

        # # 初始化boat
        # for i, boat in enumerate(boats):
        #     boat.env = self.env
        #     boat.boat_id = i
        #     boat.capacity = self.env.boat_capacity
        #     boat.total_capacity = self.env.boat_capacity
        #     boat.pos = -1
        #     boat.last_run = False

        # # 初始化最近的港口id列表
        # self.tran_time_ordered_list = [(berth.berth_id, berth.transport_time) for berth in berths]
        # self.tran_time_ordered_list.sort(key=lambda x:x[1])
        # self.tran_time_ordered_berths_id_list = [tup[0] for tup in self.tran_time_ordered_list]

        # # 排序后，前端后端两两组合港口
        # berths_cost_time = [(berth_id, berth.transport_time + int(self.env.boat_capacity/berth.loading_speed)+1 )
        #                     for berth_id, berth in enumerate(berths)]
        # berths_cost_time.sort(key=lambda x:x[1], reverse=True)
        
        # # 初始化每条船周期调度所需的参数
        # for boat_id, boat in enumerate(boats):
        #     berth_0 =  berths[berths_cost_time[boat_id][0]]
        #     berth_1 =  berths[berths_cost_time[len(berths_cost_time)-boat_id-1][0]]
        #     boat.associated_berths_list.append(berth_0)
        #     boat.phase_limited_time_list.append(berth_0.transport_time + int(boat.total_capacity/berth_0.loading_speed) + 1)
        #     boat.associated_berths_list.append(berth_1)
        #     boat.phase_limited_time_list.append(boat.phase_limited_time_list[0] + 500 + int(boat.total_capacity/berth_1.loading_speed) + 1)
        #     boat.phase_start_time = 0

        #     ####################>?????????????????????????????????????????????????????????????????????????考虑小车超时造成的影响
        #     # 每一轮所需的时间
        #     boat.cost_per_round = (berth_0.transport_time + int(boat.total_capacity/berth_0.loading_speed) + 1
        #                          + 500
        #                          + berth_1.transport_time + int(boat.total_capacity/berth_1.loading_speed) + 1)
        #     boat.num_available_rounds = int(15000 / boat.cost_per_round)

        # scheduler_logger.info("boat infomations:")
        # scheduler_logger.info(f"boat capacity: {self.env.boat_capacity}")
        # for boat_id, boat in enumerate(boats):
        #     scheduler_logger.info(f"boat_id: {boat.boat_id}, available_rounds: {boat.num_available_rounds}, \
        #                           start_delay: {15000 - boat.cost_per_round * boat.num_available_rounds}")
        # for boat_id, boat in enumerate(boats):
        #     scheduler_logger.info("%s : [%s, %s],", boat_id, boat.associated_berths_list[0].berth_id, boat.associated_berths_list[1].berth_id)
        # scheduler_logger.info("")

        # main_logger.info("boats initialization ends\n")
    
    def run(self):

        self.robots_zhen_handler()
        self.boats_zhen_handler()
        self.berths_zhen_handler()
        self.scheduler_buy()
        
    def berths_zhen_handler(self):
        # 装货
        pass

    def robots_zhen_handler(self):
        robots = self.env.robots

        robot_logger.info("")
        robot_logger.info(f"zhen: {self.env.global_zhen}")
        
        # 下一帧更新上一帧结束后robots状态的转换
        robot_logger.debug(f"update extended status")
        for robot in robots:
            # robot所有的操作基于robot.pos的位置是正确的
            robot.update_extended_status()

        # 调度器调度robots
        robot_logger.debug(f"schdule robots")
        self.schedule_robots()

        # 避障
        try:
            ordered_collision_robots_id = [ (robot.robot_id, robot.get_priority_for_A(robot)) for robot in robots]
            ordered_collision_robots_id.sort(key=lambda x:x[1])
            for i, _ in ordered_collision_robots_id:
                robot_logger.debug(f"collision_avoid by id: {robots[i].robot_id} at {robots[i].pos}, status: {robots[i].extended_status}, master_id: {robots[i].master_robot_id}, next pos is {robots[i].next_n_pos(3)}")
                robots[i].collision_avoid()
        except Exception as e:
            robot_logger.exception(f"collision_avoidance fails") 
            for robot in robots:
                robot_logger.error(f"robot_id: {robot.robot_id} at {robot.pos}, status: {robot.extended_status}, master_id: {robot.master_robot_id} next3pos:{robot.next_n_pos(3)}")
        
        # 获取命令
        robot_cmd_tran = ""
        for robot in robots:
            robot_cmd_tran += robot.paths_execution()
        print(robot_cmd_tran)

    def boats_zhen_handler(self):
        self.schedule_boats()

    def scheduler_buy(self):

        if self.env.robot_num <= 1:
            scheduler_logger.info(f"lbot {self.env.robot_purchase_point[0]}!\n")
            print("lbot", self.env.robot_purchase_point[0].x, self.env.robot_purchase_point[0].y)

        if self.env.boat_num <= 1:
            print("lboat", self.env.boat_purchase_point[0].x, self.env.boat_purchase_point[0].y)

    def schedule_gds(self, goods: Goods):
        pass        
        # # delegated_berth_id = self.env.divide_matrix[goods.y][goods.x]
        # # if (delegated_berth_id != -1):
        # #     self.env.berths[delegated_berth_id].add_goods(goods)

        # cost_matrix_list = self.env.cost_matrix_list
        # bid_cost_list: List[Tuple[int, float]] = []
        # for berth_id in range(self.env.berth_num):
        #     cost = cost_matrix_list[berth_id][goods.y][goods.x]
        #     bid_cost_list.append((berth_id, cost))
        # bid_cost_list.sort(key=lambda x: x[1])
        # # 将货物放入当前最近的3个港口中
        # for order in range(1):
        #     berth_id = bid_cost_list[order][0]
        #     self.env.berths[berth_id].add_goods(goods)

    def schedule_boats(self):
        for i in range(self.env.boat_num):
            main_logger.error(f"{self.env.boats[0].x} {self.env.boats[0].y} {self.env.boats[0].dir} {self.env.boats[0].status}")
            if i == 0 and self.env.boats[0].status == 0:
                if self.env.boat0_actions:
                    action = self.env.boat0_actions.pop(0)
                    # logger.info(action)
                    if action == "ship":
                        print("ship", 0)
                    elif action == "rot 0":
                        print("rot", 0, 0)
                    elif action == "rot 1":
                        print("rot", 0, 1)
                else:
                    print("berth 0")
            elif i == 0 and self.env.boats[0].status == 2:
                print("ship", 0)
            # else:
            #     status = random.randint(0, 1)
            #     if status == 0:
            #         print("ship", i)
            #     else:
            #         print("rot", i, random.randint(0, 1))

        # boats = self.env.boats
        # berths = self.env.berths
        # boat_capacity = self.env.boat_capacity
        # # 遍历调度每一艘轮船
        # for cur_boat in boats:
        #     berth_0 = cur_boat.associated_berths_list[0]
        #     berth_1 = cur_boat.associated_berths_list[1]
            
        #     if (self.env.global_zhen < 15000 - cur_boat.cost_per_round * cur_boat.num_available_rounds - 10):
        #         continue

        #     # 如果已经到达虚拟点
        #     if (cur_boat.num_available_rounds == 1 and cur_boat.last_run is False):
        #         cur_boat.last_run = True
        #         scheduler_logger.info("zhen: %s, boat_id: %s last_run_start ",
        #                 self.env.global_zhen, cur_boat.boat_id)

        #     # phase0 开始
        #     if cur_boat.status == 1 and cur_boat.pos == -1: 
        #         # 清空容量
        #         cur_boat.capacity = cur_boat.total_capacity
        #         # 将轮船调度到第一个目标点，开始phase0
        #         print("ship", cur_boat.boat_id, berth_0.berth_id)
        #             # 设置phase0 开始时间
        #         cur_boat.phase_start_time = self.env.global_zhen
        #         scheduler_logger.info("zhen: %s, boat_id: %s, ship from %s to %s ",
        #                 self.env.global_zhen, cur_boat.boat_id, -1, berth_0.berth_id)
            
        #     # 已经到达第一个目标港口
        #     elif cur_boat.status == 1 and cur_boat.pos == berth_0.berth_id: 
                
        #         # 装货部分
        #         # 计算港口这 一帧 能装载的货物数量
        #         num_loaded_gds = min(berth_0.cur_num_gds, berth_0.loading_speed)
        #         # 但考虑床的容量，不一定能装那么多
        #         num_loaded_gds = min(num_loaded_gds, cur_boat.capacity)
        #         ##########################？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？装货在每一帧的最后结算
        #         #》》》》》》》》》》？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？所以实际上下一帧才可以 减少，暂时不管
        #         # 计算港口剩余货量
        #         berth_0.cur_num_gds -= num_loaded_gds
        #         # 计算boat的剩余容量
        #         cur_boat.capacity -= num_loaded_gds
                
        #         # 调度部分
        #         phase0_elapsed_time = self.env.global_zhen - cur_boat.phase_start_time + 1
        #         phase0_limited_time = cur_boat.phase_limited_time_list[0]
        #         # 如果剩余时间，只够回去
        #         if cur_boat.last_run == True:
        #             if self.env.left_zhen <= 500 + berth_1.transport_time + int(cur_boat.capacity/berth_1.loading_speed) + 1:
        #                 print("ship", cur_boat.boat_id, berth_1.berth_id)
        #                 scheduler_logger.info("zhen: %s, boat_id: %s, ship from %s to %s ",
        #                     self.env.global_zhen, cur_boat.boat_id, -1, berth_1.berth_id)
        #             pass
        #             # 调度机器人
        #             for robot in self.env.robots:
        #                 if robot.berth_id == berth_0.berth_id:
        #                     robot.change_berth(berth_1.berth_id)
                
        #         elif self.env.left_zhen <= berth_0.transport_time + 1 and self.env.left_zhen <= 500+berth_1.transport_time+1:
        #             print("go", cur_boat.boat_id)
        #         # phase0花费时间等于或者超时
        #         elif phase0_elapsed_time >= phase0_limited_time:
        #             print("ship", cur_boat.boat_id, berth_1.berth_id)
        #             scheduler_logger.info("zhen: %s, boat_id: %s, ship from %s to %s ",
        #                 self.env.global_zhen, cur_boat.boat_id, -1, berth_1.berth_id)
        #             # phase2 开始
                
        #     # 到达第二个目标港口
        #     elif cur_boat.status == 1 and cur_boat.pos == berth_1.berth_id: 
        #         berth_1 = cur_boat.associated_berths_list[1]

        #         num_loaded_gds = min(berth_1.cur_num_gds, berth_1.loading_speed)
        #         num_loaded_gds = min(num_loaded_gds, cur_boat.capacity)
        #         berth_1.cur_num_gds -= num_loaded_gds
        #         cur_boat.capacity -= num_loaded_gds
                
        #         # 调度部分***************
        #         # 如果满货了,或者 港口无货了
        #         phase1_elapsed_time = self.env.global_zhen - cur_boat.phase_start_time
        #         phase1_limited_time = cur_boat.phase_limited_time_list[1]
        #         if cur_boat.last_run == True:
        #             if self.env.left_zhen <= berth_1.transport_time + 5:
        #                 print("go ", cur_boat.boat_id)
        #                 scheduler_logger.info("zhen: %s, boat_id: %s, go ",
        #                                     self.env.global_zhen, cur_boat.boat_id)
        #         elif self.env.left_zhen <= berth_1.transport_time:
        #             print("go ", cur_boat.boat_id)
        #             scheduler_logger.info("zhen: %s, boat_id: %s, go ",
        #                                self.env.global_zhen, cur_boat.boat_id)
        #         # phase1花费时间等于或者超时
        #         elif phase1_elapsed_time >= phase1_limited_time:
        #             print("go ", cur_boat.boat_id)
        #             scheduler_logger.info("zhen: %s, boat_id: %s, go ",
        #                                self.env.global_zhen, cur_boat.boat_id)
        #             cur_boat.num_available_rounds -= 1

    def schedule_robots(self):
        # if self.env.global_zhen == 200:
        #     for robot in self.env.robots:
        #         if robot.robot_id == 0:
        #             robot.convert_extended_status(Robot_Extended_Status.BackBerthAndPull)
        #             robot.paths_stk.put(robot.pos + Point(0, -1))

        # elif self.env.global_zhen >= 201:
        #     for robot in self.env.robots:
        #         if robot.robot_id == 1:
        #             robot.convert_extended_status(Robot_Extended_Status.BackBerthAndPull)
        #             robot.paths_stk.put(robot.pos + Point(0, -1))

        # else:
            for robot in self.env.robots:
                robot.back_new_berth(0)
        # robots = self.env.robots
        # for robot in robots:
        #     if robot.extended_status == Robot_Extended_Status.Uninitialized:
        #         # init_robots中只分配了港口，并未检测是否可达
        #         if (self.env.move_matrix_list[robot.berth_id][robot.y][robot.x] != UNREACHABLE_POS):
        #             robot.convert_extended_status(Robot_Extended_Status.BackBerthAndPull)
        #         else:
        #             robot.convert_extended_status(Robot_Extended_Status.UnableBackBerth)
        #     elif (robot.extended_status == Robot_Extended_Status.GotGoods):
        #         # gotgoods状态必须由GotoFetchfromBerth转入，即港口可达
        #         robot.convert_extended_status(Robot_Extended_Status.BackBerthAndPull)
        #     elif robot.extended_status == Robot_Extended_Status.OnBerth:
        #         berths = self.env.berths
        #         cur_berth  = berths[robot.berth_id]
        #         success, goods = cur_berth.fetch_goods()

        #         # 避免分配当前港口拿不到的物品
        #         if success:
        #             if self.env.move_matrix_list[robot.berth_id][goods.y][goods.x] != UNREACHABLE_POS:
        #                 scheduler_logger.info("id: %s, target_gds:%s", robot.robot_id, goods)
        #                 robot.go_to_fetch_gds_from_berth(goods)
        #                 goods.fetched = True