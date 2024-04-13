from typing import List, Tuple, Set, Dict
from queue import LifoQueue, Queue, PriorityQueue
import random

from log import scheduler_logger, main_logger, robot_logger,berth_logger, boat_logger
from core import Env
from core import Robot, Robot_Extended_Status
from core import Berth, Goods, Boat
from path_planing import Point, UNREACHABLE_POS, INFINIT_COST,save_grid_to_file,apply_move_grid_to_ch_grid,sVec
from path_planing import Boat_Direction

class Scheduler:
    def __init__(self, env: Env) -> None:
        self.env = env
        self.BUY_FIRST = True
        self.init_berths()
        self.required_boats_num_per_berth: List[int] = [0 for _ in range(self.env.berth_num)]
        self.cur_boats_num_per_berth: List[int] = [0 for _ in range(self.env.berth_num)]


    def init_robots(self):
        berths = self.env.berths
        robots = self.env.robots
                
        # 分配robot到港口
        for i, robot in enumerate(robots):
            robot.robot_id = i
            robot.berth_id = i
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
            
        # scheduler logger记录每个港口的基本信息
        scheduler_logger.info("berth information: ")
        for berth_id, berth in enumerate(self.env.berths):
            scheduler_logger.info(f"berth_id: {berth_id} at {berth.pos}, loading speed: {berth.loading_speed}")
        scheduler_logger.info("")
                
        for berth_id, berth in enumerate(self.env.berths): 
            # 计算可支援的港口
            distance_ordered_tuple: List[Tuple[Berth, int]] = []
            robot_cost_grid = berth.robot_cost_grid
            for friend_berth in self.env.berths:
                cost = robot_cost_grid[friend_berth.x][friend_berth.y]
                if (friend_berth.berth_id != berth_id) and (cost != INFINIT_COST):
                    distance_ordered_tuple.append((friend_berth, cost))
            distance_ordered_tuple.sort(key=lambda tup: tup[1])
            distance_ordered_friends_berths = [friend_berth for friend_berth, cost in distance_ordered_tuple]

            num_friend_berths = min(10, len(distance_ordered_friends_berths))
            for order in range(num_friend_berths):
                #scheduler_logger.info("friend_berth for %s is %s", berth_id, distance_ordered_friends_berths[order].berth_id)
                berth.friend_berths.append(distance_ordered_friends_berths[order])
        # main_logger.info("berths initialization ends\n")

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
        self.scheduler_buy_base()
        self.scheduler_buy_add()
        # s = str(self.env.global_zhen)+ " " + " ".join([str(berth.num_allocated_gds) + " "+ str(berth.num_losed_gds)+ " "+ str(int(berth.num_losed_gds/(berth.num_allocated_gds+1)*100)) for berth in self.env.berths]) 
        # s = str(self.env.global_zhen)+ " " + " ".join([ " ".join([str(berth.earn_when_n[0]), str(berth.earn_when_n[1]), str(berth.earn_when_n[2])]) for berth in self.env.berths]) 
        
        # boat_logger.info(f"{s}")
        
    def berths_zhen_handler(self):
        if (self.env.global_zhen % 100 == 0):
            for berth in self.env.berths:
                berth.clear_queue()

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

    # def boats_zhen_handler(self):
    #     self.schedule_boats()
    #     for boat in self.env.boats:
    #         boat.boat_execute()

    def boats_zhen_handler(self):
        for boat in self.env.boats:
            boat.collision_recovery()

        self.schedule_boats()

        for boat in self.env.boats:
            boat.boat_execute()

    def scheduler_buy_base(self):
            # 第一艘船必须买到
            left_money = self.env.money
            if self.env.boat_num ==0  and self.env.money>=8000:
                print("lboat", self.env.boat_purchase_sVec_list[0].x, self.env.boat_purchase_sVec_list[0].y)
                left_money -= 8000
                scheduler_logger.info(f"zhen {self.env.global_zhen} boat_id:{self.env.boat_num}  lboat {self.env.boat_purchase_sVec_list[0]}!\n")

            # 剩下的钱全部拿去买机器人，买空
            # robot_number = (25000-self.env.boat_price)//self.env.robot_price
            # scheduler_logger.info("robot_number:%s",robot_number)

            # if self.env.robot_num <=14 and self.env.global_zhen% 10==0 and self.env.money>=2000:
            #     scheduler_logger.info(f"zhen {self.env.global_zhen} robot_id:{self.env.robot_num} lbot {self.env.robot_purchase_point[0]} !\n")
            #     print("lbot", self.env.robot_purchase_point[0].x, self.env.robot_purchase_point[0].y)
        # 给每个港口配备一个机器人
            # per = 2
            
            # robot_number = self.env.berth_num*per
            # scheduler_logger.info("robot_number:%s",robot_number)
            # if self.env.robot_num <=(self.env.berth_num*per-1) and self.env.global_zhen% 10==0 and self.env.money>=2000:
            #     scheduler_logger.info(f"zhen {self.env.global_zhen} robot_id:{self.env.robot_num} lbot {self.env.robot_purchase_point[0]} !\n")
            #     print("lbot", self.env.robot_purchase_point[0].x, self.env.robot_purchase_point[0].y)
            for i, berth in enumerate(self.env.berths):
                # 如果港口丢失率过高
                cur_boat_num = self.cur_boats_num_per_berth[i]
                profit_from_one_more_boat = 0
                if cur_boat_num > 1 and cur_boat_num < len(berth.earn_when_n):
                    profit_from_one_more_boat = berth.earn_when_n[cur_boat_num] - berth.earn_when_n[cur_boat_num-1]
                    profit_from_one_more_boat = profit_from_one_more_boat / (berth.earn_when_n_comsuming_time[cur_boat_num] + 1) * (self.env.left_zhen-100)
                if (cur_boat_num in range(0,2)#):
                    or ( (cur_boat_num in range(2, len(berth.earn_when_n))) and (profit_from_one_more_boat > 2000))
                    ):
                # if (self.cur_boats_num_per_berth[i] < self.required_boats_num_per_berth[i]):
                    # 从最近的购买点 进货
                    dis_ordered_point = [(i, pur_pos.distance(berth.pos)) for i, pur_pos in enumerate(self.env.robot_purchase_point)]
                    dis_ordered_point.sort(key = lambda tup:tup[1])
                    if left_money > 2000:
                        print("lbot", self.env.robot_purchase_point[dis_ordered_point[0][0]].x, self.env.robot_purchase_point[dis_ordered_point[0][0]].y, 0)
                        self.required_boats_num_per_berth[i] += 1
                        indd = ((berth.earn_when_n[self.cur_boats_num_per_berth[i]]+1) - (berth.earn_when_n[self.cur_boats_num_per_berth[i]-1]+1)) / (1+berth.earn_when_n_comsuming_time[self.cur_boats_num_per_berth[i]-1]) * (self.env.left_zhen-100)
                        main_logger.error(f"bug for berth {i}, {indd}, {self.required_boats_num_per_berth[i]}")
                        left_money -= 2000
                # elif self.env.global_zhen == 4000:
                #     if (berth.lost_rate > 0.45):
                #     # 从最近的购买点 进货
                #         dis_ordered_point = [(i, pur_pos.distance(berth.pos)) for i, pur_pos in enumerate(self.env.robot_purchase_point)]
                #         dis_ordered_point.sort(key = lambda tup:tup[1])
                #         print("lbot", self.env.robot_purchase_point[dis_ordered_point[0][0]].x, self.env.robot_purchase_point[dis_ordered_point[0][0]].y)

    def scheduler_buy_add(self):
        # per = 2
        # if self.env.robot_num <=(self.env.berth_num*per-1) and self.env.global_zhen% 10==0 and self.env.money>=2000 and self.env.global_zhen>4000:
        #     scheduler_logger.info(f"zhen {self.env.global_zhen} robot_id:{self.env.robot_num} lbot {self.env.robot_purchase_point[0]} !\n")
        #     print("lbot", self.env.robot_purchase_point[0].x, self.env.robot_purchase_point[0].y)

        if self.env.boat_num==1 and self.env.money>=8000 and (self.env.robot_num >= (self.env.berth_num*2-1) or self.env.global_zhen > 4000):
            print("lboat", self.env.boat_purchase_sVec_list[1].x, self.env.boat_purchase_sVec_list[1].y)
            scheduler_logger.info(f"zhen {self.env.global_zhen} robot_id:{self.env.boat_num}  lboat {self.env.robot_purchase_point[0]}!\n")
             
    def schedule_round_trip(self,boat_id:int,purchase_id:int ,route:List[int],delivery_sVec_id:int):
        # boat_logger.error(f"{route}")
        boat_purchase_sVec = self.env.boat_purchase_sVec_list[purchase_id] 
        berthList:List[sVec] = []
        allberthsVec:List[sVec] = []
        for index in route:
            berthList.append(self.env.berths[index].sVec)
        for item in self.env.berths:
            allberthsVec.append(item.sVec)
        delivery_sVec = self.env.delivery_sVec_list[delivery_sVec_id]
        
        boat = self.env.boats[boat_id]
        # 如果船只在购买点并且路径没有初始化
        if boat.sVec == boat_purchase_sVec and len(boat.actions)==0:
            boat.ship_from_A_to_B(boat_purchase_sVec, berthList[0])
            
        # 如果船只在港口路线上
        elif boat.sVec in allberthsVec and len(boat.actions)==0:
            # 最后一趟
            if self.env.left_zhen <= 200:
                boat.ship_from_A_to_B(boat.sVec,delivery_sVec)

            index = allberthsVec.index(boat.sVec)
            berth_logger.info("boatid %s berth %s",boat_id,index)
            berth_logger.info("boatid %s boat sVec %s",boat_id,boat.sVec)
            # 当前船只的容量满了
            if boat.goods_num == self.env.boat_capacity:
                # dis_ordered_berth = [(d_p, boat.sVec.pos.distance(self.env.delivery_point[i])) for i, d_p in enumerate(self.env.delivery_sVec_list)]
                # dis_ordered_berth.sort(key = lambda tup:tup[1])
                # tmp = dis_ordered_berth[0][0]
                boat.ship_from_A_to_B(boat.sVec,delivery_sVec)
                berth_logger.info("go")
            else:
                # 如果当前港口还有货物
                if self.env.berths[index].cur_num_gds>0:
                    # 有货搬货 船有容量就搬货
                    if boat.status != 2:
                        print("berth ",boat_id)
                        berth_logger.info("boat_id %s berth %s",boat_id,boat_id)
                    elif boat.status == 2:
                        self.env.berths[index].cur_num_gds = self.env.berths[index].cur_num_gds - min(self.env.berths[index].loading_speed, self.env.berths[index].cur_num_gds)
                        berth_logger.info("boat_id %s self.env.berths[%s].cur_num_gds%s",boat_id,index,self.env.berths[index].cur_num_gds)
                else:
                # 当前港口货物装完了
                    # 但是目前的船只还有容量，前往下一个港口
                    if boat.goods_num<self.env.boat_capacity:
                        if index in route:
                            next_index = (berthList.index(boat.sVec)+1) % len(berthList)
                        else:
                            next_index = 0
                        boat.ship_from_A_to_B(boat.sVec,berthList[next_index])
                        berth_logger.info("boat_id:%s next_index:%s",boat_id,next_index)
                    else:
                        boat.ship_from_A_to_B(boat.sVec,delivery_sVec)
                        berth_logger.info("go")
        # 如果船只在售卖点
        elif boat.sVec == delivery_sVec and len(boat.actions)==0:
            boat.ship_from_A_to_B(delivery_sVec,berthList[0])
        
    def schedule_boats(self):

        if self.env.boat_num > 0:
            berth_id_list = [i for i in range(self.env.berth_num)]

            # 计算每艘船负责的港口
            count_per_boat_allocated_berth = self.env.berth_num // self.env.boat_num
            boat_i_route_list: List[List[int]] = []
            for i in range(self.env.boat_num - 1):
                boat_i_route_list.append(berth_id_list[i*count_per_boat_allocated_berth:(i+1)*count_per_boat_allocated_berth])
            boat_i_route_list.append(berth_id_list[(self.env.boat_num - 1)*count_per_boat_allocated_berth:])
            
            delivery_id = self.env.delivery_sVec_list.index(self.env.delivery_sVec_list[0])
            purchase_id = 0
            self.schedule_round_trip(0,purchase_id,boat_i_route_list[0],delivery_id)
            if self.env.boat_num == 2:
                delivery_id = self.env.delivery_sVec_list.index(self.env.delivery_sVec_list[1%(len(self.env.delivery_sVec_list))])
                purchase_id = 1
                self.schedule_round_trip(1,purchase_id,boat_i_route_list[1],delivery_id)
        
        # for i in range(self.env.boat_num):
    
        #     berth_logger.info("now %s",self.env.boats[i].sVec)
        #     berth_logger.info("now boatstatus%s",self.env.boats[i].status)
        #     boat_purchase_sVec = self.env.boat_purchase_sVec_list[0]
        #     berth0sVec = self.env.berths[0].sVec
        #     berth1sVec = self.env.berths[1].sVec
        #     berth2sVec = self.env.berths[2].sVec
        #     berth3sVec = self.env.berths[3].sVec
        #     berth3sVec = self.env.berths[4].sVec
        #     berthList = [berth0sVec,berth1sVec,berth2sVec,berth3sVec]
        #     delivery_sVec = self.env.delivery_sVec_list[0]
        #     if self.env.boats[i].sVec == boat_purchase_sVec and len(self.env.boats[i].actions)==0:
        #         self.env.boats[i].ship_from_A_to_B(boat_purchase_sVec, berth0sVec)
        #     if self.env.boats[i].sVec in berthList and len(self.env.boats[i].actions)==0:
        #         index = berthList.index(self.env.boats[i].sVec)
        #         berth_logger.info("now berth %s",index)
        #         berth_logger.info("now boat sVec %s",self.env.boats[i].sVec)
        #         if self.env.berths[index].cur_num_gds>0:
        #             # 有货搬货 船有容量就搬货
        #             if self.env.boats[i].status != 2:
        #                 print("berth ",0)
        #             elif self.env.boats[i].status == 2:
        #                 self.env.berths[index].cur_num_gds = self.env.berths[index].cur_num_gds - self.env.berths[index].loading_speed
        #                 berth_logger.info("now self.env.berths[%s].cur_num_gds%s",index,self.env.berths[index].cur_num_gds)
        #         else:
        #             # 没货走人
        #             if self.env.boats[i].goods_num<self.env.boat_capacity:
        #                 self.env.boats[i].ship_from_A_to_B(self.env.boats[i].sVec,berthList[((index+1)%len(berthList))])
        #                 berth_logger.info("next_index:%s",((index+1)%len(berthList)))
        #             else:
        #                 self.env.boats[i].ship_from_A_to_B(self.env.boats[i].sVec,delivery_sVec)
        #                 berth_logger.info("go")
        #     if self.env.boats[i].sVec == delivery_sVec and len(self.env.boats[i].actions)==0:
        #         self.env.boats[i].ship_from_A_to_B(delivery_sVec,berth0sVec)

                # while(len(self.env.test_route)>0):
                #     start_sVec, end_sVec = self.env.test_route.pop(0)

                #     if self.env.boats[0].sVec ==  start_sVec:
                #         # berth_logger.info("schedule boat %s %s",start_sVec,end_sVec)
                #         self.env.boats[0].ship_from_A_to_B(start_sVec, end_sVec)
                #         break
                #     else:
                #         continue
                # if self.env.boats[0].sVec == (self.env.boat_purchase_sVec_list[0]):
                #     self.env.boats[0].ship_from_A_to_B(self.env.boat_purchase_sVec_list[0], self.env.berths[0].sVec)
                # else:
                #     for id, berth in enumerate(self.env.berths):
                #         if self.env.boats[0].sVec == berth.sVec:
                #             self.env.boats[0].ship_from_A_to_B(self.env.berths[id].sVec, self.env.berths[((id+1)%len(self.env.berths))].sVec)

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
        robots = self.env.robots
        berths = self.env.berths
        # if self.env.global_zhen %100 ==0:
        #     for berth in berths:
        #         berth_logger.info("zhen %s cur_num_gds:%s ",self.env.global_zhen,berth.cur_num_gds)


        if self.env.global_zhen == 999:
            for index ,item in self.env.boat_route_dict.items():
                berth_logger.info("(%s,%s) :%s - %s",index[0].pos,index[1].pos,len(item),item)
            for item in self.env.delivery_point:
                berth_logger.info("%s",item)
            
        if self.env.global_zhen %10:
            for berth in berths:
                a = " ".join([str(boat.goods_num) for boat in self.env.boats])
                berth_logger.info(f"{a}")
                berth_logger.info("zhen %s, id: %s cur_num_gds:%s ",self.env.global_zhen, berth.berth_id, berth.cur_num_gds)
        if self.env.global_zhen ==14999:
            for berth in berths:
                berth_logger.info("zhen %s total_earn:%s ",self.env.global_zhen,berth.total_earn)
            berth_logger.info("total_value:%s ",self.env.goods_total_value)
        # if self.env.global_zhen == 1000:
        #     for item in self.env.test_route:
        #         berth_logger.info("item_start:%s- item_end:%s ",item[0],item[1])
        #     for item in self.env.boat_purchase_sVec_list:
        #         berth_logger.info("boat_purchase_sVec:%s ",item)
        #     for item in self.env.delivery_sVec_list:
        #         berth_logger.info("delivery_sVec_list:%s ",item)

        for robot in robots:
            if robot.extended_status == Robot_Extended_Status.Uninitialized:
                # init_robots中只分配了港口，并未检测是否可达
                # robot.berth_id = robot.robot_id
                dis_ordered_berth = [(i, robot.pos.distance(berth.pos)) for i, berth in enumerate(self.env.berths)]
                dis_ordered_berth.sort(key = lambda tup:tup[1])
                for i, berth in dis_ordered_berth:
                    if self.cur_boats_num_per_berth[i] < self.required_boats_num_per_berth[i]:
                        robot.berth_id = i
                        # berth.num_allocated_gds = 0
                        # berth.num_losed_gds = 0
                        self.cur_boats_num_per_berth[i] += 1
                        break
                
                robot.berth_id = robot.robot_id%self.env.berth_num
                if(berths[robot.berth_id].robot_move_grid[robot.x][robot.y]!= UNREACHABLE_POS):
                    robot.convert_extended_status(Robot_Extended_Status.BackBerthAndPull)
                else:
                    robot.convert_extended_status(Robot_Extended_Status.UnableBackBerth)
            elif (robot.extended_status == Robot_Extended_Status.GotGoods):
                # gotgoods状态必须由GotoFetchfromBerth转入，即港口可达
                robot.convert_extended_status(Robot_Extended_Status.BackBerthAndPull)
            elif robot.extended_status == Robot_Extended_Status.OnBerth:
                cur_berth = berths[robot.berth_id]
                success, goods = cur_berth.fetch_goods()

                # 避免分配当前港口拿不到的物品
                if success:
                    if berths[robot.berth_id].robot_move_grid[goods.x][goods.y] != UNREACHABLE_POS:
                        scheduler_logger.info("id: %s, target_gds:%s", robot.robot_id, goods)
                        robot.go_to_fetch_gds_from_berth(goods)
                        goods.fetched = True