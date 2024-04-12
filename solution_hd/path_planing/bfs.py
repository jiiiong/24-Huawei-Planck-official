from queue import Queue, PriorityQueue
from typing import List, Tuple, Dict, Any

from cfg import N
from log import main_logger

from .base import Pixel_Attrs, Point
from .base import Robot_Move, INFINIT_COST
from .base import Boat_Action, Boat_Direction

# robot相关

robot_move_list = [Robot_Move.UP, Robot_Move.DOWN, Robot_Move.LEFT, Robot_Move.RIGHT, Robot_Move.HOLD]

def robot_bfs(attrs_grid: List[List[Pixel_Attrs]], source_point: Point) -> Tuple[List[List[Point]], List[List[int]]]:
    '''
    输入：
    attrs_grid: 每个元素为pixel_attrs，is_ground表示机器人可以运动，is_ocean表示船可以运动
    输出：
    robot_cost_grid: 地图上任一点到source point需要移动的步数, 若不可达, 则为-1
    robot_move_grid: 表示从该点回source point需要移动的方向, -2,-2表示不可达
    注意：
    source_point必须可以到达
    '''
    frontier: Queue[Point] = Queue()
    came_from: Dict[Point, Point] = dict()
    robot_cost_grid = [ [INFINIT_COST  for _ in range(N)] for _ in range(N)]
    robot_move_grid = [ [Robot_Move.BAD_MOVE for _ in range(N)] for _ in range(N)]


    frontier.put(source_point)
    came_from[source_point] = source_point
    robot_cost_grid[source_point.x][source_point.y] = 0
    robot_move_grid[source_point.x][source_point.y] = Robot_Move.HOLD
    
    global robot_move_list
    # 未考虑初始点为障碍物时的情况
    while frontier.empty() is False:

        cur_pos = frontier.get()
        for move in robot_move_list:
            next_pos = cur_pos + move
            if (next_pos.is_in_grid()
                and attrs_grid[next_pos.x][next_pos.y].is_ground
                and next_pos not in came_from):
                frontier.put(next_pos)
                came_from[next_pos] = cur_pos
                robot_cost_grid[next_pos.x][next_pos.y] = robot_cost_grid[cur_pos.x][cur_pos.y] + 1
                robot_move_grid[next_pos.x][next_pos.y] = -move
    return robot_move_grid, robot_cost_grid

def one_move_avoidance(attrs_grid: List[List[Pixel_Attrs]], source_point: Point) -> Tuple[List[Point], bool]:
    '''
    输入：
    attrs_grid: 每个元素为pixel_attrs，is_ground表示机器人可以运动，is_ocean表示船可以运动
    原点必为障碍物？？？
    输出：
    
    '''
    success = False
    avoidance_paths: List[Point] = []
    
    # 未考虑初始点为障碍物时的情况
    y_len = len(attrs_grid)
    x_len = len(attrs_grid[0])

    global robot_move_list
    for move in robot_move_list:
        next_pos:Point = source_point + move
        if (0<=next_pos.y and next_pos.y<y_len 
            and 0<=next_pos.x and next_pos.x<x_len
            and attrs_grid[next_pos.y][next_pos.x].is_ground): # 下一个位置不会碰撞
            avoidance_paths.append(next_pos)
            success = True
            break

    return (avoidance_paths, success)

# boat相关

def boat_bfs(attrs_grid: List[List[Pixel_Attrs]], boat_direction_grid:List[List[ (List[str]) ]],
             start_sVec: Tuple[Point, str], end_sVec: Tuple[Point, str]) -> List:
    queue = PriorityQueue()
    queue.put((0, 0, start_sVec, []))
    visited = set()
    g_scores = {start_sVec: 0}
    ret = []

    while not queue.empty():
        
        (f_current, g_current, cur_sVec, actions) = queue.get()
        cur_sVec: Tuple[Point, str]
        actions: List[str]
        g_current: int

        # main_logger.error(f"!! {current_position} {current_direction}")
        if cur_sVec == end_sVec:
            return actions
        if cur_sVec in visited:
            continue
        visited.add(cur_sVec)

        # 尝试 ship_one_move
        okk, new_sVec = ship_one_move(cur_sVec, boat_direction_grid)
        # main_logger.error(f"{okk} {new_position}")
        if okk:
            okk = ship_position_available(new_sVec, boat_direction_grid, attrs_grid)
            # main_logger.error(f"{okk} {new_position} {new_position not in g_scores}")
        if okk:
            new_g = g_current + 1  # 假设每次移动成本为1
            
            if new_sVec not in g_scores or new_g < g_scores[new_sVec]:
                g_scores[new_sVec] = new_g
                f = new_g + heuristic(new_sVec[0], end_sVec[0])
                queue.put((f, new_g, new_sVec, actions + ["ship"]))
                # test = actions + ["ship"]
                # main_logger.error(f"okkkk {new_position} {test}")
        # if okk == False:
        #     main_logger.error(f"invalid {new_position} {test}")

        # 尝试 rotate_one_move，顺时针和逆时针旋转
        for rotation in [1, 0]:
            okk, new_sVec = rotate_one_move(cur_sVec, rotation, boat_direction_grid, attrs_grid)
            if okk:
                new_g = g_current + 1  # 假设旋转成本为1
                if new_sVec not in g_scores or new_g < g_scores[new_sVec]:
                    g_scores[new_sVec] = new_g
                    f = new_g + heuristic(new_sVec[0], end_sVec[0])
                    queue.put((f, new_g, new_sVec, actions + [f"rot {rotation}"]))
                    # test = actions + [f"rot {rotation}"]
                    # main_logger.error(f"okkkk {new_pos_after_rot} {test}")
            # else:
            #     main_logger.error(f"invalid {new_pos_after_rot} {test}")

    #main_logger.error(f"++{count}")     
    return ret



# 先复制过来用一下，因为在 env 中 boat_bfs 命名冲突
# 这是a*算法
# 除了返回到达 target 的一系列动作外，我还希望返回 到达 target 时船的方向
def route_a_star(attrs_grid: List[List[Pixel_Attrs]], boat_direction_grid: List[List[(List[str])]],
             start_sVec: Tuple[Point, str], end_sVec: Tuple[Point, str]) -> List:


    # 如果轮船到达靠泊点，且目的地是港口时，尝试靠泊
    def ready_to_berth(cur_point, berth_point):
        if attrs_grid[cur_point.x][cur_point.y].is_mooring and attrs_grid[berth_point.x][berth_point.y].is_berth:
            if cur_point.distance(berth_point) <= 3:
                return True
        return False


    queue = PriorityQueue()
    queue.put((0, 0, start_sVec, []))
    visited = set()
    g_scores = {start_sVec: 0}
    ret = []

    while not queue.empty():

        (f_current, g_current, cur_sVec, actions) = queue.get()
        cur_sVec: Tuple[Point, str]
        actions: List[str]
        g_current: int

        # 先判断目标点是否为港口, 如果是港口且已到达靠泊区，可以直接添加靠泊命令并返回
        if ready_to_berth(cur_sVec[0], end_sVec[0]) is True:
            actions = actions + ["berth"]

            # 这里的方向应该是有问题的
            return actions, cur_sVec[1]



        # main_logger.error(f"!! {current_position} {current_direction}")
        # 如果规定了方向就正常比较就好
        if end_sVec[1] != Boat_Direction.ANYWHERE:
            if cur_sVec == end_sVec:
                return actions, cur_sVec[1] # 返回动作栈和抵达时的方向
        # 如果没有规定方向，则直接比较 当前坐标点就好
        else:
            if cur_sVec[0] == end_sVec[0]:
                return actions, cur_sVec[1]

        if cur_sVec in visited:
            continue
        visited.add(cur_sVec)

        # 尝试 ship_one_move
        okk, new_sVec = ship_one_move(cur_sVec, boat_direction_grid)
        # main_logger.error(f"{okk} {new_position}")
        if okk:
            okk = ship_position_available(new_sVec, boat_direction_grid, attrs_grid)
            # main_logger.error(f"{okk} {new_position} {new_position not in g_scores}")
        if okk:
            # if attrs_grid[cur_sVec[0].x][cur_sVec[0].y].is_main_channel:
            #     new_g = g_current + 2  # 如果在主航道上 ，每次移动的成本为2
            # else:
            #     new_g = g_current + 1  # 如果不在主航道上，每次移动的成本为1
            new_g = g_current + 1
            if new_sVec not in g_scores or new_g < g_scores[new_sVec]:
                g_scores[new_sVec] = new_g
                f = new_g + heuristic(new_sVec[0], end_sVec[0])
                queue.put((f, new_g, new_sVec, actions + ["ship"]))
                # test = actions + ["ship"]
                # main_logger.error(f"okkkk {new_position} {test}")
        # if okk == False:
        #     main_logger.error(f"invalid {new_position} {test}")

        # 尝试 rotate_one_move，顺时针和逆时针旋转
        for rotation in [1, 0]:
            okk, new_sVec = rotate_one_move(cur_sVec, rotation, boat_direction_grid, attrs_grid)
            if okk:
                # if attrs_grid[cur_sVec[0].x][cur_sVec[0].y].is_main_channel:
                #     new_g = g_current + 2  # 如果在主航道上 ，每次移动的成本为2
                # else:
                #     new_g = g_current + 1  # 如果不在主航道上，每次移动的成本为1
                new_g = g_current + 1
                if new_sVec not in g_scores or new_g < g_scores[new_sVec]:
                    g_scores[new_sVec] = new_g
                    f = new_g + heuristic(new_sVec[0], end_sVec[0])
                    queue.put((f, new_g, new_sVec, actions + [f"rot {rotation}"]))
                    # test = actions + [f"rot {rotation}"]
                    # main_logger.error(f"okkkk {new_pos_after_rot} {test}")
            # else:
            #     main_logger.error(f"invalid {new_pos_after_rot} {test}")

    # main_logger.error(f"++{count}")
    return ret, None

def reverse_path():
    pass


# 新的想法是，先以核心点为主，生成一条能从起点到终点的路径，再通过 boat_direction_grid 修改得到具体的动作栈
# def refined_route_a_star(attrs_grid: List[List[Pixel_Attrs]], boat_direction_grid: List[List[(List[str])]],
#              start_sVec: Tuple[Point, str], end_sVec: Tuple[Point, str]) -> List:
#     queue = PriorityQueue()
#     queue.put((0, 0, start_sVec[0], []))
#     g_scores = {start_sVec[0]: 0}
#
#     while not queue.empty():
#         (f_current, g_current, cur_point, point_list) = queue.get()
#         cur_point: Point
#         point_list: List[Point]
#         g_current: int
#
#         # 如果当前 queue 中取出的点为最终的点, 则退出循环，进入路径查找状态
#         if cur_point == end_sVec[0]:
#             break
#
#         # 遍历上下左右




# 判断当前船的位置是否合法
def ship_position_available(cur_sVec: Tuple[Point, str], 
                            boat_direction_grid:List[List[(List[str])]], attrs_grid: List[List[Pixel_Attrs]]):
    current_position = cur_sVec[0]
    current_direction = cur_sVec[1]

    if current_position.x < 0 or current_position.x >= N or current_position.y < 0 or current_position.y >= N:
        return False
    if attrs_grid[current_position.x][current_position.y].is_ocean == False:
        return False
    if current_direction not in boat_direction_grid[current_position.x][current_position.y]:
        return False
    return True

def rotate_one_move(cur_sVec: Tuple[Point, str], rotation,
                    boat_direction_grid:List[List[(List[str])]], attrs_grid: List[List[Pixel_Attrs]]):
    x = cur_sVec[0].x
    y = cur_sVec[0].y
    current_direction = cur_sVec[1]
    # 方向转换逻辑
    def direction_transfer(current_direction, rotation):
        direction_order = [Boat_Direction.RIGHT, Boat_Direction.DOWN, Boat_Direction.LEFT, Boat_Direction.UP]
        index = direction_order.index(current_direction)
        if rotation == 0:  # 顺时针旋转
            new_ship_direction = direction_order[(index + 1) % 4]
        else:  # 逆时针旋转
            new_ship_direction = direction_order[(index - 1) % 4]
        return new_ship_direction

    # 更新方向
    new_direction = direction_transfer(current_direction, rotation)

    # 根据当前方向和旋转方向，确定核心点的新位置
    # 注意：这里的逻辑可能需要根据船只的具体布局和旋转前后的位置调整
    new_x = 0
    new_y = 0
    if rotation == 0:  # 顺时针旋转
        if current_direction == Boat_Direction.RIGHT:
            new_x, new_y = x, y + 2
        elif current_direction == Boat_Direction.DOWN:
            new_x, new_y = x + 2, y
        elif current_direction == Boat_Direction.LEFT:
            new_x, new_y = x, y - 2
        elif current_direction == Boat_Direction.UP:
            new_x, new_y = x - 2, y
    else:  # 逆时针旋转
        if current_direction == Boat_Direction.RIGHT:
            new_x, new_y = x + 1, y + 1
        elif current_direction == Boat_Direction.DOWN:
            new_x, new_y = x + 1, y - 1
        elif current_direction == Boat_Direction.LEFT:
            new_x, new_y = x - 1, y - 1
        elif current_direction == Boat_Direction.UP:
            new_x, new_y = x - 1, y + 1

    new_sVec = (Point(new_x, new_y), new_direction)
    okk = ship_position_available(new_sVec, boat_direction_grid, attrs_grid)

    return okk, new_sVec

# 用于前进 ship, 返回一个 bool 类型表示成功或失败，返回一个新的坐标，表示移动后核心点的位置
def ship_one_move(cur_Svec: Tuple[Point, str], boat_direction_grid:List[List[ (List[str]) ]]) -> Tuple[bool, Tuple[Point, str]]:
    current_pos = cur_Svec[0]
    current_direction = cur_Svec[1]
    okk = False
    # 向上移动一步
    if (current_direction == Boat_Direction.UP and current_pos.x - 1 >= 0 
        and current_direction in boat_direction_grid[current_pos.x - 1][current_pos.y]):
        new_pos = Point(current_pos.x - 1, current_pos.y)
        okk = True
        return okk, (new_pos, current_direction)
    # 向下移动一步
    elif (current_direction == Boat_Direction.DOWN and current_pos.x + 1 < N 
          and current_direction in boat_direction_grid[current_pos.x + 1][current_pos.y]):
        new_pos = Point(current_pos.x + 1, current_pos.y)
        okk = True
        return okk, (new_pos, current_direction)
    # 向左移动一步
    elif (current_direction == Boat_Direction.LEFT and current_pos.y - 1 >= 0 
          and current_direction in boat_direction_grid[current_pos.x][current_pos.y - 1]):
        new_pos = Point(current_pos.x, current_pos.y - 1)
        okk = True
        return okk, (new_pos, current_direction)
    # 向右移动一步
    elif (current_direction == Boat_Direction.RIGHT and current_pos.y + 1 < N 
          and current_direction in boat_direction_grid[current_pos.x][current_pos.y + 1]):
        new_pos = Point(current_pos.x, current_pos.y + 1)
        okk = True
        return okk, (new_pos, current_direction)
    else:
        return False, (Point(), "")
    
def heuristic(a: Point, b: Point):
    # 使用曼哈顿距离作为启发式函数
    return abs(a.x - b.x) + abs(a.y - b.y)