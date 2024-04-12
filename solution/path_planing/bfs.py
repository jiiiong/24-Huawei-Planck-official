from queue import Queue, PriorityQueue
from typing import List, Tuple, Dict, Any, Set
import time

from cfg import N
from log import main_logger, func_timer

from .base import Pixel_Attrs, Point, sVec
from .base import Robot_Move, INFINIT_COST
from .base import Boat_Action, Boat_Direction, Boat_Ship_Offset

# robot相关

robot_move_list = [Robot_Move.UP, Robot_Move.DOWN, Robot_Move.LEFT, Robot_Move.RIGHT, Robot_Move.HOLD]

def robot_bfs(task_id: int, attrs_grid: List[List[Pixel_Attrs]], source_point: Point) -> Tuple[int, List[List[Point]], List[List[int]]]:
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
    return task_id, robot_move_grid, robot_cost_grid

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
    x_len = len(attrs_grid)
    y_len = len(attrs_grid[0])

    global robot_move_list
    for move in robot_move_list:
        next_pos:Point = source_point + move
        if (0<=next_pos.y and next_pos.y<y_len 
            and 0<=next_pos.x and next_pos.x<x_len
            and attrs_grid[next_pos.x][next_pos.y].is_ground): # 下一个位置不会碰撞
            avoidance_paths.append(next_pos)
            success = True
            break

    return (avoidance_paths, success)

# boat相关
@func_timer
def boat_bfs(task_id: Tuple[sVec,sVec],
             attrs_grid: List[List[Pixel_Attrs]], boat_valid_grid:List[List[ (List[Boat_Direction]) ]],
             start_sVec: sVec, end_sVec: sVec, lock_grid) -> Tuple[Tuple[sVec,sVec], List[str]]:
    queue: PriorityQueue[Tuple[float, int, int, sVec, List[str]]] = PriorityQueue()
    queue.put((0, 0, 0,start_sVec, []))
    visited = set()
    g_scores = {start_sVec: 0}
    ret = []
    count = 0
    while not queue.empty():
        
        count += 1
        (f_current, h_current, g_current, cur_sVec, actions) = queue.get()
        cur_sVec: sVec
        actions: List[str]
        g_current: int

        # main_logger.error(f"!! {cur_sVec} \t\tf:{f_current} h:{h_current} g:{g_current}")
        if cur_sVec == end_sVec:
            main_logger.error(f"--{start_sVec}, {end_sVec}, {count}")
            return task_id, actions
        if cur_sVec in visited:
            continue
        visited.add(cur_sVec)
        
        # p_c = 1

        # 尝试 ship_one_move
        okk, new_sVec = ship_one_move(cur_sVec, boat_valid_grid, attrs_grid)
        # new_sVec = sVec(cur_sVec.pos+Boat_Ship_Offset[cur_sVec.dir], cur_sVec.dir)
        # okk = ship_position_available(new_sVec, boat_valid_grid)
        if okk:
            new_g = g_current + boat_gen_cost(attrs_grid, new_sVec, lock_grid)  # 假设每次移动成本为1
            # new_p = cur_penalty + (boat_gen_cost(attrs_grid, new_sVec, lock_grid) - 1) * p_c
            if new_sVec not in g_scores or new_g < g_scores[new_sVec]:
                g_scores[new_sVec] = new_g
                new_h = heuristic(new_sVec.pos, end_sVec.pos)#  + new_p
                f = new_g + new_h
                # f = 0
                queue.put((f, new_h, new_g, new_sVec, actions + ["ship"]))
                # queue.put((new_h,f,  new_g, new_p, new_sVec, actions + ["ship"]))

        # 尝试 rotate_one_move，顺时针和逆时针旋转
        for rotation in [1, 0]:
            okk, new_sVec = rotate_one_move(cur_sVec, rotation, boat_valid_grid)
            if okk:
                new_g = g_current + boat_gen_cost(attrs_grid, new_sVec, lock_grid)  # 假设旋转成本为1
                # new_p = cur_penalty + (boat_gen_cost(attrs_grid, new_sVec, lock_grid) - 1) * p_c
                if new_sVec not in g_scores or new_g < g_scores[new_sVec]:
                    g_scores[new_sVec] = new_g
                    new_h = heuristic(new_sVec.pos, end_sVec.pos)#  + new_p 
                    f = new_g + new_h
                    # f = 0
                    # queue.put((new_h,f,  new_g, new_p, new_sVec, actions + ["rot " + str(rotation)]))
                    queue.put((f, new_h, new_g, new_sVec, actions + ["rot " + str(rotation)]))
        
    main_logger.error(f"++{count}")  
    return task_id, ret

# @func_timer
def boat_gen_cost(attrs_grid: List[List[Pixel_Attrs]], cur_sVec: sVec, lock_grid) -> int:
    lt_pos, rb_pos = cur_sVec.proj()
    max_num_shared_roads = 0
    for x in range(lt_pos.x, rb_pos.x+1):
        for y in range(lt_pos.y, rb_pos.y+1):
            if attrs_grid[x][y].is_free_ocean:
                return 2
            #     return 2 + lock_grid[x][y].num_shared_roads
            # elif lock_grid[x][y].num_shared_roads > max_num_shared_roads:
            #     max_num_shared_roads = lock_grid[x][y].num_shared_roads
    return 1 # + max_num_shared_roads

# 判断当前船的位置是否合法
def ship_position_available(cur_sVec: sVec, boat_valid_grid:List[List[(List[Boat_Direction])]]):
    if cur_sVec.pos.is_in_grid() is False:
        return False
    if cur_sVec.dir not in boat_valid_grid[cur_sVec.pos.x][cur_sVec.pos.y]:
        return False
    return True

def rotate_one_move(cur_sVec: sVec, rotation, boat_valid_grid:List[List[(List[Boat_Direction])]]):
    x = cur_sVec.x
    y = cur_sVec.y
    current_direction = cur_sVec.dir
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

    new_sVec = sVec(Point(new_x, new_y), new_direction)
    okk = ship_position_available(new_sVec, boat_valid_grid)

    return okk, new_sVec

# 用于前进 ship, 返回一个 bool 类型表示成功或失败，返回一个新的坐标，表示移动后核心点的位置
def ship_one_move(cur_sVec: sVec, boat_valid_grid:List[List[ (List[Boat_Direction]) ]],
                  attrs_grid: List[List[Pixel_Attrs]]) -> Tuple[bool, sVec]:

    new_sVec = sVec(cur_sVec.pos + Boat_Ship_Offset[cur_sVec.dir], cur_sVec.dir)
    if ship_position_available(new_sVec, boat_valid_grid):
        return True, new_sVec
    else:
        return False, new_sVec
    
def heuristic(a: Point, b: Point):

    dis = abs(a.x - b.x) + abs(a.y - b.y)
    # return dis 
    # 使用曼哈顿距离作为启发式函数
    if dis <= 6:
        return -10
    else:
        return dis 

def boat_actions_to_poses(start_sVec:sVec, actions: List[str]) -> Set[Point]:

    pos_list: Set[Point]= set()
    lt_pos, rb_pos = start_sVec.proj()
    for x in range(lt_pos.x, rb_pos.x+1):
        for y in range(lt_pos.y, rb_pos.y+1):
            pos_list.add(Point(x,y))
    cur_sVec = start_sVec
    for action in actions:
        cur_pos = cur_sVec.pos
        if action == "ship":
            if cur_sVec.dir == Boat_Direction.RIGHT:
                pos_list.add(cur_pos+Point(0, 3))
                pos_list.add(cur_pos+Point(1, 3))
            elif cur_sVec.dir == Boat_Direction.LEFT:
                pos_list.add(cur_pos+Point(0, -3))
                pos_list.add(cur_pos+Point(-1, -3))
            elif cur_sVec.dir == Boat_Direction.UP:
                pos_list.add(cur_pos+Point(-3, 0))
                pos_list.add(cur_pos+Point(-3, 1))
            elif cur_sVec.dir == Boat_Direction.DOWN:
                pos_list.add(cur_pos+Point(3, 0))
                pos_list.add(cur_pos+Point(3, -1))

            cur_sVec = sVec(cur_sVec.pos + Boat_Ship_Offset[cur_sVec.dir], cur_sVec.dir)

        elif action == "rot 0":
            if cur_sVec.dir == Boat_Direction.RIGHT:
                pos_list.add(cur_pos+Point(2, 1))
                pos_list.add(cur_pos+Point(2, 2))
            elif cur_sVec.dir == Boat_Direction.LEFT:
                pos_list.add(cur_pos+Point(-2, -1))
                pos_list.add(cur_pos+Point(-2, -2))
            elif cur_sVec.dir == Boat_Direction.UP:
                pos_list.add(cur_pos+Point(-1, 2))
                pos_list.add(cur_pos+Point(-2, 2))
            elif cur_sVec.dir == Boat_Direction.DOWN:
                pos_list.add(cur_pos+Point(1, -2))
                pos_list.add(cur_pos+Point(2, -2))
            cur_sVec = rotate_one_move_no_test(cur_sVec, 0)

        elif action == "rot 1":
            if cur_sVec.dir == Boat_Direction.RIGHT:
                pos_list.add(cur_pos+Point(-1, 1))
                pos_list.add(cur_pos+Point(-1, 2))
            elif cur_sVec.dir == Boat_Direction.LEFT:
                pos_list.add(cur_pos+Point(1, -1))
                pos_list.add(cur_pos+Point(1, -2))
            elif cur_sVec.dir == Boat_Direction.UP:
                pos_list.add(cur_pos+Point(-1, -1))
                pos_list.add(cur_pos+Point(-2, -1))
            elif cur_sVec.dir == Boat_Direction.DOWN:
                pos_list.add(cur_pos+Point(1, 1))
                pos_list.add(cur_pos+Point(2, 1))
            cur_sVec = rotate_one_move_no_test(cur_sVec, 1)

    return pos_list

def boat_actions_to_poses_per_action(start_sVec:sVec, actions: List[str]) -> List[List[Point]]:
    ret: List[List[Point]] = []

    cur_sVec = start_sVec
    poses: List[Point]= []
    lt_pos, rb_pos = start_sVec.proj()
    for x in range(lt_pos.x, rb_pos.x+1):
        for y in range(lt_pos.y, rb_pos.y+1):
            poses.append(Point(x,y))
    ret.append(poses)
    for action in actions:
        
        if action == "ship":
            cur_sVec = sVec(cur_sVec.pos + Boat_Ship_Offset[cur_sVec.dir], cur_sVec.dir)
        elif action == "rot 0":
            cur_sVec = rotate_one_move_no_test(cur_sVec, 0)
        elif action == "rot 1":
            cur_sVec = rotate_one_move_no_test(cur_sVec, 1)

        poses: List[Point]= []
        lt_pos, rb_pos = cur_sVec.proj()
        for x in range(lt_pos.x, rb_pos.x+1):
            for y in range(lt_pos.y, rb_pos.y+1):
                poses.append(Point(x,y))
        ret.append(poses)

    return ret

def boat_actions_to_sVecs(start_sVec:sVec, actions: List[str]) -> List[sVec]:
    
    sVec_list: List[sVec] = [start_sVec]
    cur_sVec = start_sVec
    for action in actions:
        if action == "ship":
            cur_sVec = sVec(cur_sVec.pos + Boat_Ship_Offset[cur_sVec.dir], cur_sVec.dir)
        elif action == "rot 0":
            cur_sVec = rotate_one_move_no_test(cur_sVec, 0)
        elif action == "rot 1":
            cur_sVec = rotate_one_move_no_test(cur_sVec, 1)
        sVec_list.append(cur_sVec)
    return sVec_list
    # for cur_sVec in sVec_list:
    #     lt_pos, rb_pos = cur_sVec.proj()
    #     for x in range(lt_pos.x, rb_pos.x+1):
    #         for y in range(lt_pos.y, rb_pos.y+1):
    #             if attrs_grid[x][y].is_free_ocean:

def boat_next_sVec(start_sVec:sVec, action: str) -> sVec:
    
    if action == "ship":
        cur_sVec = sVec(start_sVec.pos + Boat_Ship_Offset[start_sVec.dir], start_sVec.dir)
    elif action == "rot 0":
        cur_sVec = rotate_one_move_no_test(start_sVec, 0)
    elif action == "rot 1":
        cur_sVec = rotate_one_move_no_test(start_sVec, 1)
    else:
        cur_sVec = start_sVec

    return cur_sVec

def rotate_one_move_no_test(cur_sVec: sVec, rotation):
    x = cur_sVec.x
    y = cur_sVec.y
    current_direction = cur_sVec.dir
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

    new_sVec = sVec(Point(new_x, new_y), new_direction)

    return new_sVec