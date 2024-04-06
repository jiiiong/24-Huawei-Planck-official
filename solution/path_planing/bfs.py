from queue import Queue, PriorityQueue
from typing import List, Tuple, Dict, Any

from cfg import N
from log import main_logger

from .base import Pixel_Attrs, Point, sVec
from .base import Robot_Move, INFINIT_COST
from .base import Boat_Action, Boat_Direction, Boat_Ship_Offset

# robot相关

robot_move_list = [Robot_Move.UP, Robot_Move.DOWN, Robot_Move.LEFT, Robot_Move.RIGHT]

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

def boat_bfs(task_id: Tuple[sVec,sVec],
             attrs_grid: List[List[Pixel_Attrs]], boat_valid_grid:List[List[ (List[Boat_Direction]) ]],
             start_sVec: sVec, end_sVec: sVec) -> Tuple[Tuple[sVec,sVec], List[str]]:
    queue: PriorityQueue[Tuple[float, int, sVec, List[str]]] = PriorityQueue()
    queue.put((0, 0, start_sVec, []))
    visited = set()
    g_scores = {start_sVec: 0}
    ret = []
    count = 0
    while not queue.empty():
        count += 1
        (f_current, g_current, cur_sVec, actions) = queue.get()
        cur_sVec: sVec
        actions: List[str]
        g_current: int

        # main_logger.error(f"!! {cur_sVec}")
        if cur_sVec == end_sVec:
            main_logger.error(f"--{start_sVec}, {end_sVec}, {count}")
            # main_logger.error(f"{actions}") 
            return task_id, actions
        if cur_sVec in visited:
            continue
        visited.add(cur_sVec)

        # 尝试 ship_one_move
        okk, new_sVec = ship_one_move(cur_sVec, boat_valid_grid, attrs_grid)
        # new_sVec = sVec(cur_sVec.pos+Boat_Ship_Offset[cur_sVec.dir], cur_sVec.dir)
        # okk = ship_position_available(new_sVec, boat_valid_grid)
        if okk:
            new_g = g_current + boat_gen_cost(attrs_grid, new_sVec)  # 假设每次移动成本为1
            if new_sVec not in g_scores or new_g < g_scores[new_sVec]:
                g_scores[new_sVec] = new_g
                f = new_g + 1 * heuristic(new_sVec.pos, end_sVec.pos) 
                queue.put((f, new_g, new_sVec, actions + ["ship"]))

        # 尝试 rotate_one_move，顺时针和逆时针旋转
        for rotation in [1, 0]:
            okk, new_sVec = rotate_one_move(cur_sVec, rotation, boat_valid_grid)
            if okk:
                new_g = g_current + boat_gen_cost(attrs_grid, new_sVec)  # 假设旋转成本为1
                if new_sVec not in g_scores or new_g < g_scores[new_sVec]:
                    g_scores[new_sVec] = new_g
                    f = new_g + 1 * heuristic(new_sVec.pos, end_sVec.pos)
                    queue.put((f, new_g, new_sVec, actions + ["rot " + str(rotation)]))

    main_logger.error(f"++{count}")     
    return task_id, ret

def boat_gen_cost(attrs_grid: List[List[Pixel_Attrs]], cur_sVec: sVec) -> int:
    lt_pos, rb_pos = cur_sVec.proj()
    for x in range(lt_pos.x, rb_pos.x+1):
        for y in range(lt_pos.y, rb_pos.y+1):
            if attrs_grid[x][y].is_free_ocean:
                #main_logger.error("dsbiusbdo")
                return 2
    #main_logger.error("shit")
    return 1

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
    # 使用曼哈顿距离作为启发式函数
    return abs(a.x - b.x) + abs(a.y - b.y)