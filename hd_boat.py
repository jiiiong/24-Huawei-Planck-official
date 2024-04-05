import sys
import random
from log import logger
from enum import Enum
from collections import deque
import time
from queue import PriorityQueue

N = 200

robot_num = 20
boat_num = 10
berth_num = 10

goods_num = 0
frame_id = 0
money = 0
boat_capacity = 0
boat_price=8000
robot_price=2000

grid = []

map_for_boat = []
map_for_robot = []
direction_matrix = [[[] for _ in range(N)] for _ in range(N)]  # 正确初始化二维数组


# 定义一个枚举类来表示动作
class Action(Enum):
    SHIP = "ship"
    ROTATE1 = "rotate1"
    ROTATE0 = "rotate0"
    STAY = "stay"

# 定义一个枚举类来表示方向
class Direction(Enum):
    DOWN = "D"
    RIGHT = "R"
    UP = "U"
    LEFT = "L"

class Position:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance(self, other):
        return abs(self.x - other.x) + abs(self.y - other.y)

    # 实现小于比较
    def __lt__(self, other):
        return (self.x, self.y) < (other.x, other.y)


def process_robot_map(grid):
    global map_for_robot

    for i in range(N):
        newLine = []
        for j in range(N):
            if grid[i][j] == '.' or grid[i][j] == 'C':
                newLine.append(1)
            elif grid[i][j] == '>' or grid[i][j] == 'R' or grid[i][j] == 'B' or \
                    grid[i][j] == 'c':
                newLine.append(2)
            else:
                newLine.append(0)
        # newLine_str = ' '.join(str(e) for e in newLine)
        # logger.info(newLine_str)
        map_for_robot.append(newLine)

def process_boat_map(grid):
    global map_for_boat

    for i in range(N):
        newLine = []
        for j in range(N):
            if grid[i][j] == '*' or grid[i][j] == 'C':
                newLine.append(1)
            elif grid[i][j] == '~' or grid[i][j] == 'S' or grid[i][j] == 'B' or grid[i][j] == 'K' or\
                    grid[i][j] == 'c' or grid[i][j] == 'T':
                newLine.append(2)
            else:
                newLine.append(0)
        # newLine_str = ' '.join(str(e) for e in newLine)
        # logger.info(newLine_str)
        map_for_boat.append(newLine)

def init_direction_matrix():
    global direction_matrix

    for i in range(N):
        for j in range(N):
            directions = []
            if map_for_boat[i][j] != 0:
                if can_place_down(i, j): directions.append(Direction.DOWN)
                if can_place_right(i, j): directions.append(Direction.RIGHT)
                if can_place_up(i, j): directions.append(Direction.UP)
                if can_place_left(i, j): directions.append(Direction.LEFT)
            direction_matrix[i][j] = directions  # 正确地为每个点赋值
            # logger.info("pos %d %d", i, j)
            # logger.info(direction_matrix[i][j])
            # if len(directions) != 0:
            #     logger.info("pos %d %d", i, j)
            #     for direction in directions:
            #         logger.info(direction)

def can_place_down(x, y):
    if x + 2 >= N or y - 1 < 0 or x + 1 >= N:
        return False
    for i in range(x, x + 2):
        for j in range(y - 1, y):
            if map_for_boat[i][j] == 0:
                return False
    return True

def can_place_up(x, y):
    if y + 1 >= N or x - 2 < 0 or x - 1 < 0:
        return False
    for i in range(x - 2, x):
        for j in range(y, y + 1):
            if map_for_boat[i][j] == 0:
                return False
    return True

def can_place_left(x, y):
    if x - 1 < 0 or y - 1 < 0 or y - 2 < 0:
        return False
    for i in range(x - 1, x):
        for j in range(y - 2, y):
            if map_for_boat[i][j] == 0:
                return False
    return True

def can_place_right(x, y):
    if x + 1 >= N or y + 1 >= N or y + 2 >= N:
        return False
    for i in range (x , x + 1):
        for j in range (y, y + 2):
            if map_for_boat[i][j] == 0:
                return False
    return True

# 用于前进 ship, 返回一个 bool 类型表示成功或失败，返回一个新的坐标，表示移动后核心点的位置
def ship_one_move(current_direction, current_pos, status_matrix):
    # logger.info("func: ship_one_move current_direction: %s, current_pos: %d %d", current_direction, current_pos.x , current_pos.y)
    okk = False
    # 向上移动一步
    if current_direction == Direction.UP and current_pos.x - 1 >= 0 and current_direction in status_matrix[current_pos.x - 1][current_pos.y]:
        new_pos = Position(current_pos.x - 1, current_pos.y)
        okk = True
        return okk, new_pos
    # 向下移动一步
    elif current_direction == Direction.DOWN and current_pos.x + 1 < N and current_direction in status_matrix[current_pos.x + 1][current_pos.y]:
        new_pos = Position(current_pos.x + 1, current_pos.y)
        okk = True
        return okk, new_pos
    # 向左移动一步
    elif current_direction == Direction.LEFT and current_pos.y - 1 >= 0 and current_direction in status_matrix[current_pos.x][current_pos.y - 1]:
        new_pos = Position(current_pos.x, current_pos.y - 1)
        okk = True
        return okk, new_pos
    # 向右移动一步
    elif current_direction == Direction.RIGHT and current_pos.y + 1 < N and current_direction in status_matrix[current_pos.x][current_pos.y + 1]:
        new_pos = Position(current_pos.x, current_pos.y + 1)
        okk = True
        return okk, new_pos
    else:
        return False, None

# 用于旋转 rotate，返回一个 bool 类型表示成功或者失败，返回一个新的坐标，表示旋转后核心点的位置，返回一个新的方向
def rotate_one_move(x, y, current_direction, rotation):
    # 方向转换逻辑
    def direction_transfer(current_direction, rotation):
        direction_order = [Direction.RIGHT, Direction.DOWN, Direction.LEFT, Direction.UP]
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
        if current_direction == Direction.RIGHT:
            new_x, new_y = x, y + 2
        elif current_direction == Direction.DOWN:
            new_x, new_y = x + 2, y
        elif current_direction == Direction.LEFT:
            new_x, new_y = x, y - 2
        elif current_direction == Direction.UP:
            new_x, new_y = x - 2, y
    else:  # 逆时针旋转
        if current_direction == Direction.RIGHT:
            new_x, new_y = x + 1, y + 1
        elif current_direction == Direction.DOWN:
            new_x, new_y = x + 1, y - 1
        elif current_direction == Direction.LEFT:
            new_x, new_y = x - 1, y - 1
        elif current_direction == Direction.UP:
            new_x, new_y = x - 1, y + 1

    new_position = Position(new_x, new_y)
    okk = ship_position_available(new_position, new_direction, direction_matrix, map_for_boat)

    return okk, new_position, new_direction

# 判断当前船的位置是否合法
def ship_position_available(current_position, current_direction, status_matrix, boat_map):
    if current_position.x < 0 or current_position.x >= N or current_position.y < 0 or current_position.y >= N:
        return False
    if boat_map[current_position.x][current_position.y] == 0:
        return False
    if current_direction not in status_matrix[current_position.x][current_position.y]:
        return False
    return True


def bfs(start_pos, start_direction, end_pos):
    # logger.info("start_pos: %d %d", start_pos.x, start_pos.y)
    # logger.info("start_direction: %s", start_direction)


    # 初始化队列，每个元素是（当前位置，当前方向，到达这里的动作队列）
    queue = deque([(start_pos, start_direction, [])])
    visited = set()
    # logger.info("while queue start")
    while queue:
        current_position, current_direction, actions = queue.popleft()
        # logger.info("current_position: %d %d current_direction: %s", current_position.x, current_position.y, current_direction)
        if current_position.x == end_pos.x and current_position.y == end_pos.y:
            return actions
        # 防止重复访问
        if (current_position.x, current_position.y, current_direction) in visited:
            continue
        visited.add((current_position.x, current_position.y, current_direction))

        # 尝试执行 ship_one_move
        okk, new_position = ship_one_move(current_direction, current_position, direction_matrix)
        if okk:
            okk = ship_position_available(new_position, current_direction, direction_matrix, map_for_boat)
        if okk:
            # logger.info("ship to new position: %d %d", new_position.x, new_position.y)
            queue.append((new_position, current_direction, actions + ["ship"]))

        # 尝试执行 rotate_one_move，顺时针和逆时针旋转
        for rotation in [0, 1]:
            okk, new_pos_after_rot, new_dir_after_rot = rotate_one_move(current_position.x, current_position.y, current_direction, rotation)
            if okk:
                # logger.info("new_pos_after_rot: %d %d new_dir_after_rot %s", new_pos_after_rot.x, new_pos_after_rot.y, new_dir_after_rot)
                queue.append((new_pos_after_rot, new_dir_after_rot, actions + [f"rot {rotation}"]))
    return [] # 如果没有找到，返回空列表

class Robot:
    def __init__(self, id=0, startX=0, startY=0, goods=0):
        self.id = id
        self.x = startX
        self.y = startY
        self.goods = goods

robot = [Robot() for _ in range(robot_num)]

class Berth:
    def __init__(self, x=0, y=0, loading_speed=0):
        self.x = x
        self.y = y
        self.loading_speed = loading_speed

berth = [Berth() for _ in range(berth_num)]

class Boat:
    def __init__(self, id=0, x=0, y=0, dir=0, num=0, status=0):
        self.id = id
        self.num = num
        self.x = x
        self.y = y
        self.dir = dir
        self.status = status

boat = [Boat() for _ in range(10)]

robot_purchase_point = []
boat_purchase_point = []
delivery_point = []

def process_map(grid):
    for i in range(N):
        for j in range(N):
            if grid[i][j] == 'R':
                robot_purchase_point.append((i, j))
            elif grid[i][j] == 'S':
                boat_purchase_point.append((i, j))
            elif grid[i][j] == 'T':
                delivery_point.append((i, j))

def Init():
    for i in range(0, N):
        line = input()
        grid.append(list(line))
        # logger.info(line)
    process_map(grid)
    process_boat_map(grid)
    process_robot_map(grid)
    start_time = time.time()
    logger.info("init_direction_matrix start at %.2f", start_time)
    init_direction_matrix()
    end_time = time.time()
    logger.info("init_direction_matrix cost %.2f", (end_time - start_time) * 1000)
    berth_num = int(input())
    for i in range(berth_num):
        id, x, y, loading_speed = map(int, input().split())
        berth.append(Berth(x, y, loading_speed))

    boat_capacity = int(input())
    okk = input()
    print("OK")

def Input():
    goods_num = int(input())
    for i in range(goods_num):
        x, y, val = map(int, input().split())
        grid[x][y] = val

    global robot_num
    robot_num = int(input())
    for i in range(robot_num):
        robot[i].id, robot[i].goods, robot[i].x, robot[i].y = map(int, input().split())

    global boat_num
    boat_num = int(input())
    for i in range(boat_num):
        boat[i].id, boat[i].goods_num, boat[i].x, boat[i].y, boat[i].dir, boat[i].status = map(int, input().split())
    okk = input()

def transfer_direction(boat_dir):
    if boat_dir == 0:
        return Direction.RIGHT
    elif boat_dir == 1:
        return Direction.LEFT
    elif boat_dir == 2:
        return Direction.UP
    elif boat_dir == 3:
        return Direction.DOWN


# 启发式函数
def heuristic(a, b):
    # 使用曼哈顿距离作为启发式函数
    return abs(a.x - b.x) + abs(a.y - b.y)


def a_star(start_pos, start_direction, end_pos):
    # 初始化优先级队列，每个元素格式为 (f(n), g(n), 当前位置, 当前方向, 到达这里的动作队列)
    # 注意 PriorityQueue 使用元组的第一个元素作为比较的依据
    queue = PriorityQueue()
    queue.put((0, 0, start_pos, start_direction, []))

    visited = set()
    g_scores = {start_pos: 0}

    while not queue.empty():
        (f_current, g_current, current_position, current_direction, actions) = queue.get()

        if current_position.x == end_pos.x and current_position.y == end_pos.y:
            return actions

        if (current_position.x, current_position.y, current_direction) in visited:
            continue
        visited.add((current_position.x, current_position.y, current_direction))

        # 尝试 ship_one_move
        okk, new_position = ship_one_move(current_direction, current_position, direction_matrix)
        if okk:
            okk = ship_position_available(new_position, current_direction, direction_matrix, map_for_boat)
        if okk:
            new_g = g_current + 1  # 假设每次移动成本为1
            if new_position not in g_scores or new_g < g_scores[new_position]:
                g_scores[new_position] = new_g
                f = new_g + heuristic(new_position, end_pos)
                queue.put((f, new_g, new_position, current_direction, actions + ["ship"]))

        # 尝试 rotate_one_move，顺时针和逆时针旋转
        for rotation in [0, 1]:
            okk, new_pos_after_rot, new_dir_after_rot = rotate_one_move(current_position.x, current_position.y,
                                                                        current_direction, rotation)
            if okk:
                new_g = g_current + 1  # 假设旋转成本为1
                if new_pos_after_rot not in g_scores or new_g < g_scores[new_pos_after_rot]:
                    g_scores[new_pos_after_rot] = new_g
                    f = new_g + heuristic(new_pos_after_rot, end_pos)
                    queue.put((f, new_g, new_pos_after_rot, new_dir_after_rot, actions + [f"rot {rotation}"]))


if __name__ == "__main__":
    Init()
    B = Position(95, 26)
    A = Position(2, 195)
    count = 0
    actions = []
    while True:
        try:

            frame_id, money = map(int, input().split(" "))
            Input()

            if boat_num < 1:
                print("lboat", boat_purchase_point[0][0], boat_purchase_point[0][1])

            for i in range(robot_num):
                print("move", i, random.randint(0, 3))

            for i in range(boat_num):
                if boat[0].x == boat_purchase_point[0][0] and boat[0].y == boat_purchase_point[0][1] and count == 0:
                    direction = transfer_direction(boat[0].dir)
                    # start_time = time.time()
                    # logger.info("bfs start at %.2f", start_time)
                    # actions = bfs(Position(boat_purchase_point[0][0], boat_purchase_point[0][1]), direction, A)
                    # end_time = time.time()
                    # logger.info("bfs costs %.2f ms", (end_time - start_time) * 1000)
                    # logger.info(actions)
                    count += 1

                    start_time = time.time()
                    logger.info("astar start at %.2f", start_time)
                    actions = a_star(Position(boat_purchase_point[0][0], boat_purchase_point[0][1]), direction, A)
                    end_time = time.time()
                    logger.info("astar costs %.2f ms", (end_time - start_time) * 1000)
                    logger.info(actions)

                if frame_id > 100:
                    if i == 0 and boat[0].status == 0:
                        if actions:
                            action = actions.pop(0)
                            # logger.info(action)
                            if action == "ship":
                                print("ship", 0)
                            elif action == "rot 0":
                                print("rot", 0, 0)
                            elif action == "rot 1":
                                print("rot", 0, 1)
                # status = random.randint(0, 1)
                # if status == 0:
                #     print("ship", i)
                # else:
                #     print("rot", i, random.randint(0, 1))

            print("OK")
            sys.stdout.flush()

        except EOFError:
            break

