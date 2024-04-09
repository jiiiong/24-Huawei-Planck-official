from typing import List, Tuple, Dict, Any

from .base import Pixel_Attrs, Point, Point
from .base import Robot_Move, INFINIT_COST


def apply_move_grid_to_ch_grid(ch: List[List[str]], move: List[List[Point]]) -> List[List[str]]:
    from copy import deepcopy
    new_ch = deepcopy(ch)

    for x in range(len(move)):
        for y in range(len(move[0])):
            c = move[x][y]
            if (c == Robot_Move.UP):
                new_ch[x][y] = '↑'
            elif (c == Robot_Move.DOWN):
                new_ch[x][y] = '↓'
            elif (c == Robot_Move.LEFT):
                new_ch[x][y] = '←'
            elif (c == Robot_Move.RIGHT):
                new_ch[x][y] = '→'
    return new_ch

def save_grid_to_file(grid: List[List[Any]], name: str = 'unnamed') -> None:
    from pathlib import Path
    '''将矩阵[[]]存到文件中'''

    index = 0
    file_name = Path("solution/grids/" + name + str(index))
    while file_name.exists():
        index += 1
        file_name = Path("solution/grids/" + name + str(index))

    with open(file_name, "w+") as fd:
        for x in range(len(grid)):
            line = str(' ').join([str(item) for item in grid[x]])
            fd.write(line + "\n")