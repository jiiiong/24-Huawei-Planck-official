from path_planing import Boat_Direction

class Boat:
    def __init__(self, id=0, x=0, y=0, dir=0, num=0, status=0):
        self.id = id
        self.goods_num = num
        self.x = x
        self.y = y
        self.dir = dir
        self.status = status

def transfer_direction(boat_dir):
    if boat_dir == 0:
        return Boat_Direction.RIGHT
    elif boat_dir == 1:
        return Boat_Direction.LEFT
    elif boat_dir == 2:
        return Boat_Direction.UP
    elif boat_dir == 3:
        return Boat_Direction.DOWN
    else:
        return ""

