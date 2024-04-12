import numpy as np

grid: np.ndarray = np.full((200,200), '*')
grid[0][0] = 'adas'
print(grid.shape, grid.dtype)
print(grid[0][0], grid[1][1])
