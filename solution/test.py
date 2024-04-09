import matplotlib.pyplot as plt

# 定义网格数据
grid = [
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 9]
]

# 绘制网格
plt.imshow(grid, cmap='viridis', interpolation='nearest')
plt.colorbar()  # 添加颜色条
plt.savefig('fg.png')
