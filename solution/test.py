import numpy as np
import matplotlib.pyplot as plt

# 文件路径
file_path = 'solution/log/boat.log'

# 使用numpy读取数据：假设每个值都是浮点数，数据被空格分隔
data = np.loadtxt(file_path)

# 时间轴，假设每一行代表一个时间单位
time = np.arange(0, data.shape[0])

for j in range((data.shape[1]-1)//3):
    plt.figure(figsize=(15, 10))
    # 配置横坐标间隔为1000
    plt.xticks(np.arange(0, len(time) + 1, 500))
    plt.ylim(0, 2000)
    for i in range(1):  # 对于每个港口
        index = 3 * j + 1
        plt.plot(data[:, 0], data[:, index : index+3], label=f'Port {j}')

    plt.xlabel('zhen')
    plt.ylabel('cur_num_gds')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'./fg{j}')

# boat_berths_map = {
# 0 : [9, 0],
# 1 : [6, 4],
# 2 : [1, 8],
# 3 : [7, 2],
# 4 : [5, 3],
# }
# for j in range(5):
#     plt.figure(figsize=(15, 10))
#     # 配置横坐标间隔为1000
#     plt.xticks(np.arange(0, len(time) + 1, 500))
#     plt.ylim(0, 200)
#     for i in boat_berths_map[j]:  # 对于每个港口
#         plt.plot(data[:, 0], data[:, i+1], label=f'Port {i}')
#     plt.xlabel('zhen')
#     plt.ylabel('Cargo Arrival Rate')
#     plt.legend()
#     plt.grid(True)
#     plt.savefig(f'./solution/test/boat_group_{j}.png')