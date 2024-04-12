import sys

def weighted_prim(adj_matrix):
    num_vertices = len(adj_matrix)
    mst = [None] * num_vertices  # 存储最小生成树的边
    key = [sys.maxsize] * num_vertices  # 用于存储每个顶点的关键值
    visited = [False] * num_vertices  # 记录每个顶点是否已经加入到最小生成树中
    key[0] = 0  # 选择第一个顶点作为起始顶点
    mst[0] = -1  # 根节点没有父节点

    for _ in range(num_vertices):
        u = min_key_vertex(key, visited)
        visited[u] = True
        for v in range(num_vertices):
            if adj_matrix[u][v] != 0 and not visited[v] and adj_matrix[u][v] < key[v]:
                mst[v] = u
                key[v] = adj_matrix[u][v]

    return construct_mst(adj_matrix, mst)

def min_key_vertex(key, visited):
    min_key = sys.maxsize
    min_index = -1
    for v in range(len(key)):
        if not visited[v] and key[v] < min_key:
            min_key = key[v]
            min_index = v
    return min_index

def construct_mst(adj_matrix, mst):
    mst_edges = []
    for i in range(1, len(mst)):
        mst_edges.append((mst[i], i, adj_matrix[i][mst[i]]))
    return mst_edges

# Example adjacency matrix
adj_matrix = [[0,  4,  3,  5,  0,  0,  0,  0],
[4,  0,  6,  0,  7,  0,  0,  0],
[3,  6,  0,  0,  0,  2,  0,  0],
[5,  0,  0,  0,  0,  0,  8,  0],
[0,  7,  0,  0,  0,  0,  0,  9],
[0,  0,  2,  0,  0,  0,  1,  0],
[0,  0,  0,  8,  0,  1,  0,  0],
[0,  0,  0,  0,  9,  0,  0,  0]]

print(weighted_prim(adj_matrix))


