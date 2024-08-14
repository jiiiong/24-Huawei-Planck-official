## 比赛背景

在智能港口领域，如何规划多机器人的任务执行以实现最优调度，如何控制泊位和机器人的配合实现最高运输价值等都是非常有价值的算法难题。本次比赛通过软件模拟了智能港口的状态信息，由选手来挑战这些有价值的算法难题。

## 效果展示
[展示视频网页地址：https://jiiiong.github.io/24-Huawei-Planck-official/](https://jiiiong.github.io/24-Huawei-Planck-official/)

## 如何运行

环境要求：
- python 3.7
- ubuntu 18.04
（其他环境配置未测试，更高版本一般可行）

运行命令：
```
git clone https://github.com/jiiiong/24-Huawei-Planck-official.git
cd 24-Huawei-Planck-official

# 该命令会启动判题器，根据 solution/main.py 控制机器人和船舶在 map 上运动；
# 最终运行结果保存在 /replay 下
./run_simple_demo.sh
```

回放结果：使用 /replayer 中的播放器可以播放 /replay 下的回放文件，播放器有 windows 和 linux 两个版本

## 解决方案的概念与架构
