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

## 解决方案架构设计
![](./docs/assets/architecture.png)
路径规划模块：负责提供描述地图所需的数据结构

核心模块：
- 负责提供具备执行**声明式命令**能力的建模对象，例如 Robot 类对应的实例，声明式命令只需告诉他去某个点 A，他会自动控制到达A，而不需要在每一帧控制 Robot 的运动；他**通过向调度器提供声明式命令接口的方式，极大的减轻了调度者使用 Robot 的负担；并且使得 Robot 可以轻松更换内部的实现方法；**
- env：
	- 负责管理运行期间所有的上下文，包括地图、机器人组、船舶组、港口组等等
	- 负责处理判题器输入

 调度器：调度器拥有 env 实例，即其拥有调度所需的所有信息；此处使用调度算法，声明式地命令机器人、船舶来进行取货、返港、装货、运输等等；


Log 模块：负责记录运行过程中产生的数据，用于 debug 和 优化调度算法

## 声明式 Robot 实现

基于状态机的思想实现，状态转换图如下：
![](./docs/assets/robot-state-machine.png)
### **声明式命令**
外部系统可以**通过 api 在合适的时机对状态进行切换**。此处何时的时机一般是指 robot *未初始化*、*on 港口*或者*取到货物*，即图中绿色的状态；

例如在 robot 处于港口时，命令其去某个位置取货，此时 robot 会切换到 *正在去取货* 状态，直到其取到货之前他不会切换到 *got货物*状态；

值得注意的是，机器人去取货的操作是内部逻辑处理的，不需要外部调用者关心，即**声明式**；外部调用者只需要关心货物是否已经被取到，即 robot 的状态是否切换到了 got货物。

### 实现原理
机器人会维护一个栈，里面保存了前往目标点需要经过的位置。每一帧，如果机器人状态改变，则重新计算路径；同时每一帧会弹出栈顶，并移动到弹出的位置![](./docs/assets/declare.png)

### 多机器人碰撞避免算法
简单来说就是每个机器人在每一帧检测是否会在下一帧发生碰撞，如果会，则进入避障状态，在避障状态中尝试找到不会发生碰撞的路径；实际上，实现鲁棒性较高的避障还需要回答几个问题
1. 谁避让谁：我通过给予不同状态不同的优先级来规定避障优先级；低优先级的机器人负责避让高优先级的机器人
2. 避让路径如何决定：我的主要策略是让处于避障状态的机器人寻找路径避开其负责（需要避障）的机器人
3. 如何避免多个机器人互相避让形成环；**为了解决成环问题，我们将机器人之间的避让关系以树的形式维护，并在进行新的避让时避免在树种形成环**。具体来说，子节点（处于避让状态的机器人）会让周围所有不是他祖先的节点（它负责避让的机器人）对他自己进行避让；

    例如下图中：
![](./docs/assets/collision-avoidance.png)
③ 负责避让 ② 和 ①；如果他在避让 ① 和 ② 的过程中碰见 ④，⑤，⑥ 中的任意一个节点，③ 会让他们进入避障状态；假设 ③ 分别需要让 ④ 和 ⑤ 进行避让，则他们会变成如下树状关系：
![](./docs/assets/tree.png)

### boat 的避障算法 
Boat 的设计思路与 Robot 非常相似，均提供声明式的 api，内部由状态机实现，并将路径存储起来。

不同之处在于避障算法，由于货船既能旋转也能前进，并且其体积不是矩形，想要像小车一样在快要碰撞时互相避让似乎十分困难（实际是否可以实现由另一位同学尝试，但似乎不太顺利）。。。而我**考虑到船舶只会在几个固定点之间移动，这就构成了很多条路径，我直接考虑对目前有船在运动的路径进行上锁（独占道路）来避免碰撞**。具体回答几个问题：
1. 如何规划路段，使其能独占而不碰撞：
	以下图为例，假设从 A -> B 和 C -> D 分别存在 1，2 两条路段，且两路段存在交叉；若是想让船舶在运行时不碰撞，那么交叉路段必须被上锁（独占）。因此将交叉路段单独编号为 3，此时，若船舶想从 A 到 B，他必须获得 1 和 3 号路段的锁；若无法获得，则会被拒绝移动（值得注意的是，移动的起点和终点是无视碰撞的）。**总的来说，就是将路段根据其被共享的情况进行划分，例如 3 号路段是由 1，2 路段共享造成的**。
![](./docs/assets/path-segment.png)
2. 如何在避免死锁的前提下，最大可能地减少货船因路段上锁而停止运动的可能性：学艺不精的再次体现，这似乎与银行家算法有关。但是**目前方案的做法就是提前获取全部所需要路段的锁，并在离开某个路段时释放其对应的锁**。值得注意的是，为了能够提前释放锁，道路编号需要进行如下图所示的改动，即将原来不连续但编号相同的路段重新编号。如此能在离开 1 号路段时释放 1 号路段的锁，并继续保持对 1.1 号路段的锁定。
![](./docs/assets/more-segments.png)
## 调度策略
### 机器人取货策略
1. 货物收益率最大化：货物会在地图上随机生成，将这个货物取到港口耗时 t，产生 v 的价值，计算收益率为 v/t，显然越近的港口造成的 t 一般会越小。所以，**一个朴素的想法就是把货物取到最近的港口**
2. 平均分配：我们假设每个港口分配一个机器人。实践发现，只按照上面这种方法分配货物，可能会出现货物聚集在某个港口，导致其他港口机器人空闲的情况；**解决这个问题的朴素想法是将货物分配给离他最近的前三个港口去取货**。实践证明，在有限时间内，这种方法可以很大地提高取货效率。
3. 帮忙取货：每个货物存在的时间是有限制的，一个繁忙的港口可能会导致货物丢失。我的想法是**让空闲的港口或者当前货物收益率过低的港口在不影响自己取货的前提下，去帮忙取那些其他港口可能会丢失的货物**。

总结：其实这个一个排队论的问题，奈何学艺不精。。。

### 初赛船舶调度策略
发现 船舶的运输能力 > 港口的集货能力；即只要船舶正常往返港口和卖货点之间，港口里的货不可能堆积；

因此，**调度策略的关键在于倒计时结束时，船舶能将最后一批货送到卖货点**；实现的方法也非常简单，就是根据船舶的往返周期时长，保证在倒计时结束前不久，船舶恰好返回到卖货点；

**值得一提的是，这个“显而易见”的策略，是团队分析了港口货物数对于时间的曲线的图表才发现的。**


## 项目实践过程中遇见的问题
### 初始化超时问题
在最开始时，我们要用 bfs 搜索港口到地图上任意一点的路径，以及货船停泊点之间的路线，此过程会发生超时；

解决：比赛环境提供双核环境，使用多进程并行计算（因为是计算密集型，python 的多线程由于全局锁的缘故效果不大）；