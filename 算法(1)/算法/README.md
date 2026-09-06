# 昌平线最小实例初版算法

## 作用

该代码用于验证“无人机—自走式机巢”模型的基本状态转移是否正确，暂不追求大规模求解性能。

状态定义为：

`当前时刻 + 机巢所在站 + 无人机SOC + 已完成任务集合 + 已换电次数`

算法使用事件驱动的状态网络和标签设置法，不依赖 Gurobi、CPLEX、OR-Tools 等第三方求解器。

## 目录

- `changping_min_solver.py`：求解器。
- `data/changping_feasible_demo.json`：默认3任务可行演示场景。
- `data/changping_min_instance.json`：原始5任务压力测试实例。
- `results/summary.json`：求解状态、目标值、路径动作和诊断信息。
- `results/timeline.csv`：可以直接用Excel打开的车辆/无人机时间线。

## 运行

在本目录执行：

```powershell
python changping_min_solver.py
```

运行自动测试：

```powershell
python -m unittest -v
```

默认命令运行3任务可行演示场景。运行完整5任务压力测试：

```powershell
python changping_min_solver.py --input data/changping_min_instance.json --output-dir results_full
```

使用指定数量的备用电池：

```powershell
python changping_min_solver.py --spare-batteries 4 --output-dir results_feasible
```

允许车辆反向运行：

```powershell
python changping_min_solver.py --allow-reverse
```

## 已建模的弧

- 车辆搭载无人机移动弧；
- 无人机执行任务、机巢同步移动并在站点会合的联合服务弧；
- 电池更换弧。

等待不单独枚举为决策弧；当任务尚未进入时间窗时，等待时间自动并入服务弧。相同位置、SOC、任务集合和换电次数下，更早到达的状态会支配更晚到达的状态。

## 当前边界

- 1辆移动机巢、1架无人机；
- 线性走廊，默认车辆只从昌平西山口向南邵运行；
- 每架次只执行1个任务；
- 能量以离散SOC状态表示；
- 任务点为区间内部的无人机独有位置；
- 最小化顺序为完工时间、换电次数、无人机活动时间、车辆运行时间。

原5任务实例在120 min、2组备用电池且每架次只做1项任务时没有全任务可行解，因此保留为压力测试。程序会输出完成任务最多的部分方案，并自动检查增加备用电池后是否首次可行。这是数据与约束的诊断结果，不应把不可行实例当作算法故障。
