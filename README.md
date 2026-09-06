# 昌平线无人机—移动机巢协同巡检实验

本仓库包含昌平线最小实例数据、事件驱动状态网络求解器、结果合法性校验、时间窗口与备用电池敏感性实验，以及CSV、JSON和Excel结果。

主要内容位于[`算法(1)/算法`](算法(1)/算法)：

- `changping_min_solver.py`：单次场景求解器；
- `run_experiments.py`：运行16组时间窗口×备用电池实验；
- `test_solver.py`：自动测试；
- `experiments/experiment_matrix.csv`：综合实验结果；
- `experiments/experiment_results.xlsx`：便于查看和汇报的结果工作簿；
- `experiments/h*/summary.json`与`timeline.csv`：各实验详细结果。

详细运行方式、模型边界和结果字段参见[`算法(1)/算法/README.md`](算法(1)/算法/README.md)。

关键结论：135分钟、4组备用电池是当前参数组合中首次完成全部5项任务的场景，最优完工时间为135分钟，实际换电4次，最终SOC为40%。
