# Franka 三组闭环对照实验视频

本目录对应最终报告中的四种运行方式。每段主视频同步展示三种碰撞配置：

1. Pinocchio＋原始 Panda 几何
2. Pinocchio＋球体几何
3. NextGen＋球体几何

每种运行方式包含六个连续目标，每个目标运行 50 秒。主视频是压缩后的同步回放；`targets/` 目录同时提供六段三栏目标视频和十八段单配置目标视频。

## 主视频

- [默认运行：自适应积分](videos/default-adaptive/threeway_synchronized.mp4)
- [固定网格：nThreads=1](videos/fixed-nthreads1/threeway_synchronized.mp4)
- [固定网格：普通 nThreads=3](videos/fixed-nthreads3/threeway_synchronized.mp4)
- [固定网格：确定性 nThreads=3](videos/fixed-deterministic-nthreads3/threeway_synchronized.mp4)

## 目录结构

- `videos/<运行方式>/threeway_synchronized.mp4`：六个目标的三栏同步主视频。
- `videos/<运行方式>/threeway_synchronized_poster.png`：主视频加载前显示的封面图。
- `videos/<运行方式>/targets/target_*_threeway.mp4`：按目标拆分的三栏同步视频。
- `videos/<运行方式>/targets/<配置>/target_*.mp4`：按目标和碰撞配置拆分的单栏视频。
- `videos/<运行方式>/summary.zh-CN.md`：该运行方式的离线结果摘要。
- `evidence/`：确定性模式的重复运行记录和实现补丁。

这些视频均由实验结束后保存的在线轨迹离线生成；视频生成过程没有重新运行 MPC。
