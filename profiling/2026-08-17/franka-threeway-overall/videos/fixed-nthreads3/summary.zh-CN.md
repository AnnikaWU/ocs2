# Franka 三组位姿对比摘要

正式对照是三组，不是 NextGen 单组：Pinocchio 原几何、Pinocchio 球体几何、NextGen 球体几何。
所有数值均由既有 offline FK 轨迹生成；未重新运行在线 MPC。SUCCESS 要求终端 0.25 s 保持在 3 cm / 0.1 rad 内；其余标为 POSE_PERIODIC。
周期候选为 0.5–10 s，避免把相邻 100 Hz 样本误当成振荡周期。每个目标视频均为同一物理时间的三栏同步画面。

## 目标 1：near_initial_fk_sanity

视频：[`targets/target_01_near_initial_fk_sanity_threeway.mp4`](targets/target_01_near_initial_fk_sanity_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_01_near_initial_fk_sanity_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_01_near_initial_fk_sanity_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_01_near_initial_fk_sanity_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 63.3 | 0.263 | 1.00 | 0.0 / 0.000 | 0.0 / 0.000 | 0.2 / 0.001 |
| Pinocchio 球体几何 | POSE_PERIODIC | 46.5 | 0.260 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.4 / 0.001 |
| NextGen 球体几何 | POSE_PERIODIC | 46.5 | 0.260 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.4 / 0.001 |

## 目标 2：forearm_joint_region

视频：[`targets/target_02_forearm_joint_region_threeway.mp4`](targets/target_02_forearm_joint_region_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_02_forearm_joint_region_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_02_forearm_joint_region_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_02_forearm_joint_region_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 214.0 | 0.159 | 5.50 | 0.0 / 0.000 | 0.0 / 0.000 | 4.3 / 0.014 |
| Pinocchio 球体几何 | POSE_PERIODIC | 253.2 | 0.087 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 2.5 / 0.010 |
| NextGen 球体几何 | POSE_PERIODIC | 253.2 | 0.087 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 2.5 / 0.010 |

## 目标 3：near_base_front

视频：[`targets/target_03_near_base_front_threeway.mp4`](targets/target_03_near_base_front_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_03_near_base_front_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_03_near_base_front_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_03_near_base_front_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 167.9 | 0.125 | 0.50 | 0.0 / 0.000 | 390.1 / 1.432 | 7.5 / 0.015 |
| Pinocchio 球体几何 | POSE_PERIODIC | 951.9 | 1.656 | 0.50 | 105.4 / 0.976 | 452.0 / 1.463 | 1082.2 / 3.142 |
| NextGen 球体几何 | POSE_PERIODIC | 289.6 | 2.118 | 0.50 | 293.2 / 0.976 | 297.4 / 0.905 | 1708.7 / 3.142 |

## 目标 4：rear_center_difficult

视频：[`targets/target_04_rear_center_difficult_threeway.mp4`](targets/target_04_rear_center_difficult_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_04_rear_center_difficult_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_04_rear_center_difficult_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_04_rear_center_difficult_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 540.7 | 2.609 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.0 / 0.000 |
| Pinocchio 球体几何 | POSE_PERIODIC | 279.9 | 0.256 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.8 / 0.002 |
| NextGen 球体几何 | POSE_PERIODIC | 150.9 | 0.272 | 1.00 | 0.0 / 0.000 | 0.0 / 0.000 | 120.2 / 0.134 |

## 目标 5：low_near_base_side

视频：[`targets/target_05_low_near_base_side_threeway.mp4`](targets/target_05_low_near_base_side_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_05_low_near_base_side_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_05_low_near_base_side_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_05_low_near_base_side_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 267.8 | 0.519 | 0.50 | 14.0 / 0.023 | 5.6 / 0.012 | 73.0 / 0.111 |
| Pinocchio 球体几何 | POSE_PERIODIC | 422.4 | 1.446 | 0.50 | 489.3 / 1.474 | 534.8 / 1.675 | 509.7 / 2.689 |
| NextGen 球体几何 | POSE_PERIODIC | 209.9 | 0.225 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.4 / 0.001 |

## 目标 6：cross_body_wrist_flip

视频：[`targets/target_06_cross_body_wrist_flip_threeway.mp4`](targets/target_06_cross_body_wrist_flip_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_06_cross_body_wrist_flip_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_06_cross_body_wrist_flip_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_06_cross_body_wrist_flip_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 96.5 | 0.168 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.4 / 0.001 |
| Pinocchio 球体几何 | POSE_PERIODIC | 102.6 | 0.101 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.2 / 0.001 |
| NextGen 球体几何 | POSE_PERIODIC | 110.3 | 0.066 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.5 / 0.001 |
