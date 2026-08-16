# Franka 三组位姿对比摘要

正式对照是三组，不是 NextGen 单组：Pinocchio 原几何、Pinocchio 球体几何、NextGen 球体几何。
所有数值均由既有 offline FK 轨迹生成；未重新运行在线 MPC。SUCCESS 要求终端 0.25 s 保持在 3 cm / 0.1 rad 内；其余标为 POSE_PERIODIC。
周期候选为 0.5–10 s，避免把相邻 100 Hz 样本误当成振荡周期。每个目标视频均为同一物理时间的三栏同步画面。

## 目标 1：near_initial_fk_sanity

视频：[`targets/target_01_near_initial_fk_sanity_threeway.mp4`](targets/target_01_near_initial_fk_sanity_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_01_near_initial_fk_sanity_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_01_near_initial_fk_sanity_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_01_near_initial_fk_sanity_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 63.3 | 0.263 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.2 / 0.002 |
| Pinocchio 球体几何 | POSE_PERIODIC | 46.7 | 0.260 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.3 / 0.001 |
| NextGen 球体几何 | POSE_PERIODIC | 46.7 | 0.260 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.3 / 0.001 |

## 目标 2：forearm_joint_region

视频：[`targets/target_02_forearm_joint_region_threeway.mp4`](targets/target_02_forearm_joint_region_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_02_forearm_joint_region_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_02_forearm_joint_region_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_02_forearm_joint_region_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 214.2 | 0.158 | 9.50 | 0.7 / 0.003 | 0.8 / 0.003 | 5.5 / 0.020 |
| Pinocchio 球体几何 | POSE_PERIODIC | 253.2 | 0.087 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 2.6 / 0.010 |
| NextGen 球体几何 | POSE_PERIODIC | 253.2 | 0.087 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 2.6 / 0.010 |

## 目标 3：near_base_front

视频：[`targets/target_03_near_base_front_threeway.mp4`](targets/target_03_near_base_front_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_03_near_base_front_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_03_near_base_front_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_03_near_base_front_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 199.2 | 0.730 | 10.00 | 2.9 / 0.007 | 4.1 / 0.009 | 31.3 / 0.076 |
| Pinocchio 球体几何 | POSE_PERIODIC | 480.3 | 2.491 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.0 / 0.000 |
| NextGen 球体几何 | POSE_PERIODIC | 480.3 | 2.491 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.0 / 0.000 |

## 目标 4：rear_center_difficult

视频：[`targets/target_04_rear_center_difficult_threeway.mp4`](targets/target_04_rear_center_difficult_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_04_rear_center_difficult_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_04_rear_center_difficult_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_04_rear_center_difficult_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 202.4 | 0.389 | 2.50 | 80.5 / 0.103 | 6.7 / 0.009 | 89.6 / 0.129 |
| Pinocchio 球体几何 | POSE_PERIODIC | 286.4 | 0.303 | 1.00 | 0.0 / 0.000 | 0.0 / 0.000 | 7.2 / 0.007 |
| NextGen 球体几何 | POSE_PERIODIC | 286.4 | 0.303 | 1.00 | 0.0 / 0.000 | 0.0 / 0.000 | 7.2 / 0.007 |

## 目标 5：low_near_base_side

视频：[`targets/target_05_low_near_base_side_threeway.mp4`](targets/target_05_low_near_base_side_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_05_low_near_base_side_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_05_low_near_base_side_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_05_low_near_base_side_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 344.2 | 0.478 | 7.50 | 17.4 / 0.026 | 17.7 / 0.026 | 112.6 / 0.179 |
| Pinocchio 球体几何 | POSE_PERIODIC | 209.9 | 0.225 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.4 / 0.001 |
| NextGen 球体几何 | POSE_PERIODIC | 209.9 | 0.225 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.4 / 0.001 |

## 目标 6：cross_body_wrist_flip

视频：[`targets/target_06_cross_body_wrist_flip_threeway.mp4`](targets/target_06_cross_body_wrist_flip_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_06_cross_body_wrist_flip_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_06_cross_body_wrist_flip_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_06_cross_body_wrist_flip_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 355.6 | 0.454 | 5.00 | 1.3 / 0.006 | 0.7 / 0.003 | 0.6 / 0.002 |
| Pinocchio 球体几何 | POSE_PERIODIC | 102.4 | 0.101 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.4 / 0.001 |
| NextGen 球体几何 | POSE_PERIODIC | 102.4 | 0.101 | 0.50 | 0.0 / 0.000 | 0.0 / 0.000 | 0.4 / 0.001 |
