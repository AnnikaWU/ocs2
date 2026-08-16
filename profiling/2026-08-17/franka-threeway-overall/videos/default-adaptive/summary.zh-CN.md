# Franka 三组位姿对比摘要

正式对照是三组，不是 NextGen 单组：Pinocchio 原几何、Pinocchio 球体几何、NextGen 球体几何。
所有数值均由既有 offline FK 轨迹生成；未重新运行在线 MPC。SUCCESS 要求终端 0.25 s 保持在 3 cm / 0.1 rad 内；其余标为 POSE_PERIODIC。
周期候选为 0.5–10 s，避免把相邻 100 Hz 样本误当成振荡周期。每个目标视频均为同一物理时间的三栏同步画面。

## 目标 1：near_initial_fk_sanity

视频：[`targets/target_01_near_initial_fk_sanity_threeway.mp4`](targets/target_01_near_initial_fk_sanity_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_01_near_initial_fk_sanity_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_01_near_initial_fk_sanity_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_01_near_initial_fk_sanity_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 203.7 | 0.150 | 9.55 | 138.8 / 0.244 | 113.8 / 0.505 | 833.5 / 3.141 |
| Pinocchio 球体几何 | POSE_PERIODIC | 60.3 | 0.231 | 1.00 | 1.1 / 0.007 | 5.4 / 0.014 | 41.4 / 0.100 |
| NextGen 球体几何 | POSE_PERIODIC | 941.3 | 2.898 | 1.21 | 1020.3 / 2.642 | 150.2 / 0.486 | 1291.0 / 3.142 |

## 目标 2：forearm_joint_region

视频：[`targets/target_02_forearm_joint_region_threeway.mp4`](targets/target_02_forearm_joint_region_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_02_forearm_joint_region_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_02_forearm_joint_region_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_02_forearm_joint_region_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 559.4 | 0.800 | 0.50 | 258.0 / 0.859 | 180.3 / 0.809 | 342.5 / 1.133 |
| Pinocchio 球体几何 | POSE_PERIODIC | 420.1 | 0.879 | 0.50 | 93.8 / 0.294 | 265.9 / 1.127 | 288.0 / 1.516 |
| NextGen 球体几何 | POSE_PERIODIC | 391.9 | 0.934 | 0.50 | 90.5 / 0.087 | 248.2 / 1.007 | 238.1 / 0.853 |

## 目标 3：near_base_front

视频：[`targets/target_03_near_base_front_threeway.mp4`](targets/target_03_near_base_front_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_03_near_base_front_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_03_near_base_front_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_03_near_base_front_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 83.9 | 0.086 | 1.94 | 99.8 / 0.117 | 45.6 / 0.058 | 102.3 / 0.126 |
| Pinocchio 球体几何 | POSE_PERIODIC | 546.5 | 0.715 | 0.50 | 34.7 / 0.175 | 50.0 / 0.250 | 113.8 / 0.418 |
| NextGen 球体几何 | POSE_PERIODIC | 290.6 | 0.341 | 0.50 | 32.9 / 0.493 | 418.0 / 1.430 | 723.3 / 3.097 |

## 目标 4：rear_center_difficult

视频：[`targets/target_04_rear_center_difficult_threeway.mp4`](targets/target_04_rear_center_difficult_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_04_rear_center_difficult_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_04_rear_center_difficult_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_04_rear_center_difficult_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 690.9 | 2.349 | 0.50 | 27.1 / 0.083 | 335.5 / 1.361 | 282.4 / 1.051 |
| Pinocchio 球体几何 | POSE_PERIODIC | 172.8 | 1.899 | 0.50 | 69.1 / 0.199 | 97.2 / 0.397 | 270.7 / 1.197 |
| NextGen 球体几何 | POSE_PERIODIC | 76.5 | 0.433 | 0.50 | 133.7 / 0.466 | 186.5 / 0.705 | 410.9 / 1.249 |

## 目标 5：low_near_base_side

视频：[`targets/target_05_low_near_base_side_threeway.mp4`](targets/target_05_low_near_base_side_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_05_low_near_base_side_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_05_low_near_base_side_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_05_low_near_base_side_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 459.6 | 0.370 | 0.50 | 10.9 / 0.032 | 10.1 / 0.039 | 44.2 / 0.165 |
| Pinocchio 球体几何 | POSE_PERIODIC | 338.5 | 0.714 | 0.50 | 11.4 / 0.047 | 79.5 / 0.481 | 150.2 / 0.686 |
| NextGen 球体几何 | POSE_PERIODIC | 510.0 | 0.794 | 8.50 | 15.4 / 0.062 | 36.0 / 0.170 | 213.9 / 0.952 |

## 目标 6：cross_body_wrist_flip

视频：[`targets/target_06_cross_body_wrist_flip_threeway.mp4`](targets/target_06_cross_body_wrist_flip_threeway.mp4)

单组视频：[Pinocchio 原几何](targets/pinocchio_original/target_06_cross_body_wrist_flip_pinocchio_original.mp4) ｜ [Pinocchio 球体几何](targets/pinocchio_spheres/target_06_cross_body_wrist_flip_pinocchio_spheres.mp4) ｜ [NextGen 球体几何](targets/nextgen_spheres/target_06_cross_body_wrist_flip_nextgen_spheres.mp4)

| 组别 | 分类 | 终端位置 mm | 终端姿态 rad | 周期 s | 闭合 mm/rad | 同相抖动 RMS mm/rad | 4 s 游走 mm/rad |
|---|---|---:|---:|---:|---:|---:|---:|
| Pinocchio 原几何 | POSE_PERIODIC | 296.2 | 1.422 | 0.50 | 120.2 / 0.286 | 89.5 / 0.294 | 250.3 / 0.708 |
| Pinocchio 球体几何 | POSE_PERIODIC | 134.9 | 0.573 | 0.50 | 19.9 / 0.118 | 368.5 / 1.304 | 399.7 / 1.544 |
| NextGen 球体几何 | POSE_PERIODIC | 804.1 | 1.585 | 0.50 | 25.2 / 0.270 | 74.6 / 0.428 | 165.5 / 1.365 |
