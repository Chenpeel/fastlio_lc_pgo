# fastlio_lc_pgo

ROS 2 port of the Scan Context + GTSAM pose-graph loop-closure back end from
[yanliang-wang/FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC),
wired to consume FAST-LIO2 / Point-LIO odometry and registered clouds. Runs
loop closure on top of a LIO front end to correct drift and produce a saved
map for later localization.

## What it does

`pgo_node` subscribes to the LIO odometry and registered-cloud topics, builds
a keyframe pose graph, detects loop closures (ICP + optional Scan Context),
and optimizes the graph with GTSAM/iSAM2. It publishes:

- `/aft_pgo_odom`, `/aft_pgo_path` -- corrected odometry/path
- `/aft_pgo_map` -- accumulated, loop-closure-corrected map cloud
- `/loop_closure_constraints` -- detected loop edges (RViz markers)

## 本分支保存与关键帧约束

- 每次采集使用新的 `save_directory`, 或显式准备的空目录. 非空目录会拒绝启动, 不再清空旧 `Scans/` 或覆盖旧会话.
- 采集中实时保存 `Scans/`, `times.txt` 和优化位姿. 停车后保持 PGO 运行, 调用 `/pgo_batch_optimize` 保存 `optimized_poses_batch.txt` 与 `map_pcd_path` 指定的地图.
- 使用 `ros2 service call /pgo_batch_optimize std_srvs/srv/Trigger '{}'`, 核对返回状态与实际文件.
- 关键帧位移和转角相对上一接受关键帧计算, 避免静止抖动累计成虚假运动; 发布路径包含最新已优化关键帧.
- 独立 FAST-LIO 管线未提供 `level_frame` 变换时, 显式配置 `save_in_level_frame:=false`; 不混用已转平地图与未转平的优化位姿重建结果.
- 保留上游的平面先验, Scan Context 开关和新版本启动文件; 迁移调用方须显式确认这些参数, 不假定旧版默认值.


## Launch files

- **fastlio_lc_l2.launch.py** -- full mapping bringup on the Unitree L2 with
  a FAST-LIO front end: sensor TF, FAST-LIO odometry, `pgo_node`, the
  `pgo_map_odom_bridge` (owns `map -> odom`), a self-hit cloud filter, and
  `octomap_server` for a ray-traced 2D `/projected_map`.
- **pointlio_lc_l2.launch.py** -- the same stack with a Point-LIO front end
  instead of FAST-LIO.

```
ros2 launch fastlio_lc_pgo fastlio_lc_l2.launch.py
# or
ros2 launch fastlio_lc_pgo pointlio_lc_l2.launch.py
```

Both default to `use_sim_time:=false` (live robot); pass `true` and
`ros2 bag play <bag> --clock` for replay. Pass `occupancy:=false` to skip
`octomap_server` if you only want the pose-graph map.

The finished map (`map_pcd_path`, default under `pepper_navigation/pcd/`) is
what `fast_lio`'s `localization_l2.launch.py` localizes against.

## Notable parameters

| Param | Purpose |
|---|---|
| `save_directory` / `map_pcd_path` | where PGO writes its output |
| `keyframe_filter_size` | voxel size applied before a keyframe is stored (bounds every downstream map's density) |
| `planar_prior` | pins keyframe height to the floor plane -- turn off only if the robot changes level (ramp/lift) |
| `use_scan_context` | Scan Context loop candidates, off by default (radius search covers most indoor revisits) |
| `historyKeyframeSearchRadius`, `loopFitnessScoreThreshold`, `loopNoiseScore` | loop-closure acceptance gates |
| `occupancy`, `occ_min_z`/`occ_max_z` | enable/tune the octomap 2D projection |

See the launch files themselves for full descriptions and defaults.
