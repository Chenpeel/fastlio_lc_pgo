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

Calling `ros2 service call /pgo_batch_optimize std_srvs/srv/Trigger` saves
`map_pcd_path` plus `optimized_poses.txt`/`times.txt` under `save_directory`
-- nothing is written to disk until you call it.

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
