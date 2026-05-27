# moonmapper_localization

Sim-only fake UWB (BU04) + `robot_localization` EKF for MoonMapper.

## Arkitektur (sim)

- **sim_uwb_node**: simulerer BU04 basert på Gazebo ground truth (primært) med odom fallback → `/uwb/pose` (base-korrigert), `/uwb/tag_pose`, `/uwb/ranges`, `/uwb/status`
- **ekf_node**: `/diff_drive_controller/odom` + `/imu/data` + `/uwb/pose` → `/odometry/filtered` og TF `odom` → `base_footprint`

RTAB-Map kan fortsatt eie `map` → `odom` (SLAM), mens Nav2/RTAB bruker fused odom som odometri-input.

## Launch

```bash
# Sim: fake UWB + EKF
ros2 launch moonmapper_localization localization_fusion.launch.py use_sim_time:=true

# Full stack (valgfritt, default av i autonomous_exploration_full)
ros2 launch moonmapper_nav2 autonomous_exploration_full.launch.py \
  use_sim_uwb:=true use_sim_time:=true
```

## Test

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /odometry/filtered --once
ros2 topic echo /uwb/status --once
```

## Parametre

Se `config/uwb_anchors.yaml` og `config/ekf_sim_uwb.yaml`.
