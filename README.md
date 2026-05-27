# MoonMapper Rover

Programvare for bachelorgruppe 9 (MoonMappers) ved USN Kongsberg — digital tvilling (sim), autonom navigasjon og spektral materialgjenkjenning.

## Dokumentasjon for teamet (start her)

| Del | Guide | Innhold |
|-----|--------|---------|
| **ROS 2 / Gazebo / autonomi** | [src/digital_tvilling.md](src/digital_tvilling.md) | Pakker, installasjon (Linux/WSL), alle launch-filer, kommandoer, feilsøking |
| **Maskinlæring / Triad** | [ml/README.md](ml/README.md) | Datasett, Arduino, trening, prediksjon — Windows/macOS/Linux |

## Repo-struktur

```
├── src/              # ROS 2 workspace (4 pakker) — se digital_tvilling.md
├── ml/               # Triad ML-pipeline — se ml/README.md
├── arduino/          # Triad logger firmware
├── scripts/          # source_workspace.sh, activate_ml.sh, install_ros_deps.sh
├── requirements-ml.txt
├── setup/            # Eldre oppstart/feilsøking (fysisk robot)
├── code/             # Eldre motor/EKF-notater
└── trilateration/    # Eldre UWB-kode (arkiv)
```

## Hurtigstart

**Sim + autonom utforskning (Ubuntu 24.04, ROS 2 Jazzy):**

```bash
git clone https://github.com/g9moonmappers/rover.git && cd rover
sudo bash scripts/install_ros_deps.sh    # én gang
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
source scripts/source_workspace.sh
ros2 launch moonmapper_nav2 autonomous_exploration_full.launch.py \
  use_sim_time:=true \
  start_sim:=true start_slam:=true start_nav2:=true start_explorer:=true \
  start_rviz:=true navigation_stack_delay:=12.0 explorer_extra_delay_sec:=5.0
```

**ML (alle OS med Python 3.10+):**

```bash
source scripts/activate_ml.sh
python ml/training/extract_triad_features.py
python ml/training/train_random_forest.py
```

## Hardware (fysisk rover)

- **Compute:** NVIDIA Jetson Orin Nano Super
- **Kamera:** Intel RealSense D435
- **Motorer:** 6× Dynamixel XC430-W150-T
- **IMU:** MPU6050 (I2C bus 7)
- **Spektral:** 2× Triad AS7265X (via Arduino — se `ml/README.md`)

Eldre Jetson-oppsett (ROS 2 Humble, RealSense): se [setup/startup.md](setup/startup.md) og [setup/troubleshooting.md](setup/troubleshooting.md).

## Programvareoversikt

| Område | Teknologi |
|--------|-----------|
| Sim | Gazebo Sim 8, ros_gz, ros2_control |
| SLAM | RTAB-Map |
| Navigasjon | Nav2 |
| Utforskning | Frontier explorer (egen node) |
| ML | scikit-learn Random Forest, Triad 36-kanals spektrum |

## Lisens

Apache-2.0 (ROS-pakker). Se `package.xml` per pakke.
