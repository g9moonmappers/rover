# Digital tvilling — ROS 2 / Gazebo (`src/`)

Denne guiden er for **hele teamet** som jobber i MoonMapper-rover-repoet. Den forklarer hva som er implementert under `src/`, hvordan du installerer avhengigheter på ulike operativsystemer, og **alle kommandoer** for å bygge og kjøre simulering, RViz og autonom utforskning.

> **Kortversjon:** ROS 2 og Gazebo kjører best på **Ubuntu Linux**. ML-delen (`ml/`) fungerer på Windows og macOS — se [ml/README.md](../ml/README.md).

---

## Innhold

1. [Hva er en digital tvilling her?](#1-hva-er-en-digital-tvilling-her)
2. [Mappestruktur og pakker](#2-mappestruktur-og-pakker)
3. [Hva er implementert (oversikt)](#3-hva-er-implementert-oversikt)
4. [Operativsystem og ROS-versjon](#4-operativsystem-og-ros-versjon)
5. [Installasjon av avhengigheter](#5-installasjon-av-avhengigheter)
6. [Første gangs oppsett (klone → bygg → source)](#6-første-gangs-oppsett)
7. [Daglig bruk: source workspace](#7-daglig-bruk-source-workspace)
8. [Bygge workspace på nytt](#8-bygge-workspace-på-nytt)
9. [Launch-filer og kommandoer](#9-launch-filer-og-kommandoer)
10. [Verdener (Gazebo) og fysikkprofiler](#10-verdener-gazebo-og-fysikkprofiler)
11. [Dataflyt i autonom stack](#11-dataflyt-i-autonom-stack)
12. [Feilsøking](#12-feilsøking)
13. [Ordliste (ROS 2 for nybegynnere)](#13-ordliste-ros-2-for-nybegynnere)

---

## 1. Hva er en digital tvilling her?

En **digital tvilling** er en programmodell av den fysiske roveren som oppfører seg likt nok til utvikling og testing uten hardware.

I dette repoet betyr det:


| Komponent   | Teknologi                           | Formål                                       |
| ----------- | ----------------------------------- | -------------------------------------------- |
| Robotmodell | URDF/Xacro + STL-meshes             | Geometri, ledd, sensorer                     |
| Simulering  | **Gazebo Sim 8** (`gz sim`)         | Tyngdekraft, friksjon, kameralink            |
| Styring     | **ros2_control**                    | Hjulmotorer, diff-drive                      |
| Kartlegging | **RTAB-Map**                        | 3D/2D-kart fra dybdekamera                   |
| Navigasjon  | **Nav2**                            | Planlegging og kjøring til mål               |
| Utforskning | **Frontier explorer** (egen node)   | Velger ukjente områder på kartet             |
| Sikkerhet   | `depth_to_scan` + `safety_obstacle` | Laser-scan fra dybde + nødbrems på `cmd_vel` |


Alt under `src/` er et **ROS 2 workspace** (colcon). Du må **bygge** (`colcon build`) og **source** (`install/setup.bash`) før `ros2 launch` finner pakkene.

---

## 2. Mappestruktur og pakker

```
src/
├── moonmapper_description/   # URDF, Gazebo-verdener, mesher, rocker-bogie-plugin
├── moonmapper_bringup/       # Enkel sim-inngang (sim_rover_clean)
├── moonmapper_autonomy/      # depth→scan, safety på cmd_vel
├── moonmapper_localization/  # (valgfri) fake UWB + EKF (robot_localization) i sim
└── moonmapper_nav2/          # RTAB-Map, Nav2, frontier explorer, launch-filer
```


| Pakke                    | Type            | Hovedansvar                                                                       |
| ------------------------ | --------------- | --------------------------------------------------------------------------------- |
| `moonmapper_description` | C++ + data      | Rover-URDF, `earth_arena*.sdf` / `moon_arena.sdf`, Gazebo-plugin for rocker-bogie |
| `moonmapper_bringup`     | Python          | Wrapper som starter `gazebo_rover.launch.py` med fornuftige defaults              |
| `moonmapper_autonomy`    | Python          | `depth_to_scan_node`, `safety_obstacle_node`                                      |
| `moonmapper_localization`| Python + launch | (Sim) fake UWB + `robot_localization` EKF → `/odometry/filtered`                 |
| `moonmapper_nav2`        | Python + launch | SLAM, Nav2, frontier, alle autonomi-launch-filer                                  |


**UWB/EKF (sim):** `moonmapper_localization` er nå inkludert i denne workspacen. Den kan startes separat, eller via `moonmapper_nav2` med `use_sim_uwb:=true` (se §9.5).

---

## 3. Hva er implementert (oversikt)

### 3.1 `moonmapper_description`

- **URDF/Xacro:** `moonmapper_rover.urdf.xacro` (visning), `moonmapper_rover_gazebo.urdf.xacro` (sim med Gazebo-plugins).
- **Verdener:** `moon_arena.sdf`, `earth_arena.sdf`, `earth_arena_explore.sdf`, `earth_arena_expo_20x20.sdf` (standard autonomi).
- **ros_gz_bridge:** Kobler Gazebo-topics til ROS 2 (kamera, lidar, clock, …).
- **ros2_control:** `diff_drive_controller`, `joint_state_broadcaster`.
- **Gazebo-plugin:** `RockerBogieDifferential` — synkroniserer rocker-bogie-hjul i sim.
- **Fysikkprofiler:** `config/physics_profiles.yaml` (f.eks. `earth_stable_6wd`, `safe_6wd`).

### 3.2 `moonmapper_bringup`

- `**sim_rover_clean.launch.py`:** Anbefalt inngang for «kun sim + RViz».
- `**cmd_vel_odom_relay.py`:** Relé mellom kommandoer og odometri-topic der det trengs.

### 3.3 `moonmapper_autonomy`

- `**depth_to_scan_node`:** Konverterer dybdebilde til `/scan` (2D laser-lignende) for Nav2 og sikkerhet.
- `**safety_obstacle_node`:** Leser `/scan`, filtrerer `/cmd_vel_raw` → `/cmd_vel` (hindringsstopp).

### 3.4 `moonmapper_nav2`


| Fil / node                              | Funksjon                                       |
| --------------------------------------- | ---------------------------------------------- |
| `rtabmap_sim.launch.py`                 | Starter RTAB-Map mot sim-kamera                |
| `nav2_rtabmap_navigation.launch.py`     | Nav2 uten map_server/AMCL (kart fra RTAB)      |
| `autonomous_exploration_full.launch.py` | **Alt-i-ett:** Gazebo + SLAM + Nav2 + frontier |
| `frontier_explorer`                     | Finner grenser på kartet og sender Nav2-mål    |
| `nav2_map_ready_wait_node`              | Vent til `/map` er klar før Nav2               |
| `config/nav2_params_rtabmap_sim.yaml`   | Nav2-parametre for sim                         |
| `config/frontier_explorer.yaml`         | Utforsker-parametre                            |


**Tidssekvens (standard autonom launch):**

1. Gazebo + rover spawn (med en gang)
2. RTAB-Map etter ~6 s (`sim_stabilization_before_slam_sec`)
3. Nav2 etter ~12 s (`navigation_stack_delay`)
4. Frontier explorer etter ~20 s (12 + 8 s `explorer_extra_delay_sec`)

---

## 4. Operativsystem og ROS-versjon


| OS                             | Anbefalt ROS 2 | Støtte for `src/` sim                                                                                         |
| ------------------------------ | -------------- | ------------------------------------------------------------------------------------------------------------- |
| **Ubuntu 24.04** (PC)          | **Jazzy**      | ✅ Full — anbefalt for team-PC                                                                                 |
| **Ubuntu 22.04** (Jetson m.m.) | **Humble**     | ⚠️ Mulig, men pakkene i repoet er testet mot Jazzy; bytt pakkenavn `ros-jazzy-`* → `ros-humble-*` ved install |
| **Windows**                    | —              | ❌ Ikke innebygd. Bruk **WSL2 + Ubuntu 24.04** for ROS, eller kun ML på Windows                                |
| **macOS**                      | —              | ❌ Ikke offisielt for Gazebo Sim 8-stack. Bruk Linux VM/WSL eller fokuser på ML                                |


**Jetson / ekte robot:** Se også rot-`README.md` (hardware, Humble, RealSense). Sim-stack i `src/` er primært for **utviklings-PC**.

---

## 5. Installasjon av avhengigheter

### 5.1 Ubuntu 24.04 (anbefalt — automatisk skript)

Fra **rot** av repoet (mappen som inneholder `src/` og `ml/`):

```bash
cd /sti/til/rover   # f.eks. ~/mlOGdt
sudo bash scripts/install_ros_deps.sh
```

Skriptet installerer bl.a.: `ros-jazzy-desktop`, `ros-jazzy-ros-gz`, Nav2, RTAB-Map, ros2_control, colcon, Gazebo Sim 8 dev-pakker, og kjører `rosdep` på `src/`.

### 5.2 Ubuntu 24.04 (manuelt, kort)

```bash
# ROS 2 Jazzy — følg også: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-nav2-bringup \
  ros-jazzy-rtabmap-ros ros-jazzy-xacro ros-jazzy-teleop-twist-keyboard \
  python3-colcon-common-extensions python3-rosdep \
  libgz-sim8-dev libgz-plugin2-dev
sudo rosdep init   # kun første gang, ignorer feil hvis allerede gjort
rosdep update
```

### 5.3 Ubuntu 22.04 (Humble — Jetson)

Bytt `jazzy` med `humble` i alle pakkenavn og bruk [Humble install-guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Source deretter:

```bash
source /opt/ros/humble/setup.bash
```

Tilpass `scripts/source_workspace.sh` lokalt om du bruker Humble (eller lag eget alias).

### 5.4 Windows (WSL2)

1. Installer [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install) med **Ubuntu 24.04**.
2. Klone repoet inn i WSL-filsystemet (`~/rover`, ikke `/mnt/c/...` — mye raskere).
3. Følg §5.1 i WSL-terminalen.
4. **GUI (Gazebo/RViz):** krever WSLg (Windows 11) eller X-server — test med `gz sim -v 4` og `rviz2`.

### 5.5 macOS

ROS 2 + Gazebo Sim 8-stack er **ikke** dokumentert her. Teammedlemmer på Mac kan:

- Jobbe med **ML** ([ml/README.md](../ml/README.md)) uten ROS, eller
- Bruke en **Linux-VM** / fjern-PC for sim.

---

## 6. Første gangs oppsett

```bash
# 1. Klon repo (hvis ikke gjort)
git clone https://github.com/g9moonmappers/rover.git
cd rover    # eller mlOGdt — samme struktur

# 2. Installer ROS-avhengigheter (Ubuntu, én gang)
sudo bash scripts/install_ros_deps.sh

# 3. Bygg workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 4. Verifiser
source scripts/source_workspace.sh
ros2 pkg list | grep moonmapper
```

Forventet output: fire pakker — `moonmapper_autonomy`, `moonmapper_bringup`, `moonmapper_description`, `moonmapper_nav2`.

---

## 7. Daglig bruk: source workspace

**Åpne alltid en terminal og kjør dette før `ros2`-kommandoer:**

```bash
cd /sti/til/rover
source scripts/source_workspace.sh
```

Dette laster ROS 2 Jazzy **og** ditt lokale `install/`. Uten dette får du `ros2: command not found` eller «package not found».

---

## 8. Bygge workspace på nytt

Kjør etter endringer i `src/`:

```bash
cd /sti/til/rover
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source scripts/source_workspace.sh
```

Kun én pakke (raskere):

```bash
colcon build --packages-select moonmapper_nav2 --symlink-install
```

---

## 9. Launch-filer og kommandoer

Alle kommandoer forutsetter §7 (`source scripts/source_workspace.sh`).

### 9.1 Oversikt launch-filer


| Launch-fil                              | Pakke                    | Hva den gjør                                |
| --------------------------------------- | ------------------------ | ------------------------------------------- |
| `sim_rover_clean.launch.py`             | `moonmapper_bringup`     | Gazebo + rover + kontrollere + valgfri RViz |
| `gazebo_rover.launch.py`                | `moonmapper_description` | Full Gazebo-launch (brukes også indirekte)  |
| `rtabmap_sim.launch.py`                 | `moonmapper_nav2`        | Kun RTAB-Map SLAM                           |
| `nav2_rtabmap_navigation.launch.py`     | `moonmapper_nav2`        | Nav2 + safety (forventer kart/TF fra RTAB)  |
| `autonomous_exploration_full.launch.py` | `moonmapper_nav2`        | **Hele stacken** inkl. utforsker            |


### 9.2 Gazebo + RViz (manuell kjøring, tastatur senere)

```bash
ros2 launch moonmapper_bringup sim_rover_clean.launch.py \
  use_sim_time:=true \
  use_rviz:=true
```

**Verdener:** `world_preset:=expo_20x20` (standard) | `moon` | `earth` | `earth_explore`

Teleop (hvis `teleop_twist_keyboard` er installert):

```bash
# Ny terminal, source workspace først
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=false
```

### 9.3 Kun RTAB-Map (Gazebo må kjøre i annen terminal)

```bash
ros2 launch moonmapper_nav2 rtabmap_sim.launch.py \
  use_sim_time:=true \
  rtabmap_rviz:=true
```

Valgfrie argumenter: `database_path:=/full/sti/moonmapper.db`, `rgb_topic`, `depth_topic`, `camera_info_topic`.

### 9.4 Nav2 + RTAB (uten frontier, uten Gazebo)

```bash
ros2 launch moonmapper_nav2 nav2_rtabmap_navigation.launch.py \
  use_sim_time:=true \
  rviz:=true
```

Sim-default topics for dybde er satt i launch; på ekte robot bruk RealSense-topics (se launch-filens `depth_image_topic` / `camera_info_topic`).

### 9.5 Full autonom utforskning (anbefalt demo)

```bash
ros2 launch moonmapper_nav2 autonomous_exploration_full.launch.py \
  use_sim_time:=true \
  start_sim:=true \
  start_slam:=true \
  start_nav2:=true \
  start_explorer:=true \
  start_rviz:=true \
  start_rtabmap_viz:=false \
  navigation_stack_delay:=12.0 \
  explorer_extra_delay_sec:=5.0
```

**Merk (expo standard):** Når `world_preset:=expo_20x20` brukes, lastes expo-tuning automatisk:

- `config/frontier_explorer_expo.yaml` (frontier-basert utforsking, lavere BFS-terskel)
- `config/nav2_params_expo_overlay.yaml` (Nav2/MPPI justert for å faktisk kjøre fremover i arenaen)

**Med UWB/EKF (sim):** Starter sim-UWB + EKF → `/odometry/filtered`.

```bash
ros2 launch moonmapper_nav2 autonomous_exploration_full.launch.py \
  use_sim_time:=true \
  use_sim_uwb:=true \
  start_sim:=true start_slam:=true start_nav2:=true start_explorer:=true \
  start_rviz:=true
```

| Argument                            | Standard        | Betydning                                      |
| ----------------------------------- | --------------- | ---------------------------------------------- |
| `start_sim`                         | `true`          | Start Gazebo via `sim_rover_clean`             |
| `start_slam`                        | `true`          | RTAB-Map etter stabiliseringstid               |
| `start_nav2`                        | `true`          | Nav2-stack                                     |
| `start_explorer`                    | `true`          | Frontier explorer                              |
| `start_rviz`                        | `false`         | Nav2-RViz (`true` anbefales for visning)       |
| `start_rtabmap_viz`                 | `false`         | Eget RTAB-Map-RViz-vindu                       |
| `use_sim_uwb`                       | `false`         | Start `moonmapper_localization` (fake UWB + EKF) |
| `navigation_stack_delay`            | `12.0`          | Sekunder før Nav2                              |
| `explorer_extra_delay_sec`          | `8.0`           | Ekstra sekunder etter Nav2 før explorer        |
| `sim_stabilization_before_slam_sec` | `6.0`           | Sekunder før SLAM                              |
| `delete_rtabmap_db`                 | `false`         | `true` = slett gammel kartdatabase før kjøring |
| `initial_spin`                      | `true`          | Liten rotasjon ved oppstart for SLAM           |
| `world_preset`                      | `expo_20x20`    | Se §10 (standard autonomi)                     |
| `force_set_pose_after_spawn`        | `false`         | Tving Gazebo set_pose (feilsøking)             |


**Kun sim + kartlegging (uten Nav2/utforsker):**

```bash
ros2 launch moonmapper_nav2 autonomous_exploration_full.launch.py \
  start_sim:=true start_slam:=true start_nav2:=false start_explorer:=false \
  start_rviz:=true
```

### 9.6 Enkeltstående noder (feilsøking)

```bash
ros2 run moonmapper_autonomy depth_to_scan_node
ros2 run moonmapper_autonomy safety_obstacle_node
ros2 run moonmapper_nav2 frontier_explorer
ros2 run moonmapper_nav2 nav2_map_ready_wait_node
```

### 9.7 Nyttige `ros2`-kommandoer

```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 node list
ros2 pkg executables moonmapper_nav2
```

---

## 10. Verdener (Gazebo) og fysikkprofiler


| `world_preset`  | SDF-fil                        | Bruk                                              |
| --------------- | ------------------------------ | ------------------------------------------------- |
| `expo_20x20`    | `earth_arena_expo_20x20.sdf`   | **Standard autonomi** — 20×20 expo-arena, spawn SW |
| `moon`          | `moon_arena.sdf`               | Måne-lignende arena                               |
| `earth`         | `earth_arena.sdf`              | Jord-arena                                        |
| `earth_explore` | `earth_arena_explore.sdf`      | Eldre utforskningsarena                           |

Alias: `world_preset:=expo` er det samme som `expo_20x20`.


**Fysikk:** `physics_profile` kan settes i `gazebo_rover` / `sim_rover_clean` (f.eks. `earth_stable_6wd`). Tom verdi = preset fra `world_preset`.

---

## 11. Dataflyt i autonom stack

```
Gazebo (kamera, odom)
    → RTAB-Map → /rtabmap/map → relay → /map
    → map→odom TF
depth_camera → depth_to_scan → /scan
Nav2 → /cmd_vel_raw → safety_obstacle → /cmd_vel → diff_drive
frontier_explorer → /navigate_to_pose (Nav2 action)
```

**Viktige topics:**


| Topic          | Innhold                             |
| -------------- | ----------------------------------- |
| `/cmd_vel_raw` | Nav2 og explorer                    |
| `/cmd_vel`     | Etter sikkerhetsnode, til motorer   |
| `/map`         | Occupancy grid for Nav2 og explorer |
| `/scan`        | Syntetisk laser fra dybde           |

**Merk:** `frontier_explorer` styrer *ikke* kjøring direkte (utenom initial spin ved oppstart). Den sender mål til Nav2 via `NavigateToPose`.


RTAB-database (standard): `~/.ros/moonmapper_rtabmap.db`

---

## 12. Feilsøking


| Symptom                                           | Løsning                                                                                         |
| ------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `ros2: command not found`                         | `source /opt/ros/jazzy/setup.bash`                                                              |
| `package 'moonmapper_*' not found`                | `colcon build` + `source scripts/source_workspace.sh`                                           |
| `install/setup.bash: No such file`                | Kjør `colcon build` fra repo-rot                                                                |
| `executable 'nav2_map_ready_wait_node' not found` | `chmod +x src/moonmapper_nav2/scripts/`* og `colcon build --packages-select moonmapper_nav2`    |
| `view_robot.launch.py` not found                  | Finnes **ikke** i dette repoet (arkivert i gammelt workspace). Bruk `sim_rover_clean.launch.py` |
| Gazebo svart vindu / krasj                        | Prøv `gz_sim_verbosity:=1`, sjekk GPU/WSLg                                                      |
| Nav2 starter ikke                                 | Vent `navigation_stack_delay`; sjekk at `/map` publiseres (`ros2 topic hz /map`)                |
| Rover kjører ikke                                 | Sjekk `ros2 topic echo /cmd_vel` og at kontrollere er lastet (se Gazebo-terminal)               |
| Utforsker står og venter                           | Sjekk `/frontier_explorer/status` og at Nav2 action `/navigate_to_pose` finnes                  |


**Byggefeil Gazebo-plugin:** Filen `RockerBogieDifferential.cc` skal **ikke** ha Python-trippel-anførselstegn `"""` øverst — kun C++-kommentarer `//`.

---

## 13. Ordliste (ROS 2 for nybegynnere)


| Begrep                        | Forklaring                                                |
| ----------------------------- | --------------------------------------------------------- |
| **Workspace**                 | Mappe med `src/`, `build/`, `install/` — ditt prosjekt    |
| **Package**                   | En ROS-modul (`moonmapper_nav2`, …)                       |
| **Node**                      | Et kjørende program i ROS                                 |
| **Topic**                     | Kanal for meldinger (f.eks. `/cmd_vel`)                   |
| **Launch**                    | Starter flere noder med én kommando                       |
| **URDF/Xacro**                | Robotbeskrivelse (ledd, sensorer)                         |
| **TF**                        | Koordinatsystem-kjede (`map` → `odom` → `base_footprint`) |
| **colcon build**              | Kompilerer/installerer alle pakker i `src/`               |
| **source install/setup.bash** | Gjør pakkene synlige for `ros2`                           |


---

## Relatert dokumentasjon

- [ml/README.md](../ml/README.md) — spektralsensor, datasett, trening, prediksjon
- [README.md](../README.md) — hardware og eldre Jetson-oppsett
- Rot: `scripts/install_ros_deps.sh`, `scripts/source_workspace.sh`

**Repo:** [https://github.com/g9moonmappers/rover](https://github.com/g9moonmappers/rover)