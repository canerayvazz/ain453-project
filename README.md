# Particle Filter Localization (Duckiebot)

Monte Carlo localization in a 5 m × 4 m room with eight identical-ID wall markers. Multi-hypothesis sensor model: likelihood sums over all eight tag positions (no nearest-tag shortcut).

## Repository layout

```text
project/
├── ros2_ws/          Simulation — ROS 2 Humble + Gazebo Classic
├── real/             Bonus — physical Duckiebot (ROS 1 Noetic + Duckietown dts)
└── README.md
```

## Prerequisites

### Simulation

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- `ros-humble-gazebo-ros-pkgs`, `ros-humble-robot-state-publisher`, `ros-humble-rviz2`, `ros-humble-teleop-twist-keyboard`, `ros-humble-cv-bridge`, `python3-opencv`

### Real robot (bonus)

- Duckietown shell (`dts`) with profile `daffy`
- Physical Duckiebot on the same network (example name: `wolf`)
- AprilTag **tag36h11** markers (0.2 m), same wall layout as simulation

## Simulation (ROS 2)

```bash
cd ros2_ws
colcon build --packages-select pf_localization
source install/setup.bash
```

**Terminal 1 — Gazebo, particle filter, RViz:**

```bash
ros2 launch pf_localization bringup.launch.py
```

**Terminal 2 — drive:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Markers in simulation use ArUco `DICT_6X6_250` (ID 0). Parameters: `ros2_ws/src/pf_localization/config/pf_params.yaml`.

Regenerate the Gazebo ArUco texture if needed:

```bash
python3 ros2_ws/src/pf_localization/generate_marker.py
```

## Real robot (ROS 1 bonus)

Code lives in `real/`. Does not modify `ros2_ws`.

**One-time build on the robot:**

```bash
cd real
dts devel build -H wolf -f
```

**Terminal 1 — particle filter on wolf:**

```bash
cd real
dts devel run -H wolf
```

**Terminal 2 — keyboard teleop:**

```bash
dts duckiebot keyboard_control wolf
```

**Terminal 3 — RViz on laptop** (only one `dts devel run` allowed; use gui-tools for visualization):

```bash
cd real
dts start_gui_tools --mount "$(pwd)":/real --wkdir /real wolf
bash launchers/rviz.sh
```

Inside the container you land in `/real`. If you used `--mount "$(pwd)"` without `:/real`, run
`bash /home/caner/project/real/launchers/rviz.sh` (use your laptop path) or `cd` there first.

Set RViz **Fixed Frame** to `odom` if the view is empty. Camera-only check:

```bash
rqt_image_view /camera/image_annotated
```

Real robot uses AprilTag `tag36h11` (6.5 cm markers) on a **0.9 m × 0.9 m** lab grid; tag centers are in `real/packages/pf_localization_real/config/pf_params.yaml`. Physical tags may have **different IDs** — the filter still sums likelihoods over all eight landmark positions (same multi-hypothesis logic as simulation). Odometry pose comes from `/{veh}/velocity_to_pose_node/pose` (integrated wheel velocity). Change `wolf` to your robot name in launchers and commands.

**Laptop amd64 image** (only if you run `dts devel run` locally without `-H`):

```bash
cd real
dts devel build -f
```

## Parameters

| File | Use |
|------|-----|
| `ros2_ws/src/pf_localization/config/pf_params.yaml` | Simulation |
| `real/packages/pf_localization_real/config/pf_params.yaml` | Physical robot |

Simulation map: 5×4 m room with eight asymmetric tag centers. Lab map: 0.9×0.9 m grid (origin bottom-left). Both use `num_particles: 500` and the same multi-hypothesis sensor model.

## Visualization topics

Both stacks publish:

- `/particle_markers` — weight-colored particles
- `/tag_markers` — room outline and T1–T8
- `/odom_path` — red (odometry)
- `/pf_path` — green (filter estimate)
- `/camera/image_annotated` — detections overlay

Real robot also publishes `/robot_markers`:

- **Cyan** — highest-weight particle (best hypothesis)
- **Light green** — weighted-mean PF estimate
