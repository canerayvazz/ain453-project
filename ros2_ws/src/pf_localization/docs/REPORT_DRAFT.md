# Particle Filter Localization with AR Tags: Implementation Log (Phases 1–12)

**Package:** `pf_localization`  
**Platform:** Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11  
**Purpose of this file:** Internal reminder of what was built, in order — **not** the final graded report. Use it when writing the formal PDF and preparing the presentation.

---

## Implementation Status (*Project.pdf*)

| Area | Status |
|------|--------|
| §2 Simulation (Gazebo, room, 8 tags, Duckiebot, camera, teleop) | ✅ Done |
| §3.1 Particle filter \((x,y,\theta)\), unknown initial pose option | ✅ Done |
| §3.2 Multi-hypothesis sensor (sum over 8 tags, no nearest-tag shortcut) | ✅ Done |
| §3.3 Real-time visualization (room, tags, weighted particles, dual paths) | ✅ Done |
| YAML parameterization (no hardcoded map/noise in Python) | ✅ Done (Phase 9) |
| §4 Written report (Bayesian discussion, screenshots, GitHub link) | ⏳ You prepare separately |
| §5 Presentation + experiment videos | ⏳ You prepare separately |
| §7 Bonus — real Duckiebot | ✅ Done (Phase 12, separate `real/` tree) |

**Bottom line:** Simulation and core code are **complete**. Bonus sim-to-real port lives in `~/project/real/` (ROS 1 Noetic, untouched `ros2_ws`). Remaining deliverables: formal report PDF, presentation, experiment videos, GitHub link, submission zip.

---

## How to Use This Document

| Part | Sections | Purpose |
|------|----------|---------|
| **Narrative (what we did)** | §1–§2 | Step-by-step history — **presentation outline** (*Project.pdf* §5). |
| **Technical reference (simulation)** | §3–§10 | Equations, parameters, topics — **source for formal report** (*Project.pdf* §4). |
| **Bonus (real Duckiebot)** | §12 | ROS 1 port, `dts` deployment, AprilTag `tag36h11`. |

---

## 1. Step-by-Step Implementation Timeline (What We Did)

Work proceeded in twelve phases (1–11 simulation, 12 bonus sim-to-real). Each phase added one runnable milestone before moving on.

### Phase 1 — ROS 2 workspace and package skeleton

**Goal:** Create a buildable `ament_python` package for all project code.

**Steps performed:**

1. Created `ros2_ws` and package `pf_localization` with `ros2 pkg create --build-type ament_python`.
2. Declared dependencies: `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`, `gazebo_ros_pkgs`, launch/URDF stack, `rviz2`.
3. Registered the future estimator as executable `pf_node` in `setup.py`.
4. Installed `colcon` and wired workspace overlay sourcing (`source install/setup.bash`).

**Outcome:** Empty package builds; ready for robot and simulation assets.

---

### Phase 2 — Duckiebot model for Gazebo Classic (ROS 2)

**Goal:** Spawn the Duckiebot with wheel odometry and a forward camera in simulation.

**Steps performed:**

1. Imported `duckiebot_description` meshes and XACRO from Duckietown simulation sources into `urdf/` and `meshes/`.
2. Rewrote all `package://` URIs to `package://pf_localization/...`.
3. Migrated Gazebo plugins to ROS 2 Humble style:
   - `libgazebo_ros_diff_drive.so` → `/cmd_vel`, `/odom`, wheel separation 0.14 m, diameter 0.064 m;
   - `libgazebo_ros_camera.so` → namespace `/camera`, image on `/camera/camera_sensor/image_raw`.

**Outcome:** Valid URDF/XACRO; robot model ready to spawn (details in §3.2).

---

### Phase 3 — Room world and landmark placeholders

**Goal:** Closed 5 m × 4 m room with eight wall landmarks at known map coordinates.

**Steps performed:**

1. Authored `worlds/room_tags.world` (SDF 1.6): floor, four walls, interior bounds \(x \in [0,5]\), \(y \in [0,4]\).
2. Placed **eight 0.2 m × 0.2 m markers** at \(z = 0.2\) m on walls (two per wall, **asymmetric** positions per project spec).
3. **Deliberate simplification:** markers were **distinctively colored boxes** (not ArUco textures yet) so we could validate geometry, poses, and wall orientations quickly without fighting Gazebo material paths.

**Outcome:** Gazebo world matched `tag_x` / `tag_y` in `config/pf_params.yaml`; landmarks visible but **not** decodable by OpenCV ArUco.

**Design note:** Colored placeholders were an **intermediate step**, not the final sensor setup. They let us test spawn pose and room layout before Phase 8 textures.

---

### Phase 4 — Bring-up launch (Gazebo + robot + RViz)

**Goal:** One command starts simulation and visualization.

**Steps performed:**

1. Wrote `launch/bringup.launch.py`: Gazebo + `room_tags.world`, `robot_state_publisher`, delayed spawn, RViz2.
2. Spawn at \((2.5,\,2.0,\,0.05)\) with `-package_to_model` and **8 s** spawn delay.
3. Set `GAZEBO_MODEL_PATH` so `model://pf_localization/meshes/...` resolves.

**Issues encountered and fixed:**

| Symptom | Cause | Fix |
|---------|--------|-----|
| Robot invisible / underground | Spawn at \(z=0\), mesh timing | Spawn at \(z=0.05\); delay spawn 8 s |
| Meshes not found | Model URI resolution | `-package_to_model`; append `GAZEBO_MODEL_PATH` |

**Outcome:** Stable `ros2 launch pf_localization bringup.launch.py` with Duckiebot in the room and RViz opening (RViz layout completed in Phase 7).

---

### Phase 5 — Particle filter core (`pf_node`)

**Goal:** Implement Task 1–2 from *Project.pdf*: prediction, multi-hypothesis update, resampling — **no nearest-tag shortcut**.

**Steps performed:**

1. Implemented `pf_localization/pf_node.py` with \(N=500\) particles, **uniform prior** over room pose.
2. **Prediction:** `/odom` deltas in previous body frame + Gaussian noise (\(\sigma_{\mathrm{trans}}, \sigma_{\mathrm{rot}}\) from YAML).
3. **Update:** \(p(\mathbf{z}|\mathbf{x}) = \sum_{i=1}^{8} p(\mathbf{z}|\mathbf{x}, \mathrm{tag}_i)\) for every detection.
4. **Resampling:** low-variance (systematic) when \(N_{\mathrm{eff}} < 0.5\,N\).
5. Centralized parameters in `config/pf_params.yaml` (room bounds, all eight tag centers, noise, frame `odom`).

**Outcome:** Filter ran in simulation; without real ArUco images (Phase 3 placeholders), weights stayed near-uniform and the estimate did not localize.

---

### Phase 6 — ArUco vision pipeline (same node)

**Goal:** Turn camera images into range–bearing observations for the filter.

**Steps performed:**

1. Subscribed to `/camera/camera_sensor/image_raw` (configurable `camera_image_topic`).
2. `cv2.aruco.detectMarkers` with **`DICT_6X6_250`** (same ID for all physical tags).
3. Range from marker bounding-box width (pinhole + 0.2 m tag size); bearing from horizontal pixel offset.
4. Published `/camera/image_annotated` for debugging in RViz.

**Issues encountered and fixed:**

| Symptom | Cause | Fix |
|---------|--------|-----|
| No camera in RViz | Wrong default topic | Default `camera_image_topic` → `.../camera_sensor/image_raw` |
| Node crash on detect | Old OpenCV API | `DetectorParameters_create()` fallback |

**Outcome:** Vision pipeline ready; **still no measurements** in sim while walls used colored placeholders only.

---

### Phase 7 — Trajectories, RViz layout, teleoperation

**Goal:** Satisfy Task 3 visualization — particle cloud, red odom path, green PF path, side-by-side comparison.

**Steps performed:**

1. Record **`/odom_path`** (red) on each `/odom` message.
2. Record **`/pf_path`** (green) from weighted mean at 10 Hz.
3. Publish **`/particles`** (blue arrows in RViz) and **`/estimated_pose`**.
4. Authored `rviz/pf_loc.rviz` (grid, robot model, particles, both paths, annotated camera).
5. Manual driving via `teleop_twist_keyboard` → `/cmd_vel`.

**What we observed at this stage:**

- Red path followed the driven trajectory.
- Green path stayed near the room center or moved inconsistently; blue particles spread across the room.
- **Expected:** without ArUco detections, the filter cannot correct odometry (see §8.5).

**Outcome:** Full visualization stack matched *Project.pdf* §3.3 structurally; landmark **appearance** still blocked closing the loop.

---

### Phase 8 — Real ArUco textures in Gazebo (replace placeholders)

**Goal:** Replace colored boxes with **decodable** `DICT_6X6_250` ID 0 textures so measurement updates run end-to-end.

**Steps performed:**

1. Added `generate_marker.py` → `materials/textures/aruco_marker.png` (200×200, ID 0).
2. Added `materials/scripts/aruco.material` (`Aruco/Marker`).
3. Updated all eight `ar_tag_*` models in `room_tags.world` to use script materials (identical texture on every tag).
4. Installed materials via `setup.py`; exported `gazebo_media_path` in `package.xml`.
5. Fixed launch environment:
   - Append **`/usr/share/gazebo-11`** and package share to `GAZEBO_RESOURCE_PATH` (gzclient had crashed when only the package path was set).
   - Use **directory** URIs in SDF (`file://materials/scripts` + `file://materials/textures`), not `.../aruco.material` (had produced yellow/black hazard-stripe “missing texture”).

**Evolution summary (landmarks):**

```text
Phase 3:  colored box placeholders  →  geometry & map params OK, no ArUco decoding
Phase 8:  shared ArUco texture      →  OpenCV detections → filter updates
```

**What we observed after Phase 8:**

- Wall markers show black-and-white ArUco; **CameraAnnotated** shows green outlines when facing a tag.
- Blue particles move with driving; cluster tightens when tags are visible.
- Green and red paths often move **in parallel** before **overlapping** — consistent with identical marker IDs and multi-hypothesis ambiguity until the robot disambiguates by motion (§8.5, §7.5).

**Outcome:** End-to-end localization pipeline operational in simulation.

---

### Phase 9 — YAML parameterization

**Goal:** All project constants in `config/pf_params.yaml`; no room/tag/noise defaults hardcoded in Python (*Project.pdf* / `.cursorrules`).

**Steps performed:**

1. Expanded `pf_params.yaml`: `num_particles`, room bounds, eight \((x,y)\) tags + `tag_yaw`, `odom_noise_*`, `sensor_noise_*`, camera intrinsics, `tag_size_m`.
2. Refactored `pf_node.py`: `declare_parameter(name, type)` only; values loaded via `get_parameter()`.
3. Updated `bringup.launch.py` to start **`pf_node` with `pf_params.yaml`** in the same launch as Gazebo and RViz.
4. Confirmed `setup.py` installs `config/*.yaml` into `share/pf_localization/config/`.

**Outcome:** Tuning and report tables can reference a single YAML file; rebuild not required for parameter experiments.

---

### Phase 10 — Sensor-model fix (wrong-corner localization)

**Symptom:** Blue particles clustered at the **wrong corner** while the car was elsewhere; green/red paths parallel but offset even after long drives with many ArUco sightings.

**Root cause:** Camera bearing was compared to \(\atan2(\Delta y, \Delta x) - \theta_{\mathrm{particle}}\). Using each particle’s \(\theta\) in the expected bearing let **wrong \((x,y,\theta)\) triples** match the same camera reading, so incorrect corners could win.

**Fix:**

1. **Measured bearing in world frame:** \(\beta_{\mathrm{world}} = \psi_{\mathrm{odom}} + \beta_{\mathrm{camera}} + \texttt{camera\_yaw\_offset}\).
2. **Expected bearing from particle position only:** \(\hat{\beta}_{\mathrm{world}} = \atan2(y_i - y^{(j)},\, x_i - x^{(j)})\) — **no** \(\theta^{(j)}\) in the bearing likelihood.
3. Optional **`seed_particles_from_odom`**: Gaussian cloud on first `/odom` for faster demos (see Phase 11).

**Outcome:** Particle cloud tracks the robot; user confirmed blue arrows much better aligned with the car.

---

### Phase 11 — PDF visualization polish and unknown initial pose

**Goal:** Close remaining *Project.pdf* §3.3 gaps without changing filter math.

**Steps performed:**

1. **`/particle_markers`** (`visualization_msgs/MarkerArray`): arrows colored by weight (red = low, green = high; size scales with weight).
2. **`/tag_markers`**: room outline, eight tag cubes (T1–T8 labels), same coordinates as the filter map — room + tags visible in RViz alongside particles.
3. **`seed_particles_from_odom`** (default **`false`**): strict “unknown location” = **uniform prior** over the room; set `true` in YAML for faster debugging (particles start near first odometry).
4. Updated `rviz/pf_loc.rviz` for `ParticlesWeighted` and `ARTags` displays.

**Unknown initial pose (what *Project.pdf* means):**

| Mode | `seed_particles_from_odom` | Behavior |
|------|----------------------------|----------|
| **Strict PDF** | `false` | Filter does **not** know spawn pose; particles uniform over \(5\times4\) m room until landmarks update. |
| **Demo / debug** | `true` | Particles Gaussian around first `/odom` — easier startup, but the filter is given a strong hint of initial pose. |

**Outcome:** Task 3 visualization requirements met in RViz; default init matches “unknown location” wording in §3.1.

---

### Phase 12 — Sim-to-real bonus (ROS 1 / physical Duckiebot)

**Goal:** Port the particle filter to a physical Duckiebot using the official Duckietown ROS 1 template, **without modifying** `ros2_ws`.

**Constraint:** `ros2_ws` left intact; all bonus code under `~/project/real/`.

**Steps performed:**

1. Cloned Duckietown template: `git clone --depth 1 -b v3 https://github.com/duckietown/template-ros.git real`.
2. Created catkin package `real/packages/pf_localization_real/` (`CMakeLists.txt`, `package.xml`, `config/pf_params.yaml`, launch file).
3. Ported `pf_node.py` → `scripts/pf_node_ros1.py` (rospy): same `ParticleFilter` class, multi-hypothesis sum over 8 tags, world-frame bearing (Phase 10 fix), low-variance resampling.
4. Hardware topics (vehicle `wolf`): `/{veh}/camera_node/image/compressed`, `/{veh}/kinematics_node/pose` (`duckietown_msgs/Pose2DStamped`).
5. DTProject integration: `Dockerfile` (`REPO_NAME=real`), `launchers/default.sh`, `dependencies-py3.txt` (`opencv-contrib-python>=4.7`).
6. RViz on laptop via **`dts start_gui_tools --mount "$(pwd)" wolf`** + `bash launchers/rviz.sh` (same pattern as assignment4; avoids second `dts devel run`).
7. **Lab fiducials:** real robot uses **AprilTag `tag36h11`** (`DICT_APRILTAG_36h11`); simulation remains **ArUco `DICT_6X6_250`**.

**Issues encountered and fixed:**

| Symptom | Cause | Fix |
|---------|--------|-----|
| `RLException` on launch | Nested `$(optenv ...)` in ROS 1 launch XML | `$(optenv VEHICLE_NAME wolf)` only |
| `dts devel run -R wolf -l rviz` runs old `path_planner` | Stale amd64 Docker image from prior project | `dts devel build -f` on laptop |
| Second `dts devel run` kills first | One active devel container per project | PF on wolf (`-H`); viz via `start_gui_tools` |
| `rospack find pf_localization_real` fails in gui-tools | Generic gui-tools image has no custom package | Mount repo + `launchers/rviz.sh` with relative path to `pf_loc.rviz` |

**Runtime commands (wolf):**

```bash
# Terminal 1 — PF on robot
cd ~/project/real && dts devel build -H wolf -f && dts devel run -H wolf

# Terminal 2 — teleop
dts duckiebot keyboard_control wolf

# Terminal 3 — RViz on laptop
cd ~/project/real
dts start_gui_tools --mount "$(pwd)" wolf
bash launchers/rviz.sh
```

**Outcome:** Bonus codebase ready for lab demo; same map/noise YAML as simulation; detector family differs (`tag36h11` on hardware). End-to-end on-robot testing is operator-dependent (rebuild after code changes).

---

### Phase summary table

| Phase | Focus | Key artifacts | Runnable milestone |
|-------|--------|---------------|-------------------|
| 1 | Workspace | `pf_localization` package | `colcon build` |
| 2 | Robot | `urdf/`, `meshes/`, Gazebo plugins | URDF valid |
| 3 | World | `room_tags.world`, **colored tag placeholders** | Gazebo room + 8 markers |
| 4 | Launch | `bringup.launch.py` | One-launch sim + spawn |
| 5 | Filter math | `pf_node.py`, `pf_params.yaml` | PF predicts from odom |
| 6 | Vision | ArUco in `pf_node` | Annotated image (no sim detections yet) |
| 7 | Viz + teleop | `pf_loc.rviz`, path topics | Red/green paths in RViz |
| 8 | ArUco textures | `generate_marker.py`, `materials/` | Detections + filter updates |
| 9 | YAML params | `config/pf_params.yaml`, launch loads YAML | No hardcoded map in `.py` |
| 10 | Bearing fix | World-frame bearing in likelihood | Particles track robot |
| 11 | Viz + prior | `/particle_markers`, `/tag_markers`, uniform default | Full §3.3 in RViz |
| 12 | Sim-to-real | `real/pf_localization_real`, `pf_node_ros1.py`, `dts` | PF on wolf + RViz via gui-tools |

---

## 2. Presentation and Experiment Videos (Guide)

*Project.pdf* requires the presentation to **explain what you did** and include **videos of simulation experiments**. §1 above is the spoken narrative; record clips that match each story beat.

### 2.1 Suggested talk structure (≈5–10 minutes)

1. **Problem** — localize Duckiebot in a 5×4 m room; eight tags, **same ArUco ID** → multi-hypothesis sensor model required.
2. **Simulation setup** — Gazebo Classic, room layout, asymmetric tags (§3.3–§3.4).
3. **What we built, in order** — walk through §1 Phases 1–11; mention placeholders → ArUco textures → YAML → bearing fix → weighted particles; optional Phase 12 bonus on real Duckiebot.
4. **Filter** — motion model, summed likelihood over eight tags, resampling (§4–§6); **no nearest-tag shortcut**.
5. **Demos (videos)** — table below.
6. **Results / limitations** — parallel green vs red paths until disambiguation; what “converged” looks like in RViz (§8.5).
7. **GitHub link** — repository URL for source (*Project.pdf* §4, §6).

### 2.2 Suggested videos to record

| # | Title (slide) | What to show | Relates to |
|---|----------------|--------------|------------|
| 1 | Bring-up | `ros2 launch pf_localization bringup.launch.py` — room, robot, RViz | Phase 4 |
| 2 | Placeholder era (optional) | Archive or slide: colored wall markers, green path not tracking red | Phase 3 + 7 |
| 3 | ArUco in Gazebo | Close-up of wall texture; not hazard stripes | Phase 8 |
| 4 | Initial particle cloud | Uniform prior: spread over room; weighted arrows red/green mix | Phase 11, *Project.pdf* §4 |
| 5 | Detection + driving | Teleop toward wall; **CameraAnnotated** with green ArUco box | Phase 6 + 8 |
| 6 | Partial convergence | Green/red parallel paths; cluster tightening | §8.5 |
| 7 | Converged | Weighted arrows on robot; green ≈ red | *Project.pdf* §4 final state |
| 8 | Tag map in RViz | Yellow T1–T8 markers + room outline | Phase 11 |
| 9 | Bonus (optional) | Real wolf: annotated camera + RViz particles/paths | Phase 12 |

**Commands for live demo / recording (simulation):**

```bash
cd ~/project/ros2_ws && colcon build --packages-select pf_localization && source install/setup.bash

# Terminal 1 — Gazebo, pf_node (YAML), RViz
ros2 launch pf_localization bringup.launch.py

# Terminal 2 — teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 2.3 Written report checklist (*Project.pdf* §4)

Use **§3–§11 (Technical reference)** for the formal write-up. Ensure the final PDF includes:

- [ ] Simulation setup (room, tags, robot, camera) — §3.2–§3.4, §9  
- [ ] Motion model and noise parameters — §4  
- [ ] Multi-hypothesis sensor model — §5  
- [ ] Bayesian filtering discussion (prior / likelihood / posterior per step) — *to add in final report section if not yet written*  
- [ ] Screenshots or videos at initial / partial / final convergence — §2.2  
- [ ] GitHub link — fill in when repository is public  
- [ ] Bonus section (if claiming extra credit) — §12, real-robot video  

---

## Part II — Technical Reference (Final System)

The following sections document the **as-built** system: mathematics, parameters, topics, and configuration. Read together with §1 for a complete picture.

---

## 3. Simulation Environment & Setup

### 3.1 Software Infrastructure

The project is organized as a standard ROS 2 `colcon` workspace (`ros2_ws`) built on Ubuntu 22.04 with ROS 2 Humble. The primary software package is `pf_localization`, implemented as an `ament_python` package using `rclpy`. Build and overlay integration follow ROS 2 conventions: packages are compiled with `colcon build`, and the workspace overlay is sourced via `install/setup.bash` so that executables, URDF assets, world files, launch files, and parameter files are discoverable through the ament index.

Declared runtime dependencies include `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`, `gazebo_ros_pkgs`, `xacro`, `urdf`, `robot_state_publisher`, `launch`, `launch_ros`, `gazebo_ros`, and `rviz2`. Vision processing additionally requires `cv_bridge` and the system OpenCV Python bindings (`python3-opencv`). Manual driving uses `teleop_twist_keyboard` from `ros-humble-teleop-twist-keyboard`. The particle filter node is registered as the executable `pf_node` in `setup.py`. Package assets (URDF, meshes, worlds, launch files, RViz configuration, parameter YAML, Gazebo material scripts and textures, and documentation under `docs/`) are installed into `share/pf_localization/` via `colcon build`. The `package.xml` export block declares `<gazebo_ros gazebo_media_path="${prefix}"/>` so Gazebo can resolve package-local media when launched outside the provided bring-up file.

### 3.2 Duckiebot Kinematic Description and Gazebo Integration

The Duckiebot physical model was extracted from the upstream Duckietown simulation repository (`duckiebot_description`) and integrated into `pf_localization` under `urdf/` and `meshes/`. The description comprises:

- `duckiebot.xacro` — primary robot kinematic tree (links: `chassis`, `left_wheel`, `right_wheel`, `computer`, `camera`);
- `duckiebot.gazebo` — Gazebo Classic extensions and sensor/actuator plugins;
- `materials.xacro` and `macros.xacro` — supporting XACRO definitions;
- seven COLLADA (`.dae`) mesh assets for visual and collision geometry.

All package-relative resource URIs were migrated from `package://duckiebot_description/...` to `package://pf_localization/...`, and XACRO includes were rewritten to resolve via `$(find pf_localization)`.

#### 3.2.1 ROS 2 Gazebo Classic Plugin Migration

The legacy ROS 1–style plugin configuration was replaced with ROS 2 Humble–compatible Gazebo Classic plugins using snake_case parameter tags and explicit ROS remapping blocks.

**Differential drive** (`libgazebo_ros_diff_drive.so`):

| Parameter | Value |
|-----------|-------|
| `left_joint` / `right_joint` | `left_wheel_hinge` / `right_wheel_hinge` |
| `wheel_separation` | 0.14 m |
| `wheel_diameter` | 0.064 m |
| `update_rate` | 50 Hz |
| `odometry_frame` | `odom` |
| `robot_base_frame` | `chassis` |
| Remappings | `/cmd_vel`, `/odom` |

The plugin publishes wheel odometry and the `odom` → `chassis` transform.

**Camera** (`libgazebo_ros_camera.so`):

| Parameter | Value |
|-----------|-------|
| ROS namespace | `/camera` |
| Image topic (actual) | `/camera/camera_sensor/image_raw` (sensor name `camera_sensor` appended by `gazebo_ros_camera`) |
| Camera info | `/camera/camera_sensor/camera_info` |
| `frame_name` | `camera` |
| Resolution | 640 × 480, `R8G8B8` |
| `horizontal_fov` | 1.04 rad |
| `update_rate` | 30 Hz |

Wheel contact parameters (`mu1`, `mu2`, `kp`, `kd`) were specified on the drive wheels to ensure physically plausible interaction with the ground plane.

### 3.3 Enclosed Room World Model

The simulation environment is defined in `worlds/room_tags.world` using **SDF version 1.6** for Gazebo Classic. The navigable enclosure is a closed rectangle with interior extent:

\[
x \in [0,\,5.0]\ \text{m}, \qquad y \in [0,\,4.0]\ \text{m}.
\]

Structural elements include:

- A floor box of size \(5.0 \times 4.0 \times 0.02\) m, centered at \((2.5,\,2.0,\,0)\);
- Four wall segments of height \(1.0\) m and thickness \(0.1\) m, placed on the south (\(y=0\)), north (\(y=4\)), west (\(x=0\)), and east (\(x=5\)) boundaries, each with matched `<visual>` and `<collision>` elements to prevent penetration by the mobile base.

### 3.4 AR Tag Landmarks in Simulation

Eight landmark markers are mounted in simulation as \(0.2 \times 0.2\) m planar targets at height \(z = 0.2\) m (tag center). Each marker is a thin box visual flush to the interior wall surface, textured with a **shared** ArUco pattern (`DICT_6X6_250`, marker ID 0) via the custom Ogre material `Aruco/Marker` (§9). **Implementation history:** markers were first **colored placeholders** (§1 Phase 3) for layout validation, then replaced with identical ArUco textures (§1 Phase 8). All eight tags are visually identical, matching the project constraint that the robot cannot distinguish markers by ID. Geometric placement and nominal outward-facing orientations match the project specification and are mirrored in the filter parameter file `config/pf_params.yaml`.

| Tag ID | Wall | Center \((x,\,y)\) [m] | Nominal yaw [rad] | Facing direction |
|--------|------|------------------------|-------------------|------------------|
| 1 | North (\(y=4.0\)) | \((1.0,\,4.0)\) | \(-1.57\) | South (\(-\hat{y}\)) |
| 2 | North (\(y=4.0\)) | \((3.5,\,4.0)\) | \(-1.57\) | South (\(-\hat{y}\)) |
| 3 | South (\(y=0.0\)) | \((0.5,\,0.0)\) | \(+1.57\) | North (\(+\hat{y}\)) |
| 4 | South (\(y=0.0\)) | \((4.0,\,0.0)\) | \(+1.57\) | North (\(+\hat{y}\)) |
| 5 | East (\(x=5.0\)) | \((5.0,\,1.5)\) | \(\pi\) | West (\(-\hat{x}\)) |
| 6 | East (\(x=5.0\)) | \((5.0,\,3.0)\) | \(\pi\) | West (\(-\hat{x}\)) |
| 7 | West (\(x=0.0\)) | \((0.0,\,0.8)\) | \(0\) | East (\(+\hat{x}\)) |
| 8 | West (\(x=0.0\)) | \((0.0,\,3.2)\) | \(0\) | East (\(+\hat{x}\)) |

The layout is **asymmetric** (two markers per wall at non-uniform along-wall positions), which is essential for global localization: symmetric arrangements admit pose ambiguities that cannot be resolved by bearing-only landmark observations.

### 3.5 System Bring-Up

The launch file `launch/bringup.launch.py` concurrently starts Gazebo Classic with `room_tags.world`, processes `duckiebot.xacro` through `robot_state_publisher`, spawns the robot at \((x,\,y,\,z) = (2.5,\,2.0,\,0.05)\) m via `gazebo_ros` `spawn_entity.py` (with `-package_to_model` for mesh resolution), and opens RViz2 with the preconfigured layout `rviz/pf_loc.rviz` (passed via `rviz2 -d`). The spawn height offset prevents initial interpenetration with the ground plane while preserving the nominal planar placement at the room center. A **8 s** timer delays spawn until Gazebo has loaded the world.

**Gazebo environment paths** (required for meshes, shaders, and ArUco textures):

| Variable | Appended path | Purpose |
|----------|---------------|---------|
| `GAZEBO_MODEL_PATH` | `share/pf_localization/..` | Resolve `model://pf_localization/meshes/...` URIs |
| `GAZEBO_RESOURCE_PATH` | `/usr/share/gazebo-11` | System shaders and default materials (must not be omitted) |
| `GAZEBO_RESOURCE_PATH` | `share/pf_localization` | Package `materials/scripts` and `materials/textures` |

Omitting the system resource path causes Gazebo Classic to fail shader initialization (`Unable to find shader lib`), gzclient crashes, and custom textures fall back to the yellow/black “missing texture” hazard pattern. Both paths are appended in `bringup.launch.py`.

### 3.6 Integrated Runtime Configuration

The simulation stack is operated as **two** processes after workspace overlay sourcing:

1. `ros2 launch pf_localization bringup.launch.py` — Gazebo, `robot_state_publisher`, **`pf_node` with `config/pf_params.yaml`**, and RViz2 (`pf_loc.rviz`);
2. `ros2 run teleop_twist_keyboard teleop_twist_keyboard` — manual `/cmd_vel` commands.

Verified behavior: textured wall markers; `/odom` and camera stream; \(N=500\) particles (uniform prior by default); ArUco overlays in `/camera/image_annotated`; `/particle_markers` (weight-colored), `/tag_markers`, `/odom_path`, `/pf_path`, `/estimated_pose`.

---

## 4. Motion Model (Prediction Step)

### 4.1 Odometry Interface

The particle filter node (`pf_node`) subscribes to `nav_msgs/Odometry` on `/odom`, which is produced by the Gazebo differential-drive plugin. Each message provides the noisy wheel-odometry pose estimate of the `chassis` frame expressed in the `odom` frame.

### 4.2 Incremental Motion Extraction

Let \((x_t,\,y_t,\,\theta_t)\) denote the odometry pose at time step \(t\). Upon receipt of consecutive odometry messages, the node computes the translational increment in the **previous** robot frame (body frame at \(t-1\)):

\[
\begin{aligned}
\Delta x_{\mathrm{b}} &= \cos(\theta_{t-1})\,(x_t - x_{t-1}) + \sin(\theta_{t-1})\,(y_t - y_{t-1}), \\
\Delta y_{\mathrm{b}} &= -\sin(\theta_{t-1})\,(x_t - x_{t-1}) + \cos(\theta_{t-1})\,(y_t - y_{t-1}), \\
\Delta\theta &= \mathrm{wrap}\!\left(\theta_t - \theta_{t-1}\right),
\end{aligned}
\]

where \(\mathrm{wrap}(\cdot)\) maps angles to \([-\pi,\,\pi]\).

### 4.3 Particle Propagation with Gaussian Odometry Noise

Each particle \(j\) carries a pose \((x^{(j)},\,y^{(j)},\,\theta^{(j)})\) and non-negative weight \(w^{(j)}\). The prediction step applies the body-frame increment with additive Gaussian noise to model slip, encoder error, and integration uncertainty:

\[
\begin{aligned}
\widetilde{\Delta x}_{\mathrm{b}} &= \Delta x_{\mathrm{b}} + \mathcal{N}(0,\,\sigma_{\mathrm{trans}}^2), \\
\widetilde{\Delta y}_{\mathrm{b}} &= \Delta y_{\mathrm{b}} + \mathcal{N}(0,\,\sigma_{\mathrm{trans}}^2), \\
\widetilde{\Delta\theta} &= \Delta\theta + \mathcal{N}(0,\,\sigma_{\mathrm{rot}}^2),
\end{aligned}
\]

followed by the rigid-body update

\[
\begin{aligned}
x^{(j)} &\leftarrow x^{(j)} + \widetilde{\Delta x}_{\mathrm{b}} \cos\theta^{(j)} - \widetilde{\Delta y}_{\mathrm{b}} \sin\theta^{(j)}, \\
y^{(j)} &\leftarrow y^{(j)} + \widetilde{\Delta x}_{\mathrm{b}} \sin\theta^{(j)} + \widetilde{\Delta y}_{\mathrm{b}} \cos\theta^{(j)}, \\
\theta^{(j)} &\leftarrow \mathrm{wrap}\!\left(\theta^{(j)} + \widetilde{\Delta\theta}\right).
\end{aligned}
\]

Particles are subsequently clamped to the configured room bounds \([x_{\min},\,x_{\max}] \times [y_{\min},\,y_{\max}]\) to respect the known spatial support of the prior.

Default noise parameters (declared as ROS parameters, loaded from `config/pf_params.yaml`) are \(\sigma_{\mathrm{trans}} = 0.05\) m and \(\sigma_{\mathrm{rot}} = 0.05\) rad. These values are not hard-coded in the algorithmic core; they are supplied at runtime through the parameter server.

### 4.4 Initial Distribution

Default (**Project.pdf** “unknown location”): \(N = 500\) particles from a **uniform prior** over \((x,\,y,\,\theta)\) in the room rectangle and \([-\pi,\,\pi]\), weights \(w^{(j)} = 1/N\). Controlled by `seed_particles_from_odom: false` in `pf_params.yaml`.

Optional demo mode (`seed_particles_from_odom: true`): one-time Gaussian reinit around the first `/odom` pose with standard deviations `init_std_x`, `init_std_y`, `init_std_yaw`.

---

## 5. Sensor Model (Multi-Hypothesis Update Step)

### 5.1 Observation Representation

A camera measurement is abstracted as a tag observation vector

\[
\mathbf{z} = \begin{bmatrix} r \\ \beta \end{bmatrix},
\]

where \(r\) is measured range to the marker (pinhole + marker width) and \(\beta_{\mathrm{world}}\) is measured **world-frame** bearing to the landmark:

\[
\beta_{\mathrm{world}} = \mathrm{wrap}\!\left(\psi_{\mathrm{odom}} + \beta_{\mathrm{camera}} + \texttt{camera\_yaw\_offset}\right),
\]

with \(\psi_{\mathrm{odom}}\) from the latest `/odom` yaw and \(\beta_{\mathrm{camera}} = \atan2((u - c_x),\, f_x)\) from the image center \(u\). Range–bearing pairs enter `apply_tag_observation()` unchanged through the multi-hypothesis sum (§5.3).

### 5.2 Single-Hypothesis Likelihood

For a particle at position \((x^{(j)},\,y^{(j)})\) and landmark \((x_i,\,y_i)\):

\[
\hat{r}_i = \sqrt{(x_i - x^{(j)})^2 + (y_i - y^{(j)})^2}, \qquad
\hat{\beta}_{i,\mathrm{world}} = \atan2(y_i - y^{(j)},\, x_i - x^{(j)}).
\]

Particle heading \(\theta^{(j)}\) is **not** used in the bearing term (Phase 10 fix). Gaussian likelihood:

\[
p(\mathbf{z} \mid \mathbf{x}^{(j)},\, \mathrm{tag}_i) =
\exp\!\left(
-\frac{1}{2}
\left[
\frac{(r - \hat{r}_i)^2}{\sigma_r^2}
+
\frac{\left(\mathrm{wrap}(\beta_{\mathrm{world}} - \hat{\beta}_{i,\mathrm{world}})\right)^2}{\sigma_\beta^2}
\right]
\right).
\]

YAML defaults: `sensor_noise_range` \(= 0.25\) m, `sensor_noise_bearing` \(= 0.12\) rad; motion noise `odom_noise_trans` / `odom_noise_rot` \(= 0.05\).

### 5.3 Multi-Hypothesis Summation (Identical Marker IDs)

All eight physical markers share the **same ArUco identifier**. A single detection therefore does not uniquely identify which wall-mounted tag produced the observation. Treating the measurement as originating from the geometrically nearest tag constitutes a **data-association shortcut** that collapses the mixture distribution to a single mode. Under pose ambiguity—particularly in an asymmetric but partially symmetric environment—such shortcuts induce weight degeneracy at incorrect poses and are known to destabilize Monte Carlo localization.

The implemented sensor model adheres to the project-mandated **mixture likelihood**:

\[
\boxed{
p(\mathbf{z} \mid \mathbf{x}) = \sum_{i=1}^{8} p(\mathbf{z} \mid \mathbf{x},\, \mathrm{tag}_i)
}
\]

Each particle weight is updated multiplicatively according to Bayes’ rule:

\[
w^{(j)} \leftarrow w^{(j)} \cdot p(\mathbf{z} \mid \mathbf{x}^{(j)}),
\]

followed by normalization over \(j\). This summation propagates the association uncertainty through the measurement update rather than pre-committing to a single landmark correspondence.

**Explicit design constraint:** No nearest-neighbor tag selection, no maximum-likelihood hard assignment, and no pre-filtering of hypotheses appear in the update implementation. All eight tag coordinates loaded from ROS parameters participate in every measurement update.

### 5.4 Tag World Coordinates in the Filter

The eight hypothesis positions are stored as parameter arrays `tag_x` and `tag_y` (see §3.4). This parameterization decouples the mathematical core from static source-code literals and ensures consistency between the simulation world, configuration file, and estimator.

---

## 6. Filter Pipeline & Particle Management

### 6.1 Complete Estimation Cycle

The `pf_node` implements the standard Monte Carlo localization recursion:

1. **Prediction:** triggered on each `/odom` message (§4);
2. **Update:** invoked when `apply_tag_observation()` receives a valid \(\mathbf{z}\) from ArUco detections (§5 and §7);
3. **Resampling:** conditional on effective sample size (§6.2);
4. **Output:** periodic publication of the particle set, estimated pose, trajectory paths, and annotated camera image (§6.3, §7, and §8).

While odometry prediction runs continuously during simulation, the measurement update executes whenever one or more markers are detected in a camera frame. Each detection induces an independent call to `apply_tag_observation()`, applying the summed likelihood of §5.3 followed by conditional resampling. In the absence of detections, the cloud evolves under prediction only and disperses according to \(\sigma_{\mathrm{trans}}\) and \(\sigma_{\mathrm{rot}}\); when valid range–bearing measurements arrive, the multi-hypothesis update concentrates posterior mass around poses consistent with the observation under all eight landmark hypotheses.

### 6.2 Low-Variance Resampling

After normalization, particle degeneracy is monitored through the **effective sample size**:

\[
N_{\mathrm{eff}} = \frac{1}{\sum_{j=1}^{N} \left(w^{(j)}\right)^2}.
\]

When

\[
N_{\mathrm{eff}} < \eta_{\mathrm{th}}\, N,
\]

with default \(\eta_{\mathrm{th}} = 0.5\), the filter applies **low-variance resampling** (systematic resampling) to reduce duplicate impoverishment while preserving stochastic diversity. Resampled particles receive uniform weights \(w^{(j)} = 1/N\).

The algorithm maintains a single random offset \(r \sim \mathcal{U}(0,\,1/N)\) and walks the cumulative weight distribution with a fixed stride \(1/N\), yielding \(\mathcal{O}(N)\) complexity and lower variance than multinomial resampling.

### 6.3 State Estimation and ROS Visualization

**Weighted mean pose.** The reported robot pose is the posterior mean:

\[
\hat{x} = \sum_{j=1}^{N} w^{(j)} x^{(j)}, \qquad
\hat{y} = \sum_{j=1}^{N} w^{(j)} y^{(j)},
\]

with circular mean for orientation:

\[
\hat{\theta} = \atan2\!\left(\sum_{j} w^{(j)} \sin\theta^{(j)},\,\sum_{j} w^{(j)} \cos\theta^{(j)}\right).
\]

**Publications.**

| Topic | Message type | Content |
|-------|--------------|---------|
| `/particles` | `geometry_msgs/PoseArray` | Legacy pose array (optional debug) |
| `/particle_markers` | `visualization_msgs/MarkerArray` | Weight-colored arrows (§8.2) |
| `/tag_markers` | `visualization_msgs/MarkerArray` | Room outline + eight tag positions T1–T8 |
| `/estimated_pose` | `geometry_msgs/PoseStamped` | Weighted mean \((\hat{x},\,\hat{y},\,\hat{\theta})\) |
| `/odom_path` | `nav_msgs/Path` | Odometry-only trajectory (§8) |
| `/pf_path` | `nav_msgs/Path` | PF estimate trajectory (§8) |
| `/camera/image_annotated` | `sensor_msgs/Image` | ArUco detection debug (§7) |

The particle, pose, and path topics are published at `publish_rate_hz = 10\) Hz via a wall timer, decoupling visualization cadence from asynchronous odometry and measurement events. Odometry poses are additionally appended to the odometry path history on each `/odom` callback. The annotated image stream is published synchronously with each processed camera frame. This design permits RViz2 to render the full particle cloud, point estimate, dual trajectories, and live perception debug view during continuous operation.

### 6.4 Parameterization Summary

All structural constants—particle count, room bounds, tag coordinates, noise standard deviations, resampling threshold, publication rate, coordinate frame, trajectory history length, and camera image topic—are declared as ROS 2 parameters in `pf_node` with defaults centralized in `config/pf_params.yaml`, supplemented by node-level declarations for `path_max_poses` (default \(0\), unlimited) and `camera_image_topic` (default `/camera/camera_sensor/image_raw`). The implementation contains no embedded literals for room geometry or landmark positions within the algorithmic update logic. Camera intrinsics and physical tag size used for monocular range estimation are defined as module-level constants in `pf_node.py`, consistent with the Gazebo camera specification in §3.2.1.

---

## 7. ArUco Vision and Image Processing

### 7.1 Software Stack and Image Ingress

The executable `pf_node` integrates OpenCV (`cv2`, `cv2.aruco`) with `cv_bridge` to consume `sensor_msgs/Image` messages on the configurable topic `camera_image_topic` (default `/camera/camera_sensor/image_raw`, matching the Gazebo camera plugin output). Each callback performs the following sequence:

1. Conversion from ROS Image to an OpenCV BGR matrix via `CvBridge.imgmsg_to_cv2(..., encoding='bgr8')`;
2. Grayscale conversion for marker detection;
3. Publication of an annotated output image on `/camera/image_annotated` for RViz2 or `rqt` inspection.

OpenCV API compatibility is handled at initialization: systems exposing `cv2.aruco.DetectorParameters` use the modern constructor, while older distributions fall back to `DetectorParameters_create()`.

### 7.2 Marker Detection

Markers are detected with the predefined dictionary **`cv2.aruco.DICT_6X6_250`**, invoked through `cv2.aruco.detectMarkers`. Detected corner polygons and identifiers are rendered on the output image using `cv2.aruco.drawDetectedMarkers`. The detector does not discriminate among marker IDs at the data-association stage: all detections are treated as observations of the same logical landmark class, consistent with the identical ArUco ID shared by all eight wall-mounted tags in the project specification.

### 7.3 Range and Bearing Extraction

For each detected marker, corner pixels are reduced to a horizontal center coordinate \(u\) and an apparent width \(w_{\mathrm{px}}\) (extent along the image \(u\)-axis). A pinhole-camera approximation, calibrated to the Duckiebot simulation camera, yields:

**Focal length** (from horizontal field of view \(\mathrm{HFOV} = 1.04\) rad and image width 640 px):

\[
f_x = \frac{(W/2)}{\tan(\mathrm{HFOV}/2)}.
\]

**Range** (known physical tag edge length \(L = 0.2\) m):

\[
r \approx \frac{f_x \cdot L}{w_{\mathrm{px}}}.
\]

**Bearing** in the camera frame (horizontal offset from principal point \(c_x = W/2\)):

\[
\beta = \atan2(c_x - u,\, f_x).
\]

Invalid or degenerate measurements (\(w_{\mathrm{px}} < 1\) px, non-finite or non-positive range) are discarded. The bearing is used directly as the observation \(\beta\) in the filter under the planar forward-camera approximation; this interfaces without alteration to the likelihood function in §5.2.

### 7.4 Coupling to the Multi-Hypothesis Filter

Each valid detection \(\mathbf{z} = [r,\,\beta]^\top\) is wrapped as a `TagObservation` and passed to `apply_tag_observation()`, which executes:

\[
w^{(j)} \leftarrow w^{(j)} \cdot \sum_{i=1}^{8} p(\mathbf{z} \mid \mathbf{x}^{(j)},\, \mathrm{tag}_i),
\]

followed by weight normalization and conditional low-variance resampling (§6.2). When multiple markers appear in a single frame, the update is applied **sequentially** for each detection, which corresponds to incorporating the product of measurement likelihoods under conditional independence—a standard recursive Bayesian update.

No nearest-neighbor tag pruning is applied at the vision stage: association uncertainty is retained entirely within the summation over the eight configured world positions (`tag_x`, `tag_y` parameters).

### 7.5 Simulation Environment Integration

Wall markers in `room_tags.world` use the `Aruco/Marker` material (§9). With textures loaded correctly, `cv2.aruco.detectMarkers` succeeds when a tag fills the camera field of view; `/camera/image_annotated` shows green corner overlays. Each valid detection triggers `apply_tag_observation()` and the multi-hypothesis update of §5.3.

**Operational note:** Because all eight tags share ID 0, a single sighting does not uniquely fix global pose. The particle cloud may track **relative motion** (green and red paths moving in parallel) while retaining a **constant translational offset** until disambiguating observations from multiple walls break symmetry. This is expected under the mandated mixture sensor model, not a defect in path recording.

### 7.6 Subscriptions and Publications Summary

| Direction | Topic | Role |
|-----------|-------|------|
| Subscribe | `/odom` | Prediction (§4) |
| Subscribe | `/camera/camera_sensor/image_raw` | ArUco detection ingress (configurable) |
| Publish | `/particles`, `/particle_markers` | Particle cloud (pose array + weight-colored markers) |
| Publish | `/tag_markers` | Room + tag map for RViz |
| Publish | `/estimated_pose` | Weighted mean pose estimate |
| Publish | `/odom_path` | Odometry trajectory (§8) |
| Publish | `/pf_path` | Particle-filter trajectory (§8) |
| Publish | `/camera/image_annotated` | Detection debug image |

---

## 8. Trajectory Visualization and Teleoperation

### 8.1 Dual Trajectory Recording in `pf_node`

To satisfy the project requirement of comparing dead-reckoning against filtered localization, `pf_node` maintains two pose histories and publishes them as `nav_msgs/Path`:

**Odometry path (`/odom_path`).** On every `nav_msgs/Odometry` message received on `/odom`, the current wheel-odometry pose (position and orientation from the message, expressed in `map_frame`) is appended to an internal list. This records the **integrated odometry trajectory** as reported by the Gazebo differential-drive plugin, independent of particle states.

**Particle-filter path (`/pf_path`).** At the visualization timer rate (`publish_rate_hz = 10\) Hz), after computing the weighted mean pose \((\hat{x},\, \hat{y},\, \hat{\theta})\), that estimate is appended to a second list. This records the **filter’s pose estimate over time**, reflecting the combined effect of prediction, multi-hypothesis updates (when detections occur), and resampling.

Both paths are published on each timer tick with header `frame_id` equal to `map_frame` (default `odom`), enabling RViz2 `Path` displays to render the full history. An optional parameter `path_max_poses` (default \(0\)) truncates each history to the most recent \(N\) poses when set to a positive integer, bounding memory use during long runs.

### 8.2 RViz2 Configuration (`rviz/pf_loc.rviz`)

A dedicated RViz2 configuration file preloads the displays required for project visualization:

| Display | Topic / source | Visual encoding |
|---------|----------------|-----------------|
| Grid | — | Floor reference |
| RobotModel | `/robot_description` | Duckiebot mesh |
| ParticlesWeighted | `/particle_markers` | Arrows: **green** = high weight, **red** = low |
| ARTags | `/tag_markers` | Gray room border, yellow tag cubes, T1–T8 labels |
| OdomPath | `/odom_path` | **Red** trajectory |
| PFPath | `/pf_path` | **Green** trajectory |
| CameraAnnotated | `/camera/image_annotated` | Live ArUco overlays |

The global fixed frame is set to `odom`, and the default camera view is centered on \((2.5,\,2.0,\,0)\) m, coinciding with the room center and nominal spawn pose.

### 8.3 Launch Integration

`bringup.launch.py` passes the configuration file to RViz2:

```text
rviz2 -d share/pf_localization/rviz/pf_loc.rviz
```

Operators therefore obtain the full visualization context immediately upon launch, without manual display setup.

### 8.4 Manual Teleoperation

Exploratory trajectories are generated with the standard ROS 2 keyboard teleoperator:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The node publishes `geometry_msgs/Twist` commands on `/cmd_vel`, which the Gazebo `libgazebo_ros_diff_drive` plugin consumes. Driving the robot around the enclosed room produces a coherent red odometry path. With ArUco textures active, measurement updates concentrate particle weights when tags are visible; operators should drive toward walls and vary heading so multiple landmark hypotheses can be pruned (§7.5).

### 8.5 Side-by-Side Trajectory Interpretation

| Phase | Red path (`/odom_path`) | Green path (`/pf_path`) | Blue particles |
|-------|-------------------------|-------------------------|----------------|
| Startup, uniform prior | Follows robot from spawn | Mean near room center | Spread across room (red/green mix arrows) |
| After bearing fix + tags | Follows robot | Approaches red | **Green/red arrows on robot** |
| Fully localized | On robot | Overlaps red | Tight green cluster on robot |

After Phase 10, wrong-corner locking should not persist. Parallel green/red paths can still occur briefly under identical-ID ambiguity until multiple walls are observed. **RobotModel** should lie on the red path; **ParticlesWeighted** (green arrows) should sit on the robot when localized.

---

## 9. ArUco Textures for Gazebo Classic (Phase 8)

### 9.1 Marker Image Generation

The script `generate_marker.py` (package root) uses OpenCV `cv2.aruco` to render a **`DICT_6X6_250`** marker with **ID 0** at \(200 \times 200\) pixels. The output is written to:

```text
materials/textures/aruco_marker.png
```

The script supports both `generateImageMarker` (OpenCV 4.7+) and legacy `drawMarker`. Regenerate after changing marker ID or dictionary:

```bash
python3 src/pf_localization/generate_marker.py
colcon build --packages-select pf_localization
```

### 9.2 Ogre Material Definition

`materials/scripts/aruco.material` defines material name **`Aruco/Marker`**, referencing `aruco_marker.png` with trilinear filtering and clamp addressing. Diffuse and ambient are set to white so the texture is not tinted.

### 9.3 World File Binding

Each of the eight `ar_tag_*` models in `room_tags.world` applies:

```xml
<material>
  <script>
    <uri>file://materials/scripts</uri>
    <uri>file://materials/textures</uri>
    <name>Aruco/Marker</name>
  </script>
</material>
```

Gazebo expects **directory** URIs for scripts and textures (not a path to the `.material` file itself). Incorrect URIs (e.g. `file://materials/scripts/aruco.material`) prevent texture loading and produce the yellow/black diagonal hazard-stripe fallback, which OpenCV cannot decode as ArUco.

### 9.4 Installation and Resource Resolution

`setup.py` installs `materials/scripts/` and `materials/textures/` into `share/pf_localization/`. `bringup.launch.py` appends both `/usr/share/gazebo-11` and `share/pf_localization` to `GAZEBO_RESOURCE_PATH` (§3.5). `package.xml` exports `gazebo_media_path` for compatibility with `gazebo_ros` tooling.

### 9.5 Verification Checklist

1. Gazebo GUI opens without immediate gzclient crash after launch.
2. Wall markers show black-and-white ArUco squares (not hazard stripes).
3. RViz **CameraAnnotated** shows green detection outlines when facing a tag.
4. Weight-colored `/particle_markers` collapse onto the robot after tag observations.
5. Green `/pf_path` approaches overlap with red `/odom_path` after disambiguating motion.

---

## 10. Configuration File Reference (`config/pf_params.yaml`)

All tunables used by `pf_node` (no defaults duplicated in Python):

| Parameter | Role |
|-----------|------|
| `num_particles` | \(N\) (default 500) |
| `room_min_x` … `room_max_y` | Particle support / clamping |
| `tag_x`, `tag_y`, `tag_yaw` | Eight landmark hypotheses (yaw for documentation; bearing model uses centers) |
| `tag_size_m` | Physical marker size for range from bbox |
| `odom_noise_trans`, `odom_noise_rot` | Prediction noise \(\sigma\) |
| `sensor_noise_range`, `sensor_noise_bearing` | Measurement noise \(\sigma\) |
| `seed_particles_from_odom` | `false` = unknown pose (uniform); `true` = odom seed |
| `init_std_x`, `init_std_y`, `init_std_yaw` | Odom seed spread (if enabled) |
| `camera_*`, `camera_yaw_offset` | Pinhole model + mount correction |
| `publish_rate_hz`, `map_frame`, `path_max_poses`, `camera_image_topic` | ROS I/O |

Edit YAML → rebuild → relaunch; or override at launch with `--params-file`.

---

## 11. Remaining Work (Outside This Codebase)

| Item | Owner | Notes |
|------|--------|------|
| Formal report PDF | You | Bayesian interpretation, figures, cite §3–§10; optional §12 for bonus |
| Presentation slides + videos | You | §2.2 shot list; add bonus clip if demoing wolf |
| GitHub link in report | You | *Project.pdf* §4, §6 — include `real/` tree |
| Submission `.zip` | You | Report + slides only (code via GitHub) |
| On-robot validation video | You | Record wolf run after `dts devel build -H wolf -f` |

---

## 12. Bonus: Sim-to-Real (`~/project/real/`)

### 12.1 Scope and Separation from Simulation

| Aspect | Simulation (`ros2_ws`) | Real robot (`real/`) |
|--------|------------------------|----------------------|
| ROS distro | ROS 2 Humble | ROS 1 Noetic |
| Robot stack | Gazebo Classic + `pf_localization` | Duckietown `dts` DTProject |
| Node | `pf_node.py` (`rclpy`) | `pf_node_ros1.py` (`rospy`) |
| Odometry | `nav_msgs/Odometry` on `/odom` | `duckietown_msgs/Pose2DStamped` on `/{veh}/kinematics_node/pose` |
| Camera | `sensor_msgs/Image` raw | `sensor_msgs/CompressedImage` on `/{veh}/camera_node/image/compressed` |
| Fiducial family | ArUco `DICT_6X6_250`, ID 0 | **AprilTag `tag36h11`** (`tag_family` in YAML) |
| Filter math | Multi-hypothesis sum, world-frame bearing | **Identical** to simulation |

The `ros2_ws` package was **not modified** for bonus work.

### 12.2 Package Layout

```text
real/
├── Dockerfile
├── launchers/
│   ├── default.sh          # dts devel run → roslaunch pf_localization_real
│   ├── pf_localization.sh
│   └── rviz.sh             # gui-tools: rviz -d .../pf_loc.rviz
└── packages/pf_localization_real/
    ├── scripts/pf_node_ros1.py
    ├── config/pf_params.yaml
    ├── launch/pf_localization_real.launch
    └── rviz/pf_loc.rviz
```

### 12.3 Deployment (`dts`, robot `wolf`)

**Build on robot (recommended):**

```bash
cd ~/project/real
dts devel build -H wolf -f
```

**Run PF:**

```bash
dts devel run -H wolf
```

**Drive:**

```bash
dts duckiebot keyboard_control wolf
```

**Visualize (laptop — do not use a second `dts devel run`):**

```bash
cd ~/project/real
dts start_gui_tools --mount "$(pwd)" wolf
bash launchers/rviz.sh
```

**Laptop amd64 image** (only if using local `dts devel run`; not needed for gui-tools viz):

```bash
dts devel build -f
```

### 12.4 Published Topics (same names as simulation)

`/particles`, `/particle_markers`, `/tag_markers`, `/estimated_pose`, `/odom_path`, `/pf_path`, `/camera/image_annotated` — frame `odom` by default.

### 12.5 Physical Tags

Print **AprilTag tag36h11** markers at **0.2 m × 0.2 m**, **same ID** on all eight walls, at coordinates in `real/packages/pf_localization_real/config/pf_params.yaml` (matches simulation map). Simulation ArUco textures are **not** used on the physical robot.

### 12.6 Configuration Note

`pf_params.yaml` in the real package adds:

```yaml
tag_family: tag36h11
```

Requires `opencv-contrib-python>=4.7` for `DICT_APRILTAG_36h11`. Set `veh: wolf` via launch / `VEHICLE_NAME` env.

Further operator notes: `real/README_PF_REAL.md`.

---

*End of implementation log. Narrative: §1–§2. Simulation reference: §3–§11. Bonus: §12. Core simulation: **complete**. Bonus codebase: **implemented** (on-robot demo video: operator).*
