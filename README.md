# PID Tuning + Cross-Simulator Validation (SO101 Robot Arm)

**OS:** Ubuntu 24.04
**ROS 2:** Jazzy
**Simulators:** MuJoCo + Gazebo
**Goal:** Tune P / PD / PI / PID under disturbances in MuJoCo, then replay the same commanded trajectory in Gazebo and compare behavior.

---

## 1) Project overview (what this repo is)

This project runs the **SO101** robot arm in **MuJoCo**, applies **strong perturbations**, and evaluates how controller gains change the motion:

* **P** (Ki=0, Kd=0)
* **PD** (Ki=0)
* **PI** (Kd=0)
* **PID** (all active)

After selecting the best configuration, we **export the commanded joint references to CSV** and **replay them in Gazebo** (no redesign of the motion).

**Trajectory (same for everyone):**

1. start pose → zero
2. hold
3. return to start
4. hold
   Disturbances remain ON throughout.

---

## 2) Repo structure (important files)

* `run_mujoco_simulation.py`
  Main runner: viewer + PID loop + (optional) realtime plots + (optional) CSV export.
* `so101_mujoco_pid_utils.py`
  PID loop + disturbances hooks + logging hooks.
* `so101_control.py`
  `JointPID`, `PIDGains`, and perturbation model.
* `so101_mujoco_utils2.py`
  Joint conversions (deg↔rad), pose helpers, and the Dash realtime plotter.
* `so101_ros_bridge.py`
  CSV logger for commanded references (MuJoCo → Gazebo replay).
* `model/scene_urdf.xml`
  MuJoCo model entry point (path used by scripts).

---

## 3) Requirements

### System packages (Ubuntu 24.04)

You need OpenGL + GLFW for the MuJoCo viewer:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip git \
  libgl1 libglfw3 libglew2.2 libosmesa6
```

> If you get OpenGL/GLX issues, you can run **headless** (no viewer) using `--headless` (explained below).

### Python

* Python **>= 3.10**
* Dependencies are listed in `pyproject.toml`:

  * `mujoco>=3.1.0`, `numpy`, `scipy`, `pandas`, `matplotlib`, `dash`, `plotly`

---

## 4) Setup (Python environment)

From the repo root:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install "mujoco>=3.1.0" numpy scipy pandas matplotlib dash plotly
```

---

## 5) Run MuJoCo (main workflow)

### Default run (viewer + PID + realtime plot)

```bash
python run_mujoco_simulation.py
```

* MuJoCo viewer opens
* Dash plot (realtime) is served at: `http://127.0.0.1:8050`

### Disable the realtime plot (faster / fewer dependencies)

```bash
python run_mujoco_simulation.py --no-plot
```

### Headless mode (no viewer, avoids many GLX/OpenGL issues)

```bash
python run_mujoco_simulation.py --headless --no-plot
```

---

## 6) Export commanded trajectory (MuJoCo → Gazebo)

To export the **commanded joint references** to a CSV file:

```bash
python run_mujoco_simulation.py --log-csv logs/mujoco_commands.csv
```

The CSV contains:

* `t`
* `<joint>_cmd` for each joint (stored in **radians**; gripper is also mapped to radians)

This is what you replay in Gazebo. The point is: **same motion commands**, then compare tracking differences.

---

## 7) Where to change PID gains (P / PD / PI / PID + combos)

PID gains live in `so101_mujoco_pid_utils.py` inside `build_default_pid()`.

Each joint uses:

* `kp`, `ki`, `kd`
* `i_limit` (integral clamp, anti-windup)
* `tau_limit` (torque saturation)

### Controller families

* **P:** set `ki=0`, `kd=0`
* **PD:** set `ki=0`
* **PI:** set `kd=0`
* **PID:** use all three

### Minimum experiments required

You must run **≥ 5 gain combinations per controller family**:

* 5×P, 5×PD, 5×PI, 5×PID

**Tip:** Name combos clearly and export logs consistently, e.g.

* `logs/P_combo1.csv`
* `logs/PD_combo3.csv`
* `logs/PID_combo5.csv`

---

## 8) Disturbances (how they’re applied)

Disturbances are injected as **extra joint torques** in the control loop (sinus + noise + impulses).
You can edit the perturbation parameters in:

* `so101_mujoco_pid_utils.py` → `build_default_perturbations()`
* and/or `so101_control.py` → `PerturbationConfig`

The activity requires disturbances to stay active. No “gravity cancel” tricks unless clearly justified.

---

## 9) Plotting (what to include in the report)

Minimum plots per run:

* **joint position vs time**

This repo supports:

* **Realtime joint plots** via Dash/Plotly (see Section 5)

For the final report plots, you can either:

* take clean screenshots of the plot page per run, **or**
* extend the code to save measured joint positions to CSV (recommended for clean Matplotlib figures).

---

## 10) Gazebo replay (ROS 2 Jazzy)

High-level steps:

1. Install ROS 2 Jazzy and a ROS-compatible Gazebo distribution
2. Launch the SO101 robot in Gazebo with ROS control enabled
3. Write/use a small ROS 2 node that reads the CSV and publishes a `JointTrajectory`
4. Replay the CSV **without changing the motion**, then compare:

   * tracking degradation
   * delays
   * oscillation differences
   * sensitivity to modeling changes

> Note: the exact Gazebo launch + ROS controller setup depends on the course-provided SO101 Gazebo package / URDF setup.

---

## 11) Troubleshooting

### “GLXBadWindow” / viewer crashes

Run headless:

```bash
python run_mujoco_simulation.py --headless --no-plot
```

### Dash/Plotly not working

Disable plotter:

```bash
python run_mujoco_simulation.py --no-plot
```

### Wrong joint units

Conventions used here:

* inputs/prints: **degrees** for revolute joints + **0..100** for gripper
* internal control + CSV export: **radians**

---

## 12) Credits / notes

This repository is for the activity:
**“PID Tuning and Cross-Simulator Validation for the SO101 Robot Arm”**
(Controller tuning under perturbations + transfer to Gazebo.)
