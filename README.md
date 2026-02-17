# PID Tuning + Cross-Simulator Validation (SO101 Robot Arm)

**OS:** Ubuntu 24.04  
**ROS 2:** Jazzy  
**Simulators:** MuJoCo + Gazebo  
**Goal:** Tune P / PD / PI / PID under disturbances in MuJoCo, then replay the same commanded trajectory in Gazebo and compare behavior.

---

## 1) Project overview (what this repo is)

This project runs the **SO101** robot arm in **MuJoCo**, applies **strong perturbations**, and evaluates how controller gains change the motion:

- **P** (Ki=0, Kd=0)
- **PD** (Ki=0)
- **PI** (Kd=0)
- **PID** (all active)

After selecting the best configuration, we **export the commanded joint references to CSV** and **replay them in Gazebo** (no redesign of the motion).

**Trajectory (same for everyone):**
1) start pose → zero  
2) hold  
3) return to start  
4) hold  
Disturbances remain ON throughout.

---

## 2) Repo structure (important files)

- `run_mujoco_simulation.py`  
  Main runner: viewer + PID loop + (optional) realtime plots + (optional) CSV export.
- `so101_mujoco_pid_utils.py`  
  PID loop + disturbances hooks + logging hooks.
- `so101_control.py`  
  `JointPID`, `PIDGains`, and perturbation model.
- `so101_mujoco_utils2.py`  
  Joint conversions (deg↔rad), pose helpers, and the Dash realtime plotter.
- `so101_ros_bridge.py`  
  CSV logger for commanded references (MuJoCo → Gazebo replay).
- `model/scene_urdf.xml`  
  MuJoCo model entry point (path used by scripts).

---

## 3) Requirements

### System packages (Ubuntu 24.04)
You need OpenGL + GLFW for the MuJoCo viewer:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip git \
  libgl1 libglfw3 libglew2.2 libosmesa6
