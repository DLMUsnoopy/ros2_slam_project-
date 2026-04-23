# Docker — Platform Setup

## Who Uses What

| Person | OS | GPU | Dockerfile | Compose command |
|---|---|---|---|---|
| Ubuntu + NVIDIA | Linux | NVIDIA | `Dockerfile` | `compose.yaml` + `compose.nvidia.yaml` |
| WSL2 + NVIDIA | Windows/WSL2 | NVIDIA | `Dockerfile` | `compose.yaml` + `compose.nvidia.yaml` + `compose.wsl.yaml` |
| Ubuntu + no GPU | Linux | None | `Dockerfile` | `compose.yaml` only |
| Mac Apple Silicon | macOS ARM | None | `Dockerfile.mac` | `compose.yaml` + `compose.mac.yaml` |

**Full simulation + CV + tracking stack requires Linux + NVIDIA.**
Mac and no-GPU users can run ROS2 nodes and logic. They cannot run CUDA-dependent code or get usable Gazebo performance.

---

## Ubuntu + NVIDIA GPU

### One-time host setup
```bash
# Allow GUI windows
xhost +local:root

# Apply docker group if not active
newgrp docker
```

### Every session
```bash
xhost +local:root
cd docker/
docker compose -f compose.yaml -f compose.nvidia.yaml up -d
docker compose exec simulation zsh
```

### Inside the container
```bash
source ~/p03_ws/install/setup.zsh
ros2 run rmw_zenoh_cpp rmw_zenohd &
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch ipb_ros2_sim all.launch.py
```

---

## WSL2 + NVIDIA GPU (Windows)

### One-time host setup

1. Install [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/) — enable WSL2 backend during install
2. Install [NVIDIA CUDA drivers for WSL](https://developer.nvidia.com/cuda/wsl) — these are different from the standard NVIDIA drivers, don't skip this
3. GUI display — Windows 11 has WSLg built in which handles this automatically. Windows 10 needs [VcXsrv](https://sourceforge.net/projects/vcxsrv/) installed and running before launch.

### For Windows 11 (WSLg) — every session
```bash
# Inside WSL2 terminal
cd docker/
docker compose -f compose.yaml -f compose.nvidia.yaml -f compose.wsl.yaml up -d
docker compose exec simulation zsh
```

### For Windows 10 (VcXsrv) — every session
Start VcXsrv first (XLaunch → Multiple windows → Display number 0 → tick "Disable access control"), then:
```bash
export DISPLAY=$(grep nameserver /etc/resolv.conf | awk '{print $2}'):0
cd docker/
docker compose -f compose.yaml -f compose.nvidia.yaml -f compose.wsl.yaml up -d
docker compose exec simulation zsh
```

### Inside the container — same as Linux
```bash
source ~/p03_ws/install/setup.zsh
ros2 run rmw_zenoh_cpp rmw_zenohd &
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch ipb_ros2_sim all.launch.py
```

---

## Ubuntu + No Dedicated GPU

No changes needed to the Dockerfile — same image as the NVIDIA setup, just without the GPU reservation.

Gazebo will fall back to software rendering (LLVMpipe). It works but expect low FPS on complex scenes. Fine for logic development and testing ROS2 nodes.

### Every session
```bash
xhost +local:root
cd docker/
docker compose up -d        # base compose.yaml only, no nvidia override
docker compose exec simulation zsh
```

### Inside the container — same as above
```bash
source ~/p03_ws/install/setup.zsh
ros2 run rmw_zenoh_cpp rmw_zenohd &
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch ipb_ros2_sim all.launch.py
```

---

## Mac — Apple Silicon (M1/M2/M3/M4)

This uses a separate `Dockerfile.mac` that builds natively for `arm64`. This avoids Rosetta emulation for ROS2 nodes so at least the ROS2 parts run at full speed. Gazebo will still be slow — there is no GPU access inside Docker on Mac.

**CUDA nodes will not run on this platform.**

### One-time host setup

1. Install [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop/)
2. Install [XQuartz](https://www.xquartz.org/)
3. Open XQuartz → Preferences → Security → tick **"Allow connections from network clients"**
4. Restart XQuartz completely (quit and reopen)
5. In terminal:
```bash
xhost +localhost
```

### Every session

Make sure XQuartz is running first, then:
```bash
xhost +localhost
cd docker/

# First time only — build the Mac image (separate from the Linux image)
docker compose -f compose.yaml -f compose.mac.yaml build

# Start container
docker compose -f compose.yaml -f compose.mac.yaml up -d
docker compose exec simulation zsh
```

### Inside the container
```bash
source ~/p03_ws/install/setup.zsh
ros2 run rmw_zenoh_cpp rmw_zenohd &
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch ipb_ros2_sim all.launch.py
```

### What works on Mac

- ROS2 nodes, topics, services — ✅
- RViz — ✅ via XQuartz (may be slow)
- Gazebo GUI — ⚠️ software rendering, low FPS on complex scenes
- CUDA / CV stack / tracking — ❌ not available

---

## Stopping Everything (all platforms)

`Ctrl+C` in the launch terminal, then:
```bash
docker compose down
```

---

## Rebuilding the Image

If the Dockerfile changes:

```bash
# Linux / WSL2 / no-GPU
docker compose -f compose.yaml -f compose.nvidia.yaml build --no-cache

# Mac
docker compose -f compose.yaml -f compose.mac.yaml build --no-cache
```
