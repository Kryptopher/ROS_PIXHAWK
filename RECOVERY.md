# 🚁 Drone Mission System — Disaster Recovery Guide

**System:** Raspberry Pi + ROS2 Humble + ArduPilot SITL (WSL)  
**OS:** Ubuntu 22.04.5 LTS (Jammy) — arm64  
**Python:** 3.10.12  
**Last verified:** March 2026

---

## Overview

This document tells you exactly how to rebuild this drone system from scratch on a fresh Raspberry Pi. Follow the steps in order. Do not skip sections.

The system has three parts:
1. **ROS2 Humble** — installed via apt (system packages)
2. **ArduPilot workspace** (`ardu_ws`) — built from source using colcon
3. **Your custom mission files** — cloned from the `drone-mission` GitHub repo

---

## Part 1 — Base System Setup

### 1.1 Flash Ubuntu 22.04 (Jammy) to the Pi
Use Raspberry Pi Imager and select **Ubuntu Server 22.04 LTS (64-bit)**.  
Hostname: `rpi`, Username: `pi`

### 1.2 Update the system
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl wget git openssh-server build-essential
```

### 1.3 Set up SSH key for GitHub (same as before)
```bash
ssh-keygen -t ed25519 -C "your@email.com"
cat ~/.ssh/id_ed25519.pub
```
Add the public key to your GitHub account under Settings → SSH Keys.

---

## Part 2 — Install ROS2 Humble

Run these commands exactly. This installs the full desktop version (same as original).

```bash
# Set locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop (full install)
sudo apt update
sudo apt install -y ros-humble-desktop

# Install dev tools
sudo apt install -y ros-dev-tools python3-colcon-common-extensions python3-vcstool
```

### 2.1 Source ROS2 on every login
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Part 3 — Install MAVROS

```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs

# Install GeographicLib datasets (required for MAVROS)
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

> **Note:** If the above path doesn't work, use the script from your repo:
> ```bash
> sudo ~/install_geographiclib_datasets.sh
> ```

---

## Part 4 — Rebuild the ArduPilot Workspace

### 4.1 Create the workspace
```bash
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws/src
```

### 4.2 Clone ArduPilot at the exact branch used originally

**Branch:** `Copter-4.5`  
**Last known commit:** `b72075518f`

```bash
git clone https://github.com/ArduPilot/ardupilot.git --branch Copter-4.5
cd ardupilot
git submodule update --init --recursive
cd ..
```

> The `--recursive` flag pulls all of ArduPilot's own submodules (ChibiOS, MAVLink, DroneCAN, etc.) automatically. This takes a while on a Pi — be patient.

### 4.3 Install ArduPilot dependencies
```bash
cd ~/ardu_ws/src/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### 4.4 Build ArduCopter for SITL
```bash
cd ~/ardu_ws/src/ardupilot
./waf configure --board sitl
./waf copter
```

### 4.5 Add arducopter to PATH
```bash
echo 'export PATH=$PATH:~/ardu_ws/src/ardupilot/build/sitl/bin' >> ~/.bashrc
source ~/.bashrc
```

Verify it works:
```bash
arducopter --version
```

### 4.6 Build the ROS2 workspace
```bash
cd ~/ardu_ws
colcon build --symlink-install
echo "source ~/ardu_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Part 5 — Restore Your Mission Files

```bash
cd ~
git clone git@github.com:<your-username>/drone-mission.git
cd drone-mission

# Copy mission files to home directory
cp drone_mission.py ~/
cp mission_config.yaml ~/
cp start_ardupilot_ros.sh ~/
cp install_geographiclib_datasets.sh ~/
cp mav.parm ~/

# Restore terrain data
cp -r terrain/ ~/terrain/
```

---

## Part 6 — Configure the System

### 7.1 Make the startup script executable
```bash
chmod +x ~/start_ardupilot_ros.sh
```

### 7.2 Load ArduPilot parameters
After your first SITL connection, load your saved parameters:
```bash
# Via MAVProxy:
param load ~/mav.parm
```

### 7.3 Verify the full environment
```bash
# Check ROS2
ros2 topic list

# Check MAVROS is available
ros2 pkg list | grep mavros

# Check arducopter binary
arducopter --help
```

---

## Part 7 — Running the System

### Normal startup sequence:

**Step 1 — On the Pi, start ArduPilot + MAVROS:**
```bash
~/start_ardupilot_ros.sh
```

**Step 2 — On WSL (Windows side), start SITL if needed:**
```bash
# In WSL, navigate to your ArduPilot directory and run:
sim_vehicle.py -v ArduCopter --console --map
```

**Step 3 — Run a mission:**
```bash
python3 ~/drone_mission.py
```

---

## Key File Reference

| File | Purpose |
|------|---------|
| `start_ardupilot_ros.sh` | Launches ArduCopter SITL + MAVROS with stream rates configured |
| `drone_mission.py` | Main mission execution script |
| `mission_config.yaml` | Waypoints and flight parameters |
| `mav.parm` | Saved ArduPilot parameters for your vehicle |
| `install_geographiclib_datasets.sh` | One-time setup for GPS/terrain data |
| `terrain/` | Cached terrain data for flight area |

---

## Network / Connection Reference

| Connection | Address | Notes |
|-----------|---------|-------|
| MAVROS → SITL | `tcp://127.0.0.1:5760` | Configured in `start_ardupilot_ros.sh` |
| GCS (QGroundControl etc.) | `udp://@` | Forwarded by MAVROS |
| SITL default home | `-35.3609403, 149.1680841` | Canberra, AU (ArduPilot default) |

---

## Troubleshooting

**MAVROS fails to connect after startup**  
→ The `sleep 40` in `start_ardupilot_ros.sh` is intentional — MAVROS needs time to negotiate with SITL. Wait the full 40 seconds before assuming failure.

**`arducopter` command not found**  
→ Run `source ~/.bashrc` or check that the PATH line was added correctly in Step 4.5.

**colcon build fails with missing dependencies**  
→ Run `rosdep install --from-paths src --ignore-src -r -y` from inside `~/ardu_ws/` before building.

**GeographicLib errors in MAVROS**  
→ Re-run the install script in Part 3. This is a common miss on fresh installs.

**Submodule errors cloning ArduPilot**  
→ Make sure you have a working internet connection and GitHub SSH key set up (Part 1.3) before running `git submodule update`.
