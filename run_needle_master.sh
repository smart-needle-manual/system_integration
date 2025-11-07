#!/bin/bash
# Master script to run:
#   1. jhu_needle.launch.py
#   2. hyperion_interrogator calibrate_sensors
#   3. Slicer with ShapeCall module
# Each starts in its own terminal with delays between them.

set -e

# -----------------------------
# Step 1: Source environments
# -----------------------------
source /opt/ros/humble/setup.bash
source ~/sm_manual/install/setup.bash
echo "[MASTER] Environments sourced."

# -----------------------------
# Step 2: Launch the needle node (Terminal 1)
# -----------------------------
echo "[MASTER] Launching needle node..."
gnome-terminal -- bash -c "
  source /opt/ros/humble/setup.bash;
  source ~/sm_manual/install/setup.bash;
  echo '[TERMINAL 1] Starting jhu_needle.launch.py...';
  ros2 launch needle_shape_publisher jhu_needle.launch.py manual_mode:=True;
  exec bash
"

# -----------------------------
# Step 3: Wait for node to initialize
# -----------------------------
echo "[MASTER] Waiting 5 seconds before calibration..."
sleep 5

# -----------------------------
# Step 4: Calibrate sensors (Terminal 2)
# -----------------------------
echo "[MASTER] Launching calibration command..."
gnome-terminal -- bash -c "
  source /opt/ros/humble/setup.bash;
  source ~/sm_manual/install/setup.bash;
  echo '[TERMINAL 2] Running sensor calibration...';
  ros2 run hyperion_interrogator calibrate_sensors --ros-args -r __ns:=/needle;
  exec bash
"

# -----------------------------
# Step 5: Wait for calibration to start
# -----------------------------
echo "[MASTER] Waiting 5 seconds before launching Slicer..."
sleep 5

# -----------------------------
# Step 6: Launch Slicer with ShapeCall module (Terminal 3)
# -----------------------------
echo "[MASTER] Launching Slicer..."
gnome-terminal -- bash -c "
  cd ~/slicer-ros2/Slicer-SuperBuild-Release/Slicer-build;
  echo '[TERMINAL 3] Starting Slicer with ShapeCall module...';
  ./Slicer --additional-module-paths ../../NeedleShapeReceiver/ShapeCall;
  exec bash
"

echo "[MASTER] ✅ All processes launched successfully."

