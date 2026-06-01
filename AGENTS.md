# Goal

Learn Agnocast and evaluate it with ROS 2 Humble.

# Repository

This repository contains experiments only.

The Agnocast dependency is located at:

../agnocast

# Rules

Read Agnocast source code as needed.

Do not modify Agnocast unless explicitly instructed.

All experiment code belongs in this repository.

# Build

Workspace root:

~/agnocast_ws

Build:

source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Deliverables

- codebase documentation
- benchmark scripts
- latency measurements
- comparison reports

## Codex Web context
Only this experiment repo is available.
Use docs/agnocast_codebase_map.md for Agnocast knowledge.

## Codex local context
The full ROS 2 workspace is at ~/agnocast_ws.
Agnocast is located at ~/agnocast_ws/src/agnocast.
The experiment repo is located at ~/agnocast_ws/src/agnocast-ros2-experiment.