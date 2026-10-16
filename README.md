# Drone Farm Simulation

## Overview

Python-based waypoint mission control for PX4 SITL
running in Gazebo.

## Requirements

- Ubuntu
- PX4-Autopilot
- Gazebo
- Python
- MAVSDK gRPC Python package
- PyYAML

## Project Structure

- config/mission.yaml: Mission configuration
- scripts/test_connection.py: PX4 connection test
- scripts/waypoint_mission.py: Waypoint mission execution
- requirements.txt: Python dependencies

## Setup

Create and activate a Python virtual environment,
then install dependencies from requirements.txt.

## Run Simulation

Start PX4 SITL and Gazebo using the project's
configured world.

## Run Mission

Execute scripts/waypoint_mission.py from the
drone-farm project.

## Configuration

Explain altitude_m, speed_m_s, waypoints,
return_to_launch_after_mission, and safety timeouts.

## Expected Result

The drone connects, uploads the mission, follows
the waypoints, and returns/lands according to
the configured mission behavior.
