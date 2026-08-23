# 3D Flight Trajectory Simulator (RK4)

A simple C++ project to simulate 3D projectile motion with air resistance using the RK4 (Runge-Kutta 4th order) algorithm. 

I built this because standard physics problems always ignore air drag and treat everything as a basic parabola. I wanted to see how much realistic drag changes the trajectory and practice writing numerical integrators from scratch instead of using external libraries.

## Project Structure
* [`main.cpp`](.code/main.cpp) — The C++ engine that calculates position and velocity over time using RK4 and outputs data.
* [`plot-data.py`](./scripts/plot-data.py) — Python script that reads the CSV and generates the visual plot.
* [`trajectory.csv`](./images/trajectory.csv) — Output dataset containing raw spatial telemetry.
* [`trajectory.png`](./images/trajectory.png) — Rendered trajectory graph.

## How to run it

First, make sure you have `pandas` and `matplotlib` installed:
```bash
pip install pandas matplotlib
