# drone-nav: Indoor Warehouse Drone Navigation & Simulation

A standalone, highly-configurable simulation system for indoor warehouse inventory-drone flight planning, kinematic trajectory simulation, marker+IMU localisation fusion, bin label scanning, and DRIMS ingestion schema export.

---

## Key Features

- **Pure Python Core**: Runs without ROS, Gazebo, or GPU requirements on standard workstations.
- **Strictly Configurable**: Zero hardcoded warehouse dimensions, drone dynamics, or sensor noise parameters.
- **Deterministic**: Seeded random generators guarantee 100% reproducible flights and sensor readings.
- **Robust Mission Planner**: A* navigation graph, aisle entry/exit rules, one-way aisles, and battery-aware flight splitting.
- **Localisation Fusion**: Extended Kalman Filter (EKF) combining simulated IMU dead-reckoning and fiducial markers.
- **DRIMS v1 Ingestion Contract**: Validated schema output for automated inventory ingestion.
- **Visualisation**: Top-down and 3D web inspection interface.

---

## Quick Start

### 1. Local Environment Setup

```bash
# Create and activate virtual environment
python -m venv .venv
.\.venv\Scripts\Activate.ps1   # On Windows PowerShell
# source .venv/bin/activate    # On Linux / macOS

# Install package in editable mode
pip install -e ".[dev]"
```

### 2. Validate Warehouse Presets

```bash
drone-nav validate configs/presets/small.yaml
drone-nav validate configs/presets/medium.yaml
drone-nav validate configs/presets/large.yaml
```

### 3. Generate Layout & Inspect Statistics

```bash
drone-nav layout generate configs/presets/medium.yaml --summary
```

### 4. Launch Viewer

```bash
drone-nav viewer --config configs/presets/medium.yaml --port 8000
```

---

## Docker Quick Start

```bash
docker compose build
docker compose up
```

Access the top-down warehouse viewer at `http://localhost:8000`.

---

## Architecture Overview

Refer to [`docs/PLAN.md`](docs/PLAN.md) for full system architecture, phase exit proofs, and the DRIMS ingestion data contract specification.
Refer to [`THIRD_PARTY.md`](THIRD_PARTY.md) for third-party component licences and attribution.
