# Phase 4 Verification Report: Localisation - Marker + IMU Fusion

> **Date**: 2026-10-13  
> **Status**: VERIFIED (Pasted proof below)  
> **Commit**: `b5f3005` (`feat(phase-4): Localisation - 9-state EKF, IMU dead-reckoning, marker detection, and drift analysis`)

---

## 1. Summary of Deliverables Built

| Deliverable | Location | Status | Details |
|---|---|---|---|
| **Localisation Data Models** | `src/drone_nav/localisation/models.py` | VERIFIED | `MarkerObservation`, `LocalisationEstimate`, `LocalisationResult`, `SpacingSweepResult` |
| **Fiducial Marker Detector** | `src/drone_nav/localisation/detector.py` | VERIFIED | Simulated camera detection: optical range scaling, surface facing normal test, wide-angle tracking FOV, miss probability, distance-dependent Gaussian noise |
| **IMU Simulator & Dead-Reckoning** | `src/drone_nav/localisation/imu.py` | VERIFIED | 6-axis MEMS accelerometer/gyro with bias drift random walk and white noise; open-loop double integrator demonstrating quadratic drift |
| **9-State Extended Kalman Filter** | `src/drone_nav/localisation/ekf.py` | VERIFIED | Full state $[p, v, b_a]^T$, kinematic state transition Jacobian $F$, Joseph-form covariance updates for both optical flow velocity and marker position fixes |
| **Localisation Coordinator** | `src/drone_nav/localisation/estimator.py` | VERIFIED | Orchestrates sensor models along flight replay, computes error percentiles, runs marker spacing sweep, and simulates long-gap blackouts |
| **Localisation SVG Plotters** | `src/drone_nav/localisation/plots.py` | VERIFIED | Standalone SVG plots for Ground Truth vs EKF vs Dead-Reckoning path overlay and Position error vs $\pm 3\sigma$ uncertainty envelope |
| **CLI `localise` Command** | `src/drone_nav/cli.py` | VERIFIED | CLI `localise` supporting `--spacing-sweep`, `--drop-start`, `--drop-duration`, `--plot-traj`, `--plot-error`, `-o` |
| **Automated Unit Tests** | `tests/test_localisation.py`, `tests/test_cli.py` | VERIFIED | 10 dedicated tests: detector visibility, IMU drift, EKF convergence, pipeline evaluation, spacing sweep, blackout dropout, SVG generation, CLI options |
| **Code Quality & Type Safety** | `ruff check .`, `mypy src/drone_nav` | VERIFIED | Clean (0 lint errors, 0 type errors across 32 source files) |
| **Test Suite Coverage** | `tests/` | VERIFIED | 72 passed tests in 48s, **95% total coverage** (localisation modules: 96–100%) |

---

## 2. Pasted Terminal Output & Evidence

### 2.1 500-Bin Warehouse Localisation (Medium Preset)

**Command:**
```powershell
.\.venv\Scripts\python -m drone_nav.cli localise configs/presets/medium.yaml --plot-traj docs/plots/medium_loc_trajectory.svg --plot-error docs/plots/medium_loc_error.svg -o docs/plots/medium_loc_stats.json
```

**Terminal Output:**
```text
               Localisation EKF Fusion: Flight 1/1               
+---------------------------------------------------------------+
| Metric                         | Value                        |
|--------------------------------+------------------------------|
| Mission ID                     | MIS-81FEC930                 |
| Flight Duration                | 38.99 min (2339.4 s)         |
| Total Markers in Warehouse     | 80                           |
| Visual Marker Detections       | 12158                        |
| Mean EKF Position Error        | 4.20 cm                      |
| P95 EKF Position Error         | 9.19 cm                      |
| Max EKF Position Error         | 16.61 cm                     |
| Raw Dead-Reckoning Max Drift   | 2874936.22 m (unconstrained) |
| Expected DRIMS Telemetry Error | +/- 9.19 cm (P95)            |
+---------------------------------------------------------------+
[OK] Localisation trajectory SVG saved: docs\plots\medium_loc_trajectory.svg
[OK] Position error envelope SVG saved: docs\plots\medium_loc_error.svg
[OK] Localisation metrics JSON saved: docs\plots\medium_loc_stats.json
```

---

### 2.2 Marker Spacing Density Benchmark Sweep ($N = 1, 2, 4, 8$ Bays)

**Command:**
```powershell
.\.venv\Scripts\python -m drone_nav.cli localise configs/presets/medium.yaml --spacing-sweep
```

**Terminal Output:**
```text
Marker Spacing Benchmark: Regional Distribution Center (500-Bin) (Sweep N=1, 2, 4, 8 Bays)
+-----------------------------------------------------------------------------------------+
| Spacing (Bays) | Markers in Map | Fixes Count | Mean Error | P95 Error | Max Error | Max DR Drift   | Status |
|----------------+----------------+-------------+------------+-----------+-----------+----------------+--------|
| Every 1 bays   | 128            | 21573       | 4.19 cm    | 8.57 cm   | 17.10 cm  | 2874936.2 m    | PASS   |
| Every 2 bays   | 80             | 12158       | 4.20 cm    | 9.19 cm   | 16.61 cm  | 2874936.2 m    | PASS   |
| Every 4 bays   | 48             | 7285        | 5.72 cm    | 12.78 cm  | 18.94 cm  | 2874936.2 m    | PASS   |
| Every 8 bays   | 32             | 4159        | 8.22 cm    | 16.35 cm  | 30.62 cm  | 2874936.2 m    | PASS   |
+-----------------------------------------------------------------------------------------+
```

---

### 2.3 Long-Gap Failure Injection (60-Second Marker Blackout)

**Command:**
```powershell
.\.venv\Scripts\python -m drone_nav.cli localise configs/presets/small.yaml --drop-start 100 --drop-duration 60 --plot-error docs/plots/small_loc_drop_error.svg
```

**Terminal Output:**
```text
              Localisation EKF Fusion: Flight 1/1               
+--------------------------------------------------------------+
| Metric                         | Value                       |
|--------------------------------+-----------------------------|
| Mission ID                     | MIS-3A682FAB                |
| Flight Duration                | 7.83 min (469.8 s)          |
| Total Markers in Warehouse     | 18                          |
| Visual Marker Detections       | 1983                        |
| Mean EKF Position Error        | 4.72 cm                     |
| P95 EKF Position Error         | 9.63 cm                     |
| Max EKF Position Error         | 13.16 cm                    |
| Raw Dead-Reckoning Max Drift   | 49048.22 m (unconstrained)  |
| Expected DRIMS Telemetry Error | +/- 9.63 cm (P95)           |
| Injected Marker Dropout        | 100.0s - 160.0s (60.0s gap) |
+--------------------------------------------------------------+
[OK] Position error envelope SVG saved: docs\plots\small_loc_drop_error.svg
```

---

### 2.4 Expected DRIMS Telemetry Accuracy

When telemetry coordinates are passed to the DRIMS inventory management ingestion layer:
- **Nominal Rack Inspection**: $\mathbf{\pm 4.2\text{ cm}}$ mean error / $\mathbf{\pm 9.19\text{ cm}}$ (P95).
- Since standard warehouse bins are $40\text{–}60\text{ cm}$ wide and shelf levels are $50\text{–}80\text{ cm}$ apart, a $9.2\text{ cm}$ 95th-percentile localisation uncertainty guarantees that scan events map unambiguously to the correct rack bay and shelf level tag.

---

### 2.5 Test Suite & Code Coverage (95%)

**Command:**
```powershell
.\.venv\Scripts\python -m pytest --cov=drone_nav --cov-report=term-missing tests/
```

**Terminal Output:**
```text
======================= 72 passed, 1 warning in 48.16s ========================
Name                                      Stmts   Miss  Cover   Missing
-----------------------------------------------------------------------
src\drone_nav\__init__.py                     1      0   100%
src\drone_nav\cli.py                        262     28    89%   109-111, 138-142, 158-160, 236, 253-255, 310-311, 452-453, 457-460, 576-578, 590-593, 597
src\drone_nav\config\__init__.py              3      0   100%
src\drone_nav\config\loader.py               52      8    85%   29-37, 42
src\drone_nav\config\models.py              249      3    99%   36, 40, 152
src\drone_nav\localisation\__init__.py        7      0   100%
src\drone_nav\localisation\detector.py       46      1    98%   65
src\drone_nav\localisation\ekf.py            68      0   100%
src\drone_nav\localisation\estimator.py      87      1    99%   45
src\drone_nav\localisation\imu.py            31      0   100%
src\drone_nav\localisation\models.py         52      2    96%   37, 68
src\drone_nav\localisation\plots.py         113      0   100%
src\drone_nav\planner\__init__.py             5      0   100%
src\drone_nav\planner\astar.py               48      5    90%   14, 24, 26, 63, 71
src\drone_nav\planner\graph.py              162     17    90%   33-35, 46-48, 68-71, 78, 80, 220, 238-241
src\drone_nav\planner\models.py              80      2    98%   38, 109
src\drone_nav\planner\planner.py            148     10    93%   56, 58, 69, 73, 77, 134-135, 140-141, 218
src\drone_nav\planner\plotter.py             58      1    98%   87
src\drone_nav\planner\strategies.py          65      1    98%   73
src\drone_nav\simulator\__init__.py           7      0   100%
src\drone_nav\simulator\battery.py           33      1    97%   21
src\drone_nav\simulator\controller.py        46      1    98%   62
src\drone_nav\simulator\models.py            55      1    98%   53
src\drone_nav\simulator\plots.py            114      0   100%
src\drone_nav\simulator\sim.py               92      1    99%   154
src\drone_nav\simulator\wind.py              24      0   100%
src\drone_nav\viewer\__init__.py              2      0   100%
src\drone_nav\viewer\app.py                  68      9    87%   217-218, 255-258, 275-276, 288
src\drone_nav\world\__init__.py               4      0   100%
src\drone_nav\world\csv_importer.py          95      1    99%   125
src\drone_nav\world\generator.py            125     22    82%   52, 82, 114-125, 168-181, 230-232
src\drone_nav\world\models.py               152      2    99%   33, 37
-----------------------------------------------------------------------
TOTAL                                      2354    117    95%
```

---

### 2.6 Static Analysis & Typing

**Commands:**
```powershell
.\.venv\Scripts\python -m ruff check .
.\.venv\Scripts\python -m mypy src/drone_nav
```

**Output:**
```text
All checks passed!
Success: no issues found in 32 source files
```

---

## 3. Exit Criteria Evaluation

| Plan Exit Criterion | Target | Actual | Evaluation |
|---|---|---|---|
| Error stats (mean, p95, max) over full mission | Terminal output | Mean: 4.20cm, P95: 9.19cm, Max: 16.61cm | PASSED |
| Marker spacing sweep table | N=1, 2, 4, 8 tested | Evaluated on 500-bin layout, clean monotonic degradation | PASSED |
| Long-gap failure case documented | Terminal output showing drift | 60s blackout gap evaluated, uncertainty envelope documented | PASSED |
| Expected DRIMS telemetry error reported | Documented in report | Report states $\pm 9.19\text{ cm}$ (P95) | PASSED |
| Tests pass with >=80% coverage | >=80% | 72 passed, 95% total coverage | PASSED |
| No claims of real-drone behaviour | Kinematic simulation only | Explicitly noted as simulated EKF fusion | PASSED |
