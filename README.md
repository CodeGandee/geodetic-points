# geodetic-points

Sketch / architectural scaffold for a future Visual-Inertial Odometry (VIO) + GPS aided navigation and geodetic point localization system built around ROS2. **No production code yet – this repository currently serves as a planning space.**

## Vision
Provide a modular ROS2-based pipeline that fuses VIO, IMU, and multi-constellation GNSS to (1) estimate platform pose in a globally referenced frame, (2) geo-register observed 3D feature / object points, and (3) allow annotating & exporting these points ("objects of interest") to mapping platforms (e.g., Google Maps / KML / GeoJSON).

## Core Objectives (Future)
1. Sensor Fusion: Tight / loosely coupled integration of VIO and GNSS with robust time synchronization.
2. Global Pose Estimation: Maintain an ENU + WGS84 representation with confidence / covariance.
3. 3D Point Observation: Collect triangulated landmarks (from SLAM or depth) and transform them into geodetic coordinates (latitude, longitude, ellipsoidal height).
4. Object Tagging: Attach semantic labels + attributes to selected points.
5. Map Export: Produce GeoJSON / KML / CSV deliverables with metadata & uncertainty.
6. Offline + Live Modes: Support post-processing refinement (smoothing) and real-time streaming.

## High-Level Architecture (Planned)
```
Sensors (Camera(s), IMU, GNSS Receiver, (Optional: Wheel Odometry))
	│
	├── Timestamp + Frame Sync Layer
	│
	├── VIO Frontend (feature tracking / keyframe selection)
	│
	├── VIO Backend (optimization / factor graph)
	│         │
	│         └── Local map of landmarks (camera frame / body frame)
	│
	├── GNSS Processing (RTK / SBAS / raw carrier-phase ingestion) → ECEF / WGS84
	│
	├── Fusion Node (pose graph / EKF / factor graph w/ GNSS priors) → Global Pose (ENU + WGS84)
	│
	├── Landmark Geo-Projector (body/camera frame → ENU → WGS84 lat/lon/h)
	│
	├── Semantic Annotation Interface (CLI / minimal UI / API)
	│
	└── Exporters (GeoJSON / KML / CSV / custom)
```

## Data Frames & Transforms (Conceptual)
- `map`: Global ENU origin (anchored to first high-quality GNSS fix or configured datum point).
- `odom`: Continuous local frame for VIO integration (may reset / drift; periodically aligned to `map`).
- `base_link`: Vehicle / body frame.
- `camera_<i>`: Individual optical frames.
- `imu_link`: IMU sensor frame.

Planned transforms: `camera_i -> base_link`, `imu_link -> base_link`, `odom -> map` (with GNSS corrections), dynamic `base_link` pose in both `odom` and `map`.

## Geodetic Conversion Outline
1. Maintain fused pose in ENU relative to a chosen WGS84 reference (lat0, lon0, h0).
2. Convert ENU to ECEF (standard rotation + translation).
3. Convert ECEF to geodetic (iterative or closed-form algorithm).
4. Attach covariance via Jacobian propagation from local pose & landmark uncertainty.

## Anticipated ROS2 Packages (Names Tentative)
- `geo_vio_msgs`: Custom message definitions (geodetic point, annotated object, uncertainty).
- `vio_core`: VIO frontend/backend wrapper (could wrap existing systems like VINS-Fusion, OKVIS, Kimera-VIO, ORB-SLAM3 plugin style).
- `gnss_proc`: GNSS parsing + optional RTK client (NTRIP) and conversion utilities.
- `fusion_layer`: Factor graph / EKF that fuses VIO + GNSS + IMU biases.
- `geodetic_projection`: ENU/ECEF/WGS84 transformations + covariance propagation.
- `point_annotation`: Minimal interface (service / topic) for tagging observed points.
- `export_tools`: GeoJSON / KML / CSV generation.
- `demo_launch`: Example launch files tying the stack together.

## Minimal Roadmap (Draft)
| Phase | Focus | Output |
|-------|-------|--------|
| 0 | Repo scaffolding | README, LICENSE (MIT) |
| 1 | Message & data model | `.msg` & `.srv` sketches |
| 2 | Geodetic utilities | Standalone conversion funcs + tests |
| 3 | GNSS ingestion | Parse NMEA / RINEX / RTCM minimal subset |
| 4 | VIO integration | Adapter to external VIO library |
| 5 | Fusion prototype | Pose alignment & drift correction |
| 6 | Landmark geo-projection | Landmark → WGS84 w/ covariance |
| 7 | Annotation + export | CLI / API + GeoJSON/KML outputs |
| 8 | Packaging & docs | Launch examples, diagrams |

## Non-Goals (For Now)
- Full custom VIO implementation (will integrate existing).
- Advanced multi-sensor SLAM (LiDAR, radar) beyond placeholders.
- High-end real-time RTK network management.
- GUI-heavy tooling (keep early interface lightweight / scriptable).

## References / Inspirations (To Curate Later)
- WGS84 & ECEF conversion formulas.
- ROS REP 105 (Coordinate Frames) & REP 103 (Units).
- Existing open-source: VINS-Fusion, ORB-SLAM3, Nav2 GNSS integration docs.

## License
MIT – see `LICENSE` file.

## Status
Planning only. No functional code merged yet.

---
Feel free to open issues with suggestions on architecture or geodetic handling approaches before implementation starts.
