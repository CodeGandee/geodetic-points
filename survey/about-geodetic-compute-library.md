# Guide: Geodetic computation libraries for ROS 2 VIO+GPS pipelines

Context
- This guide supports the data pipeline defined in [ros2-setup.md](ros2-setup.md): time-sync VIO and GPS, estimate extrinsics to an Earth-fixed frame (ECEF), and convert/publish VIO points in global geodetic coordinates.
- Focus: widely used, reliable libraries for ECEF/LLA/ENU conversions and geodesic calculations, for both Python and C++.

Scope of tasks (from the pipeline)
- Convert geodetic WGS84 (lat, lon, alt) to ECEF and vice versa.
- Build local tangent frames (e.g., ENU) anchored at a reference LLA or ECEF point.
- Perform geodesic calculations on the ellipsoid (distance, azimuths, waypoints).
- Handle CRS transforms and datum shifts correctly when needed.

Selection criteria
- Accuracy on WGS84 ellipsoid (not spherical approximations).
- Maintained, well-documented, and widely used in geospatial/robotics.
- Availability in Python and C++.
- Proven integration in the ROS ecosystem where possible.

Recommended libraries overview
- Python:
  - pyproj (PROJ bindings): mature CRS transforms, geodesics (via Geod), geocentric & geographic conversions, datum handling.
  - GeographicLib (Python): gold-standard geodesic routines (direct/inverse, waypoints, areas).
  - pymap3d: lightweight ECEF/ENU/NED utilities commonly used in robotics; friendly API.
  - GDAL/OSR: heavy-duty geospatial stack; includes CRS transforms; installation can be complex.
- C/C++:
  - GeographicLib (C++): Geodesic, Geocentric (ECEF↔LLA), and LocalCartesian (ENU) classes; widely used in robotics.
  - PROJ (C/C++ API): generic CRS transforms, Helmert/Molodensky, datum grids.

Notes on ROS ecosystem usage
- MAVROS extras references ECEF↔ENU transforms and “use GeographicLib possibilities” in its changelog, indicating production usage paths for Earth-fixed conversions and geodesy in ROS workflows.
  - See: docs.ros.org links in References.

Details and references

1) pyproj (Python interface to PROJ)
- What it’s good for
  - Accurate CRS transformations (e.g., ECEF↔WGS84↔projected), datum shifts, and access to the PROJ transformation pipeline.
  - Geodesic calculations (distance, azimuths, polygon area) via pyproj.Geod.
  - Straightforward ECEF↔LLA conversion using Transformer with geocent/geographic CRSs.
- Why it fits
  - Broad CRS coverage, maintained by the PROJ project community, integrates well with geospatial data workflows, and is robust on Windows/Linux/macOS.
- Representative docs and examples
  - PROJ home: https://proj.org/
  - Pyproj documentation (dev snapshots surfaced via Context7): 
    - Geodesic usage examples and API (Geod): https://pyproj4.github.io/pyproj/dev/api/aoi.html/api/geod
    - Coordinate transforms, datum shift gotchas: https://pyproj4.github.io/pyproj/dev/api/aoi.html/_sources/gotchas.rst
    - Geocentric→geographic example with Transformer: https://pyproj4.github.io/pyproj/dev/api/aoi.html/_modules/pyproj/transformer
- Typical usage in this project
  - LLA→ECEF and ECEF→LLA conversions for point clouds and GPS fixes.
  - ENU frame construction with an anchor (via composed transforms or local cartesian approximations).
  - Geodesic distances (for sanity checks, e.g., RMS metrics along tracks).

2) GeographicLib (Python)
- What it’s good for
  - High-accuracy geodesic routines on the WGS84 ellipsoid: direct/inverse problems, waypoints, areas, azimuths, reduced lengths, scales.
  - Authoritative and widely cited in geodesy; excellent numerical stability.
- Why it fits
  - When you need robust geodesic calculations (e.g., lengths between LLA points, azimuths), this is a top choice.
- Representative docs and examples
  - GeographicLib (Python) docs: https://geographiclib.sourceforge.io/html/python/
  - Examples – inverse/direct/waypoints: 
    - https://github.com/geographiclib/geographiclib-python/blob/main/doc/examples.rst#_snippet_1
    - https://github.com/geographiclib/geographiclib-python/blob/main/doc/examples.rst#_snippet_2
    - Waypoints (distance-based/arc-based): 
      https://github.com/geographiclib/geographiclib-python/blob/main/doc/examples.rst#_snippet_4
      https://github.com/geographiclib/geographiclib-python/blob/main/doc/examples.rst#_snippet_5

3) pymap3d (Python)
- What it’s good for
  - Simple, robotics-friendly ECEF/LLA/ENU/NED conversion utilities (vectorized options), common in aerospace/robotics scripts.
- Why it fits
  - Lightweight and easy to use for everyday coordinate conversions without full CRS/datum machinery, suitable where WGS84 is assumed.
- Representative docs
  - Project docs: https://pymap3d.readthedocs.io/
  - Common functions: geodetic2ecef, ecef2geodetic, geodetic2enu, enu2geodetic, ecef2enu, etc.

4) GDAL/OSR (C/C++/Python)
- What it’s good for
  - Comprehensive geospatial toolkit: raster/vector IO, advanced CRS/datum transforms (via PROJ under the hood), metadata, and more.
- When to use
  - If you already depend on GDAL for broader geospatial processing, you can leverage OSR for CRS transforms.
- Caveats
  - Installation can be heavy; simple coordinate math might be faster with pyproj/pymap3d.
- Representative docs
  - GDAL site: https://gdal.org/
  - Past releases (Windows users often install via wheels): https://gdal.org/en/stable/download_past.html

5) GeographicLib (C/C++)
- What it’s good for
  - Core classes for robotics-grade Earth geometry:
    - Geodesic: inverse/direct geodesic problems.
    - Geocentric: precise ECEF↔LLA conversion.
    - LocalCartesian: construction of local tangent ENU frames about an LLA/ECEF anchor.
- Why it fits
  - Standard C++ library used throughout robotics/avionics; numerically robust and thoroughly validated.
- Representative docs
  - GeographicLib C implementation docs (algorithms shared with C++/Python): https://geographiclib.sourceforge.io/html/C/index.html
  - Project docs hub (Python shown; C++ documentation is closely related): https://geographiclib.sourceforge.io/html/python/
  - JavaScript variants (for reference/tutorials): https://geographiclib.sourceforge.io/html/js/

6) PROJ (C/C++)
- What it’s good for
  - Generic CRS transforms, including datum shifts, Helmert/Molodensky transformations, compound operations and grids.
- Why it fits
  - When you need rigorous CRS/epoch transformations beyond simple ECEF/LLA math, or alignment with national geodetic realizations.
- Representative docs
  - PROJ home: https://proj.org/en/stable/index.html
  - Using PROJ: https://proj.org/en/stable/usage/index.html
  - Transformations overview: https://proj.org/en/stable/operations/transformations/index.html
  - Helmert transform: https://proj.org/en/stable/operations/transformations/helmert.html
  - Molodensky–Badekas: https://proj.org/en/stable/operations/transformations/molobadekas.html
  - Tutorials: https://proj.org/en/stable/tutorials/index.html

ROS ecosystem notes and examples
- MAVROS extras: ECEF↔ENU transform fixes and references to using GeographicLib capabilities (indicative of ROS production usage of these geodetic components).
  - Rolling changelog snippet (ENU↔ECEF, GeographicLib): https://docs.ros.org/en/rolling/p/mavros_extras/__CHANGELOG.html
  - Kilted changelog snippet (same note): https://docs.ros.org/en/kilted/p/mavros_extras/__CHANGELOG.html
- nmea_navsat_driver (standards page): https://docs.ros.org/en/ros2_packages/jazzy/api/nmea_navsat_driver/__standards.html
  - While not a library itself, it sits in the GPS ingestion path and often coexists with the above.

Practical recommendations for this project
- Conversions (LLA↔ECEF↔ENU):
  - Python: prefer pyproj for robust CRS handling or pymap3d for lightweight WGS84-only flows.
  - C++: use GeographicLib Geocentric and LocalCartesian for deterministic Earth-fixed math.
- Geodesic calculations (distance/azimuth/waypoints/areas):
  - Prefer GeographicLib (Python or C++) for authoritative geodesic results.
  - pyproj.Geod is also suitable and PROJ-aligned; choose based on existing deps.
- Datum/CRS considerations:
  - If the application requires national datums or epoch-aware frame transforms, use PROJ (and ensure the proper proj-data package is installed).
- Performance and deployment:
  - For robotics nodes, keep dependencies lean where possible (pymap3d or GeographicLib alone can be enough if you stay in WGS84).
  - For analytics or map integration workflows, pyproj/PROJ is often necessary.
- Windows install notes:
  - Consider prebuilt wheels (e.g., GDAL wheels on Windows) if you must use GDAL; otherwise favor pyproj/pymap3d/GeographicLib for simpler installs.

Cross-check with [ros2-setup.md](ros2-setup.md)
- Time sync: independent of geodesy, but use ENU/ECEF conversions from the chosen library to compare GPS with VIO in a consistent frame.
- Extrinsics: once calibrated to an Earth-fixed frame (ECEF) or local ENU, use the same library consistently across calibration and publishing.
- Convert/publish points:
  - Use pyproj/pymap3d/GeographicLib to convert VIO points to ECEF and optionally to geodetic (lat, lon, alt) for visualization or NavSatFix publication.

References (curated)
- PROJ docs
  - Home: https://proj.org/
  - Using PROJ: https://proj.org/en/stable/usage/index.html
  - Transformations: https://proj.org/en/stable/operations/transformations/index.html
  - Helmert: https://proj.org/en/stable/operations/transformations/helmert.html
  - Molodensky–Badekas: https://proj.org/en/stable/operations/transformations/molobadekas.html
  - Tutorials: https://proj.org/en/stable/tutorials/index.html
- Pyproj (docs surfaced via Context7)
  - Geodesic API and examples: https://pyproj4.github.io/pyproj/dev/api/aoi.html/api/geod
  - Gotchas/datum shifts: https://pyproj4.github.io/pyproj/dev/api/aoi.html/_sources/gotchas.rst
  - Geocentric→geographic example: https://pyproj4.github.io/pyproj/dev/api/aoi.html/_modules/pyproj/transformer
- GeographicLib
  - Python docs: https://geographiclib.sourceforge.io/html/python/
  - C library docs (algorithms shared across impls): https://geographiclib.sourceforge.io/html/C/index.html
  - JS tutorials (for conceptual walk-throughs): https://geographiclib.sourceforge.io/html/js/tutorial-3-examples.html
- pymap3d
  - Docs: https://pymap3d.readthedocs.io/
- GDAL
  - Home: https://gdal.org/
  - Past releases: https://gdal.org/en/stable/download_past.html
- ROS ecosystem examples
  - MAVROS extras changelogs referencing ECEF↔ENU and GeographicLib: 
    - Rolling: https://docs.ros.org/en/rolling/p/mavros_extras/__CHANGELOG.html
    - Kilted: https://docs.ros.org/en/kilted/p/mavros_extras/__CHANGELOG.html

File
- You are reading: [about-geodetic-compute-library.md](gps-points/about-geodetic-compute-library.md)