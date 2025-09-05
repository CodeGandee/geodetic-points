# Time Synchronization Topics Explanation

## Background

The GPS-VIO calibration node estimates an affine time model to align GPS and VIO timestamps:
```
t_vio_corrected = a * t_gps + b
```

Where:
- `a` = drift factor (clock rate ratio)
- `b` = offset (initial time difference)

## Redesigned Topics

### 1. `/sync/time_difference` (Float64)
- **What it publishes**: Current time difference between VIO and GPS in seconds
- **Formula**: `time_diff = t_vio_corrected - t_gps = (a-1) * t_gps + b`
- **Interpretation**: Positive value means VIO clock is ahead of GPS clock
- **Example**: `0.125` means VIO timestamp is 125ms ahead of GPS

### 2. `/sync/clock_drift_rate` (Float64)
- **What it publishes**: Clock drift rate in parts per million (ppm)
- **Formula**: `drift_ppm = 1,000,000 * (a - 1.0)`
- **Interpretation**: 
  - Positive: VIO clock running faster than GPS
  - Negative: VIO clock running slower than GPS
- **Example**: `50.0` means VIO clock gains 50 microseconds per second

### 3. `/sync/time_model` (Float64MultiArray)
- **What it publishes**: Raw model parameters `[a, b]`
- **Purpose**: For advanced debugging and analysis
- **Example**: `[1.00005, 0.125]` means 50ppm drift and 125ms offset

### 4. `/sync/residual_error` (Float64)
- **What it publishes**: Spatial alignment error after time correction (meters)
- **Purpose**: Quality metric for time synchronization
- **Example**: `0.05` means 5cm residual error after time alignment

## Why This Design?

The old design published redundant model parameters:
- `/calibration/time_offset` → `b`
- `/calibration/time_drift` → `a`
- `/sync/time_model` → `[a, b]`

The new design publishes intuitive values:
- **Time difference**: What users actually want to know
- **Drift rate in ppm**: Industry-standard unit for clock drift
- **Model parameters**: Only for advanced users

## Usage Examples

### Monitor Time Synchronization
```bash
# Watch actual time difference
ros2 topic echo /sync/time_difference

# Monitor clock drift
ros2 topic echo /sync/clock_drift_rate

# Check sync quality
ros2 topic echo /sync/residual_error
```

### Typical Values
- **Good sync**: time_difference < 0.1s, drift < 100ppm, error < 0.1m
- **Poor sync**: time_difference > 1.0s, drift > 1000ppm, error > 1.0m

### Interpreting Results
```
time_difference: 0.050  # VIO 50ms ahead of GPS
clock_drift_rate: 25.0  # VIO gaining 25μs per second
residual_error: 0.03    # 3cm alignment error
```

This indicates good synchronization with small, stable time offset.