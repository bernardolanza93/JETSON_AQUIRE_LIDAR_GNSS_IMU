# Pairwise Point‑Cloud Fusion (Double Kinect) — Handover Documentation

> **Scope.** This document explains how the current codebase performs **non‑incremental, pairwise fusion** of point clouds acquired by a linear vehicle inside a row (filare). It is written for a developer taking over the project. Focus is on pipeline logic, math-y parts, I/O conventions, and the main script **`double_kinect_processing.py`**. Supporting modules are also documented.

---

## TL;DR — How the pipeline fits together

1. **Preprocess MKV → PLY** (one folder per camera) with `PREPROC_double_KA.process_mkv(...)`. It reads an Azure Kinect `.mkv`, filters bad points, and saves `pointcloud_<SystemTimestamp>.ply`. fileciteturn1file1 fileciteturn1file6 fileciteturn1file16  
2. **Couple the two cameras per timestamp** with `transform_all_lower_pc_plus_icp_registration(...)`: rigidly transform lower→upper Kinect, optionally ICP-refine, **save fused pairs** to a third folder (`coupled_saving_folder`) as `pointcloud_<timestamp>.ply`. **Units here are millimeters.** fileciteturn1file7  
3. **Load the coupled PLYs** (optionally crop ground) and **convert to meters** for SLAM/ICP stages. fileciteturn1file10  
4. **GNSS/IMU trajectory** → resample to SLAM timestamps using `LOCALIZE.resample_data_to_slam_frequency(...)`. (Includes axis remapping.) fileciteturn1file8  
5. **Hierarchical pairwise fusion** along time with `hierarchy_slam_icp(...)`: per pair, use prior transform, run filtered ICP, optionally do **GNSS/IMU + ICP hybrid** transform (`correct_slam_with_gnssimu(...)`), accumulate, post-filter points, and merge. Output: final fused cloud + per‑frame delta logs. fileciteturn1file4 fileciteturn1file18 fileciteturn1file17

---

## Repository at a glance

- **`double_kinect_processing.py`** — main pipeline (loading, pairing, ICP, GNSS/IMU correction, hierarchical fusion, saving/plots). fileciteturn1file11  
- **`PREPROC_double_KA.py`** — MKV extractor (Azure Kinect), timestamp alignment, PLY writer, basic outlier removal. fileciteturn1file6  
- **`localization_proc.py`** — GNSS/IMU processing, interpolation/resampling to SLAM timestamps, axis transforms and basic orientation synthesis. fileciteturn1file8 fileciteturn1file12 fileciteturn1file14  
- **`filter_3D_processing.py`** — point‑cloud filters (downsampling, redundancy, isolation, radius crop). fileciteturn0file3  
- **`rendering_functions.py`** — small helpers to visualize clouds (axes, viewer). fileciteturn0file5  
- **`imu_processor.py`** — IMU trajectory toy script (demo between GNSS points). Not used by the main pipeline. fileciteturn1file19

> External dependencies referenced (not included here): `external_lis`, `custom_slam_doc`, `plotter`, `matrix_utilities`, `decorator`, Open3D, SciPy, NumPy, Matplotlib, `pyk4a`.

---

## Data & I/O conventions

### File naming
- Point clouds are saved as **`pointcloud_<SystemTimestamp>.ply`**. Timestamps are integers (system time) or floats. Matching across sensors is by identical `<SystemTimestamp>`. fileciteturn1file7 fileciteturn1file5

### Coordinate frames & units
- **Preprocessing & Coupling:** raw Azure Kinect depth points are in **millimeters**; the rigid offset between Kinects is set in **mm** (`+853 mm` on X, `+6 mm` on Z). ICP thresholds there assume mm. fileciteturn1file3  
- **SLAM/ICP stage:** clouds are **converted to meters** via `convert_to_meters(...)`. Subsequent distances/voxel sizes are meters. fileciteturn1file7  
- **GNSS/IMU resampling:** axis remapping is applied to align GNSS/IMU with SLAM axes (positions `[z, x, -y]`, rotations `[z, x, -y]`), and timestamps are converted to ns to match SLAM. fileciteturn1file8

### Directory structure (suggestion)
```
data/
  kinect1/pointcloud_*.ply
  kinect2/pointcloud_*.ply
  coupled/pointcloud_*.ply     # output of pairing (per-timestamp fusion)
output/
  fused_<YYYY-mm-dd_HH-MM-SS>.ply
```

---

## Main script: `double_kinect_processing.py`

### Key switches (set near the bottom)
- `SHOW_RGB_MKV`, `EXTRACT_RAW_PCS`, `COUPLE_RAW_PC` control whether to preview MKV, extract raw PLYs, or pair cameras. fileciteturn1file3  
- `initial_trasform_fixed_high_camera` — rigid transform **lower→upper** camera (**mm**): `x=+853`, `z=+6`. Applied before ICP during pairing. fileciteturn1file3  
- `DownSampling_constant = 0.16` — typical voxel size (meters) for SLAM‑stage downsampling. fileciteturn1file10  
- Region limits (`starts`, `ends`) select a time window by **index** within the sorted coupled files. A ground crop (x ≤ 1900 mm) is applied before meter conversion. fileciteturn1file10

### Timestamp utilities
- `get_sorted_filenames_by_timestamp(dir)` returns PLY filenames sorted by the trailing `_timestamp`. fileciteturn1file11  
- `get_pointcloud_timestamps`, `find_closest_timestamp`, `compare_directories` help diagnose mismatches between two folders (plots & logs). fileciteturn0file0

### Coupling two Kinects (per‑timestamp)
- `transform_all_lower_pc_plus_icp_registration(folder1, folder2, initial_T, out_folder)`  
  For each timestamp:
  1) read source (upper) & target (lower),  
  2) apply `initial_T` to lower,  
  3) run ICP (`icp_open3d_coupling`) to correct micro‑vibrations,  
  4) save `source + transformed(lower)` into `out_folder/pointcloud_<ts>.ply`. (Still **mm**.) fileciteturn1file7  
- `icp_open3d_coupling(source, target, LIMIT_TRASFORMATION=0, max_iterations=250, threshold=0.05)`  
  Point‑to‑point ICP with validity checks on rotation matrix, **angle clamp**, and optional thresholds (`rotation_threshold≈0.1 rad`, `translation_threshold=5`). If invalid, returns identity. fileciteturn1file11

### GNSS/IMU → SLAM resampling
- `LOCALIZE.resample_data_to_slam_frequency(slam_timestamps, interpolated_data)`  
  Converts GNSS timestamps from seconds→nanoseconds, remaps axes, upsamples/downsamps to match SLAM rate (linear interpolation by default if GNSS slower). Output: dict keyed by SLAM timestamps with `position{xyz}` and `rotation{xyz}` (radians). fileciteturn1file8

### Hybrid transform (ICP ⊕ GNSS/IMU)
- `correct_slam_with_gnssimu(prev_ts, downsampled_data, cur_ts, T_icp, w_icp, w_gnss, no_pc=False)`  
  Produces a **local** SE(3) increment between frames by mixing ICP and GNSS/IMU: quaternion blend for rotation; translation as weighted blend with **safety clamps** (if ICP Δ exceeds thresholds, weight→GNSS). Returns `(T_combined, global_traj_contribution, local_translation)`. fileciteturn0file0

### Hierarchical pairwise fusion (core)
- `hierarchy_slam_icp(pointclouds, timestamp_sorted, downsampled_data, DS_const, w_icp=0)`  
  Iteratively fuses clouds **two at a time** (PC[0]+PC[1] → new target; then with PC[2], etc.), each time:  
  1) prior transform from last round → current source,  
  2) ICP on **filtered** subsets (see below),  
  3) optional **GNSS/IMU hybrid** step (set `w_icp ∈ [0,1]`; `w_gnss=1−w_icp`),  
  4) accumulate transforms, merge, and **post‑filter**.  
  Saves per‑timestamp logs of both ICP deltas and GNSS/IMU deltas (`trajectory_deltas[...]`). fileciteturn1file4 fileciteturn1file18 fileciteturn1file17

**Pre/Post‑filters used inside fusion**
- **Radius crop around mid‑centroid** for ICP: `filter_pointclouds_by_central_radius(source, target, radius=4.0, bound_threshold=5.0)` trims to a central ball if clouds are too large (speeds up & stabilizes ICP). fileciteturn1file2  
- **Voxel downsample**: `filter_3D_processing.downsample_point_cloud(pc, DS_const)` (meters). fileciteturn0file3  
- **Redundancy removal** (≥ min distance) and **radius outlier** cleanup (Open3D): applied to the merged cloud (typical params: `min_distance=0.005 m`, `nb_neighbors≈12`, `radius≈0.4 m`). fileciteturn0file3 fileciteturn1file17

**GNSS antenna ↔ Kinect offset**
- After the **first epoch**, a fixed translation (example shown: `+0.634 m` on Z) is added to the merged cloud to compensate for the GNSS→sensor lever arm. Tune to your rig. fileciteturn1file17

**Logged outputs per timestamp (example)**
```json
trajectory_deltas[<timestamp>] = {
  "slam_icp_transformation": {
    "delta_x": ...,
    "delta_y": ...,
    "delta_z": ...,
    "delta_rotation_x": ...,  // from ICP rotation matrix
    "delta_rotation_y": ...,
    "delta_rotation_z": ...,
    "rotation_matrix": [[...],[...],[...]]
  },
  "gnss_imu_transformation": {
    "delta_x": ...,
    "delta_y": ...,
    "delta_z": ...,
    "delta_rotation_x": ...,
    "delta_rotation_y": ...,
    "delta_rotation_z": ...
  }
}
```
fileciteturn1file18

---

## Supporting modules

### `PREPROC_double_KA.py`
- **`process_mkv(input_file, output_dir, start_index, timestamp_file, end_index)`** — opens Azure Kinect `.mkv`, maps **Kinect timestamps → System timestamps** via CSV, reconstructs colored cloud (`depth_point_cloud` + transformed color), removes invalid/near‑zero points, optional isolation removal, writes `pointcloud_<SystemTimestamp>.ply`. Also reports mean #points rejected. fileciteturn1file1 fileciteturn1file6 fileciteturn1file16  
- **`resize_and_rotate(image, scale_percent)`**, **`show_rgb_video_mkv(mkv_file)`** — quick RGB viewer to inspect captures. fileciteturn1file6  
- **`remove_isolated_points(pcd, nb_neighbors=10, radius=80)`** — radius outlier removal in **mm** context (preprocess stage). fileciteturn1file6

### `localization_proc.py`
- **`resample_data_to_slam_frequency(slam_ts, interpolated_data)`** — convert seconds→ns, **axis remap** (positions `[z, x, -y]`, rotations `[z, x, -y]`), then up/down‑sample GNSS/IMU to SLAM rate via interpolation. Returns dict keyed by SLAM timestamps. fileciteturn1file8  
- **`generate_gnss_based_orientation_with_imu(...)`** — filter GNSS positions (moving average), interpolate IMU Euler onto GNSS timeline, normalize/start at zero (configurable sign flip on yaw), and optionally visualize. Output is a dict with synced `rotation` and `position` per timestamp. fileciteturn1file13  
- **`generate_gnss_based_orientation(...)`** — derive a plausible orientation **from GNSS tangents** (for when IMU is missing), then smooth. fileciteturn0file1  
- **`downsample_interpolated_data_to_slam(...)`** — like `resample_...` but keeps only the overlap window and uses linear interpolation; option to normalize origin commented out. fileciteturn1file12  
- **`align_gnss_trajectory(...)`** — rotate GNSS so the initial motion aligns with +X (Z‑axis rotation only). Useful to normalize heading. fileciteturn1file14  
- Diagnostics: `plot_gnss_data`, `interpolate_gnss_for_imu` (includes frequency printouts & trajectory plots). fileciteturn1file9 fileciteturn1file15

### `filter_3D_processing.py`
- **`downsample_point_cloud(pc, voxel_size)`** — Open3D voxel grid + duplicate removal. (Meters.) fileciteturn0file3  
- **`remove_redundant_points(pc, min_distance=0.001)`** — keep only points farther than `min_distance` from their nearest neighbor (preserves colors). (Meters.) fileciteturn0file3  
- **`remove_isolated_points(pcd, nb_neighbors=5, radius=0.5)`** — radius outlier cleanup. (Meters.) fileciteturn0file3  
- **`filter_points_within_radius(pointcloud, center, radius)`** — crop to a sphere (used by central‑radius tendering). (Meters.) fileciteturn0file3

### `rendering_functions.py`
- **`visualize_pc(pc, title)`** with colored axes via `create_axes(length=1.0)`. Handy for manual inspections. fileciteturn0file5

### `imu_processor.py` (demo only)
- Shows how to integrate linear accelerations (after orientation correction) between two GNSS fixes; scales to the GNSS baseline and translates to start at P1. Useful as a reference, not used by the main script. fileciteturn1file19

---

## Important implementation details & knobs

- **Angle wrapping**: helper `delta_angle` ensures continuous deltas in \[-π, π]. (Rotations are **radians** across the project.) fileciteturn0file0  
- **ICP validity**: rotation matrix orthonormality + `det(R)≈1` check; angle computed from trace with clamped `arccos`; optional limits (`rotation_threshold≈0.1`, `translation_threshold=5`). If invalid/over‑limit ⇒ identity. fileciteturn1file11  
- **Central‑radius tendering**: before ICP, if a cloud’s bounding box exceeds `bound_threshold` (m), both source and target are cropped to a ball of radius `radius` (m) around the mid‑centroid to stabilize matches. Logs how many points are removed. fileciteturn1file2  
- **Hybrid weighting**: if ICP translation components exceed per‑axis thresholds, the code auto‑reduces `w_icp` (→ ~0) making GNSS dominate. fileciteturn0file0  
- **GNSS lever arm**: fixed translation (example `+0.634 m` on Z) applied once at epoch 1 to account for sensor offset from antenna. Adjust to your platform. fileciteturn1file17

---

## Typical run (from `double_kinect_processing.py`)

1) **Extract PLYs** from both Kinects (optional if you already have them):
```python
EXTRACT_RAW_PCS = 1
KA_PREPROC.process_mkv(input_file_mkv_1, output_folder_pc_1, start_index_1, timestamp_conversion_file_1, end_index_1)
KA_PREPROC.process_mkv(input_file_mkv_2, output_folder_pc_2, start_index_2, timestamp_conversion_file_2, end_index_2)
```
fileciteturn1file3

2) **Pair by timestamp** and fuse lower→upper (rigid + ICP), saving to `coupled_saving_folder`:
```python
COUPLE_RAW_PC = 1
initial_trasform_fixed_high_camera = np.eye(4)
initial_trasform_fixed_high_camera[0, 3] = 853  # mm
initial_trasform_fixed_high_camera[2, 3] = 6    # mm
transform_all_lower_pc_plus_icp_registration(output_folder_pc_1, output_folder_pc_2,
                                             initial_trasform_fixed_high_camera, coupled_saving_folder)
```
fileciteturn1file3

3) **Load coupled PLYs**, crop ground, convert to meters, and collect timestamps:
```python
pointcloud_files = sorted(os.listdir(coupled_saving_folder))
# select [starts:ends] by index
pcd_raw = o3d.io.read_point_cloud(os.path.join(coupled_saving_folder, file_name))
mask = np.asarray(pcd_raw.points)[:, 0] <= 1900  # mm (crop ground)
pcd_raw.points = o3d.utility.Vector3dVector(np.asarray(pcd_raw.points)[mask])
pcd_raw.colors = o3d.utility.Vector3dVector(np.asarray(pcd_raw.colors)[mask])
pcd_m = convert_to_meters(pcd_raw)                # → meters
```
fileciteturn1file10 fileciteturn1file7

4) **Resample GNSS/IMU to SLAM timestamps** and fuse hierarchically:
```python
downsampled_gnss_imu = LOCALIZE.resample_data_to_slam_frequency(timestamp_sorted, interpolated_data)
fused, trajectory_deltas = hierarchy_slam_icp(pointclouds, timestamp_sorted,
                                              downsampled_gnss_imu, DownSampling_constant, w_icp=0.5)
# Save cloud
o3d.io.write_point_cloud(f"output_double/fused_{datetime.now():%Y-%m-%d_%H-%M-%S}.ply", fused)
```
fileciteturn1file10 fileciteturn1file4

---

## FAQs / gotchas

- **Why some thresholds look tiny?** Because the coupling stage runs in **mm**, later stages in **m**. Check units before tuning. fileciteturn1file3 fileciteturn1file7  
- **My ICP explodes on curves:** enable central‑radius tendering and increase `radius` a bit; downsample more (larger voxel). fileciteturn1file2 fileciteturn0file3  
- **GNSS dominates too much:** raise `w_icp` (e.g., 0.7) **unless** the per‑axis translation checks are triggered (then code may clamp ICP weight). fileciteturn0file0  
- **Angles jump ±π:** use `delta_angle(...)` for continuous deltas. All rotations are in **radians**. fileciteturn0file0

---

## Glossary (selected variables)

- `initial_trasform_fixed_high_camera` — lower→upper Kinect rigid offset (mm). fileciteturn1file3  
- `DownSampling_constant` — voxel size (m) for SLAM‑stage downsampling. fileciteturn1file10  
- `w_icp`, `w_gnss` — hybrid weights in `correct_slam_with_gnssimu`. (Auto‑reduced if ICP Δ too large.) fileciteturn0file0  
- `GNSS_SOLO` — forces GNSS‑only transform if set; otherwise hybrid/ICP. fileciteturn1file4  
- `trajectory_deltas` — per‑frame log of ICP and GNSS/IMU increments (used for plotting). fileciteturn1file18

---

## What you can safely refactor

- Encapsulate **unit conversions** (mm↔m) and ensure ICP thresholds are scaled accordingly.  
- Centralize **axis remapping** (GNSS→SLAM) to a single helper and reuse. fileciteturn1file8  
- Move magic numbers (e.g., ground crop `x≤1900 mm`, lever arm `0.634 m`) to a config file.

---

*End of document.*
