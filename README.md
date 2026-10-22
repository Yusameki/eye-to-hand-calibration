<div align="center">

# 🔧 eye-to-hand-calibration

**An eye-to-hand calibration done with HALCON**

<sub>**English** · [中文](README.zh-CN.md)</sub>

<sub>Photoneo PhoXi 3D scanner + robot arm + calibration sphere</sub>
<sub>HALCON 21.05 · notes from a summer 2022 experiment</sub>

</div>

---

## What this is

A **fixed-mount** 3D scanner (eye-to-hand) needs to understand where the robot arm's end effector is in its view.
The approach: let the robot carry a calibration sphere through 20 different poses in the field of view, record
"robot end-effector pose" + "sphere pose in the point cloud" for each, and hand it to HALCON to solve for the
transform between camera and robot base.

> ( ¯꒳¯ ) In plain words: **introducing the robot to the camera**.

<div align="center">

![capture setup](pic/01.png)

<sub>▲ In the camera's view: robot arm + calibration sphere + calibration board</sub>

</div>

---

## Contents

- [What's in the repo](#whats-in-the-repo)
- [Pipeline](#pipeline)
  - [1. Capture](#1-capture)
  - [2. Modelling](#2-modelling)
  - [3. Calibration](#3-calibration)
  - [4. Verification](#4-verification)
- [Key parameters](#key-parameters)
- [Environment](#environment)
- [Pitfalls](#pitfalls)

---

## What's in the repo

| File / directory | Purpose |
| --- | --- |
| `get_calibration_data.hdev` | Connect to PhoXi, grab point clouds, convert them into calibration-board grayscale images |
| `perform_calibration.hdev` | Main calibration script: build the sphere model, run 20 samples, solve hand-eye |
| `evaluation.hdev` | Verify after calibration: fit a plane + find the inner circle + compute centre error |
| `ball_test.hdev` | The same pipeline for the sphere version, for a different calibration target |
| `create_pose.hdev` | Hand-written pose, purely for poking at numbers |
| `Repose/Diff_Pose_*.dat` | 20 calibration poses (rotation vector + translation, in m) |
| `Repose/Re_Cam2Obj_*.dat` | 20 camera-to-object poses |
| `pointcloud/3d_CalBoarde_*.om3` | 20 calibration-board point clouds (HALCON OM3 format) |
| `BoardSurfaceModel.stl` | 3D model of the calibration board, used for surface-based matching |
| `pic/` | Screenshots of the process; a few are embedded below |
| `PhotoneoLogFiles/` | PhoXi runtime logs, for debugging connection issues |

---

## Pipeline

### 1. Capture

`get_calibration_data.hdev` talks to the scanner. HALCON connects to Photoneo through the `PhoXi`
frame grabber interface, picking the device by serial number or index:

```halcon
AcqName := 'PhoXi'
info_framegrabber (AcqName, 'device', DeviceInfo, DeviceValues)
open_framegrabber (AcqName, 1, 1, 0, 0, 0, 0, 'default', -1, 'default', -1, \
                   'false', 'default', DeviceValues[0], -1, -1, AcqHandle)
set_framegrabber_param (AcqHandle, 'use_grab_image_async_as_trigger_only', 'enable')
grab_image_async (TextureImage, AcqHandle, 0)
```

A few notes:

- **PhoXiControl must be running first**, otherwise `open_framegrabber` fails outright
- Tick the output structure in PhoXiControl (`OutputStructure` → select the fields you need → `Set`);
  by default `PointCloud` and `Texture` are both enabled
- The point cloud comes in **mm**; convert to **m** before feeding it to calibration
- The `Texture` image's contrast needs manual adjustment, or the DataMatrix code on the board won't be readable

<div align="center">

![capture](pic/05.png)
![capture](pic/10.png)

<sub>▲ The arm moves the sphere through poses; each frame is one calibration sample</sub>

</div>

### 2. Modelling

The calibration board needs a 3D model first, so surface-based matching can find it in the point cloud.

```halcon
read_object_model_3d ('BoardSurfaceModel', 'm', [], [], BoardOM3D, Status)
create_surface_model (BoardOM3D, 0.02, [], [], BoardSurfaceModelID)
```

The sphere is simpler — just generate an analytic sphere (radius **27.7 mm**):

```halcon
gen_sphere_object_model_3d ([0,0,0,0,0,0,0], SphereRadius, SphereOM3D)
convex_hull_object_model_3d (SphereOM3D, SphereOM3D)
create_surface_model (SphereOM3D, 0.02, 'model_invert_normals', 'true', SphereSurfaceModelID)
```

That `0.02` is the relative sampling distance: smaller means slower matching, larger means driftier matching.

<div align="center">

![capture](pic/15.png)
![capture](pic/20.png)

<sub>▲ The 20 poses cover most of the workspace</sub>

</div>

### 3. Calibration

The core is two lines:

```halcon
create_calib_data ('hand_eye_stationary_cam', 0, 0, HECCalibDataID)
set_calib_data (HECCalibDataID, 'model', 'general', 'optimization_method', 'nonlinear')
```

- `hand_eye_stationary_cam` = **camera stays fixed** (eye-to-hand).
  If the camera were mounted on the arm, this would be `hand_eye_moving_cam`
- `optimization_method` is `nonlinear`: best pose accuracy, at the cost of speed

Then loop over the 20 samples:

```halcon
NumCalibrationScenes := 20
for I := 1 to NumCalibrationScenes by 1
    read_object_model_3d (filename, 'm', [], [], ObjectModel3D, Status)
    object_model_3d_to_xyz (X, Y, Z, ObjectModel3D, 'from_xyz_map', [], [])
    get_object_model_3d_params (ObjectModel3D, '&intensity', intensity)
    set_grayval (Image, CalBoard_Rows, CalBoard_Columns, intensity)
    ...
endfor
```

A small trick here: **write the point cloud's intensity values into a grayscale image**, mapping row/column
coordinates from the point cloud's x/y. Because `find_data_code_2d` only accepts byte images — a 3D point
cloud can't be fed in directly — and going around this way lets you "photograph" a calibration board out of
the point cloud to read the code.

> 🍮 Some samples fail to match. The script checks for that and **skips unmatched samples entirely**
> rather than including them — forcing them in would skew the result.

<div align="center">

![capture](pic/27.png)

</div>

### 4. Verification

`evaluation.hdev` takes a different route: no calibration sphere, just geometric verification against the board.

1. Draw an ROI by hand and crop out the planar part
   ```halcon
   reduce_object_model_3d_by_view (PlaneROI, ObjectModel3D, CamParam, Pose, ObjectModel3D_Plane)
   fit_primitives_object_model_3d (ObjectModel3D_Plane, 'primitive_type', 'plane', OM3D_Plane)
   get_object_model_3d_params (OM3D_Plane, 'primitive_pose', Cam_H_Plane)
   ```
2. Use the plane pose to straighten the point cloud: `pose_invert`, then `rigid_trans_object_model_3d`
3. Find the inner circle contour on the straightened image and fit its centre with `fit_circle_contour_xld`
4. The difference between the circle's actual coordinates and the calibrated coordinates is the error

```halcon
edges_sub_pix (ImageReduced, Circle0_Edges, 'canny', 2, 20, 60)
select_shape_xld (Circle0_Edges, Circle0_Edges_select, ['contlength','circularity'], 'and', [50,0.6], [500,1])
fit_circle_contour_xld (Circle0_Outer, 'ahuber', -1, 2, 0, 3, 2, \
                        Circle0_Row, Circle0_Column, Circle0_Radius, ...)
```

`'ahuber'` is Huber weighting, fairly tolerant of outliers on the contour — measurably more stable than
plain least squares in practice.

---

## Key parameters

| Parameter | Value | Notes |
| --- | --- | --- |
| `NumCalibrationScenes` | 20 | Number of calibration poses |
| `SphereRadius` | 27.7 | Calibration sphere radius (mm) |
| Optimisation | `nonlinear` | Most accurate poses |
| Calibration type | `hand_eye_stationary_cam` | Fixed camera |
| Camera intrinsics | `gen_cam_par_area_scan_division` | 2064×1544, focal 0.008 / pixel size 5.2e-06 |

The intrinsics aren't identical across the scripts — they were captured in different batches:

```halcon
* version with distortion
gen_cam_par_area_scan_division (0.0078066255, -0.121916, 0.00000345, 0.00000345, \
                                1071.03, 793.9, 2064, 1544, CamParam)
* version without distortion
gen_cam_par_area_scan_division (0.008, 0, 5.2e-06, 5.2e-06, \
                                1071.3, 793.9, 2064, 1544, CamParam)
```

A pose file looks like this (`Repose/Diff_Pose_01.dat`):

```
# 3D POSE PARAMETERS: rotation and translation
f 0
r -0.198842898250703 4.0770661949706 7.42384111526009
t 1.49453245821141 1.25246473791518 -0.428090294941512
```

`r` is a rotation vector (Rodrigues, in deg), `t` is translation (in **m**).
It's the format HALCON's own `write_pose` produces, readable directly by `read_pose`.

---

## Environment

| Item | Version / model |
| --- | --- |
| HALCON | 21.05.0.0 Progress |
| 3D scanner | Photoneo PhoXi (via HALCON's built-in `PhoXi` interface) |
| Robot arm | Any six-axis that can report end-effector poses |
| Calibration target | Flat board with a DataMatrix code + a 27.7mm sphere |

The scripts are `.hdev` in XML format (as saved by HDevelop); open them in HDevelop and run.

---

## Pitfalls

- **PhoXiControl not running** → `open_framegrabber` errors out; check the software is up and the device connected
- **Point cloud units mixed up** → HALCON gives mm, calibration wants m; a factor of 1000 off produces results
  that are wildly wrong but not obviously broken
- **`find_data_code_2d` only takes byte images** → must go through `object_model_3d_to_xyz` + `set_grayval` first
- **Uneven sample quality** → definitely drop samples that fail to match; better to have a few fewer than to
  contaminate the data
- **Texture image contrast** → too low and the DataMatrix code won't read; adjust manually

---

<div align="center">

<sub>A little something from 2022, kept for the record (´-ω-`)</sub>

</div>
