# Ocam Camera Model Support in Kalibr

This document describes the Ocam (Omnidirectional Camera) model support added to Kalibr.

## Overview

The Ocam model uses polynomial mapping for omnidirectional cameras, based on the CPAC (Camera Polynomial Ambient Calculator) model. It is particularly suitable for fisheye cameras with large field of view (>180 degrees).

## Mathematical Model

### Forward Projection (3D → 2D)

```
theta = atan(-z / sqrt(x^2 + y^2))
rho = a₀ + a₁*theta + a₂*theta² + ... + aₙ*thetaⁿ  (polynomial)
u = x / norm * rho
v = y / norm * rho
[u_pixel, v_pixel] = AffineTransform(u, v) + PrincipalPoint
```

### Inverse Projection (2D → 3D)

```
(u, v) = InverseAffineTransform(u_pixel - cx, v_pixel - cy)
rho = sqrt(u² + v²)
theta = b₀ + b₁*rho + b₂*rho² + ... + bₘ*rhoᵐ  (inverse polynomial)
[x, y, z] = [u/rho * sin(theta), v/rho * sin(theta), -cos(theta)]
```

## Parameters

The Ocam model requires the following parameters:

| Parameter | Description |
|-----------|-------------|
| `cx`, `cy` | Principal point (pixels) |
| `c`, `d`, `e` | Affine transformation parameters |
| `world2cam_coeffs` | Projection polynomial coefficients (3D→2D) |
| `cam2world_coeffs` | Back-projection polynomial coefficients (2D→3D) |

## Configuration File Format

### YAML Configuration

```yaml
cam0:
  camera_model: ocam
  intrinsics: [
    cx, cy,              # Principal point
    c, d, e,             # Affine parameters
    w2c_len, c2w_len,    # Polynomial lengths
    w2c_coeffs...,       # Projection polynomial
    c2w_coeffs...        # Back-projection polynomial
  ]
  distortion_model: none
  distortion_coeffs: []
  resolution: [width, height]
  rostopic: /camera/image_raw
```

### Example

```yaml
cam0:
  camera_model: ocam
  intrinsics: [
    961.44, 764.67,                    # cx, cy
    0.99888, 0.00048, -0.00079,        # c, d, e
    12, 5,                             # polynomial lengths
    829.33, 637.10, 51.24, 58.38, 80.20, -4.64, 10.81, 51.24, 3.20, -35.83, -21.63, -3.89,  # w2c
    -414.93, 0.0, 0.0003, 2.5e-07, 1.1e-10  # c2w
  ]
  distortion_model: none
  distortion_coeffs: []
  resolution: [1920, 1536]
  rostopic: /camera/image_raw
```

## Converting from JSON Format

If you have Ocam parameters in JSON format (e.g., from `park_front.json`):

```bash
python kalibr_common/OcamConverter.py \
    --input park_front.json \
    --output ocam_config.yaml \
    --names cam0 \
    --topics /camera/image_raw
```

For multiple cameras:

```bash
python kalibr_common/OcamConverter.py \
    --input front.json left.json back.json right.json \
    --output ocam_chain.yaml \
    --names cam0 cam1 cam2 cam3 \
    --topics /front/image /left/image /back/image /right/image
```

## Running Calibration

### Camera-Only Calibration

```bash
kalibr_calibrate_cameras \
    --models ocam-none \
    --bag your_data.bag \
    --target target.yaml \
    --bag-from-to 0 100
```

### Camera-IMU Calibration

```bash
kalibr_calibrate_imu_camera \
    --cam camchain.yaml \
    --imu imu.yaml \
    --target target.yaml \
    --bag your_data.bag \
    --bag-from-to 0 100
```

## Supported Camera Model Strings

| Model String | Description |
|--------------|-------------|
| `ocam-none` | Ocam without additional distortion |

Note: The Ocam model itself includes distortion through its polynomial mapping, so `none` is typically used for the distortion model.

## Implementation Details

### Files Added/Modified

1. **C++ Core**:
   - `aslam_cameras/include/aslam/cameras/OcamProjection.hpp` - Header file
   - `aslam_cameras/include/aslam/cameras/implementation/OcamProjection.hpp` - Implementation

2. **Type Definitions**:
   - `aslam_cameras/include/aslam/cameras.hpp` - Added OcamCameraGeometry typedefs

3. **Factory**:
   - `aslam_cameras/src/CameraGeometryBase.cpp` - Added Ocam to factory

4. **Python Bindings**:
   - `aslam_cv_python/src/CameraProjections.cpp` - Added OcamProjection export
   - `aslam_cv_python/src/CameraGeometries.cpp` - Added OcamCameraGeometry export
   - `aslam_cv_python/src/Frame.cpp` - Added OcamFrame export
   - `aslam_cv_backend_python/src/module.cpp` - Added reprojection error support

5. **Configuration**:
   - `kalibr_common/ConfigReader.py` - Added ocam model support
   - `kalibr_calibrate_cameras` - Added ocam-none to available models
   - `kalibr_camera_calibration/CameraUtils.py` - Added ocam export support

6. **Tools**:
   - `kalibr_common/OcamConverter.py` - JSON to YAML conversion tool

## Notes

1. **Polynomial Evaluation**: Uses Horner's method for efficient and numerically stable evaluation.

2. **Jacobian Computation**: Analytical Jacobians are implemented for optimization.

3. **FOV Limit**: The `_fov_parameter` can be set to limit the valid field of view.

4. **Initialization**: Currently uses a basic initialization. For best results, provide initial parameters close to the true values.

## References

- Original Ocam model: Scaramuzza, D., Martinelli, A., & Siegwart, R. (2006). A toolbox for easily calibrating omnidirectional cameras.
- CPAC implementation reference: `run_avm_new.cpp` in the SensorsCalibration repository.
