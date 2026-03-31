# Ocam Camera Model Implementation Summary

This document summarizes the complete implementation of Ocam camera model support in Kalibr for camera-IMU calibration.

## Overview

The Ocam (Omnidirectional Camera) model has been successfully integrated into Kalibr, enabling calibration of fisheye cameras with polynomial-based projection models. This implementation follows the CPAC (Camera Polynomial Ambient Calculator) model used in the reference `run_avm_new.cpp`.

## Implementation Checklist

### ✅ Core C++ Implementation

| File | Status | Description |
|------|--------|-------------|
| `OcamProjection.hpp` | ✅ Created | Header file with class definition |
| `implementation/OcamProjection.hpp` | ✅ Created | Template implementation with projection/back-projection |
| `cameras.hpp` | ✅ Modified | Added OcamCameraGeometry typedefs |
| `CameraGeometryBase.cpp` | ✅ Modified | Added Ocam to factory function |

### ✅ Python Bindings

| File | Status | Description |
|------|--------|-------------|
| `CameraProjections.cpp` | ✅ Modified | Added OcamProjection Python export |
| `CameraGeometries.cpp` | ✅ Modified | Added OcamCameraGeometry export |
| `Frame.cpp` | ✅ Modified | Added OcamFrame export |
| `module.cpp` (backend) | ✅ Modified | Added Ocam reprojection error support |

### ✅ Calibration Tools

| File | Status | Description |
|------|--------|-------------|
| `ConfigReader.py` | ✅ Modified | Added ocam model support |
| `kalibr_calibrate_cameras` | ✅ Modified | Added ocam-none to available models |
| `CameraUtils.py` | ✅ Modified | Added ocam export support |

### ✅ Utilities & Documentation

| File | Status | Description |
|------|--------|-------------|
| `OcamConverter.py` | ✅ Created | JSON to YAML conversion tool |
| `ocam_camera_example.yaml` | ✅ Created | Single camera config example |
| `ocam_imu_example.yaml` | ✅ Created | Camera-IMU config example |
| `testOcamProjection.cpp` | ✅ Created | Unit tests |
| `OCAM_README.md` | ✅ Created | User documentation |
| `OCAM_BUILD_NOTES.md` | ✅ Created | Build instructions |

## Key Features

### 1. Mathematical Model
- **Forward Projection**: 3D point → 2D keypoint using polynomial evaluation
- **Inverse Projection**: 2D keypoint → 3D ray using inverse polynomial
- **Affine Transformation**: Handles sensor-specific affine distortions
- **Horner's Method**: Efficient and numerically stable polynomial evaluation

### 2. Configuration Format
```yaml
cam0:
  camera_model: ocam
  intrinsics: [cx, cy, c, d, e, w2c_len, c2w_len, w2c_coeffs..., c2w_coeffs...]
  distortion_model: none
  distortion_coeffs: []
  resolution: [width, height]
  rostopic: /camera/image_raw
```

### 3. Supported Operations
- ✅ Projection (3D → 2D)
- ✅ Back-projection (2D → 3D)
- ✅ Jacobian computation for optimization
- ✅ Serialization/deserialization
- ✅ Python bindings
- ✅ Reprojection error computation

## Usage Examples

### Convert Existing Ocam Parameters
```bash
python OcamConverter.py \
    --input park_front.json \
    --output kalibr_config.yaml \
    --names cam0 \
    --topics /camera/image_raw
```

### Camera Calibration
```bash
kalibr_calibrate_cameras \
    --models ocam-none \
    --bag data.bag \
    --target target.yaml \
    --config kalibr_config.yaml
```

### Camera-IMU Calibration
```bash
kalibr_calibrate_imu_camera \
    --cam camchain.yaml \
    --imu imu.yaml \
    --target target.yaml \
    --bag data.bag
```

## Technical Details

### Polynomial Evaluation
```cpp
// Horner's method for efficient evaluation
double result = 0.0;
for (auto it = coeffs.rbegin(); it != coeffs.rend(); ++it) {
    result = result * x + *it;
}
```

### Projection Formula
```cpp
double theta = atan(-z / norm);
double rho = evaluatePolynomial(world2cam_coeffs, theta);
double u = x / norm * rho;
double v = y / norm * rho;
// Apply affine transformation
u_pixel = u + e * v + cx;
v_pixel = d * u + c * v + cy;
```

### Back-Projection Formula
```cpp
// Inverse affine transformation
double uu = c * u - e * v;
double vv = -d * u + v;
// Evaluate inverse polynomial
double theta = evaluatePolynomial(cam2world_coeffs, rho);
// Compute 3D direction
x = uu / rho * sin(theta);
y = vv / rho * sin(theta);
z = -cos(theta);
```

## Integration with Existing Kalibr Features

### Camera Models
- Works alongside existing models (pinhole, omni, eucm, ds)
- Uses same calibration pipeline
- Compatible with all target types

### Distortion Models
- Ocam uses `none` distortion model (polynomial handles distortion)
- Can be extended to support additional distortion if needed

### Shutter Models
- Supports both GlobalShutter and RollingShutter
- Mask variants also supported

### Optimization
- Full analytical Jacobian support
- Compatible with Ceres Solver backend
- Supports intrinsic/extrinsic optimization

## Testing Strategy

### Unit Tests
- Projection/back-projection consistency
- Principal point accuracy
- Parameter getters/setters
- Edge cases (center point, boundaries)

### Integration Tests
- End-to-end calibration with synthetic data
- Comparison with reference implementation
- Real-world calibration validation

## Future Enhancements

### Potential Improvements
1. **Automatic Initialization**: Implement better initial parameter estimation
2. **FOV Limiting**: Add configurable field of view limits
3. **Polynomial Order**: Support variable polynomial orders more flexibly
4. **Distortion Combination**: Allow combining with additional distortion models

### Extensions
1. **Multi-Camera**: Full support for multi-camera rigs with Ocam
2. **Online Calibration**: Real-time parameter refinement
3. **Uncertainty Estimation**: Covariance propagation through polynomials

## References

1. Scaramuzza, D., Martinelli, A., & Siegwart, R. (2006). A toolbox for easily calibrating omnidirectional cameras.
2. Kalibr: https://github.com/ethz-asl/kalibr
3. Reference implementation: `run_avm_new.cpp` in SensorsCalibration

## Conclusion

The Ocam camera model has been fully integrated into Kalibr, providing:
- Complete C++ implementation with Python bindings
- Full calibration pipeline support
- Comprehensive documentation and examples
- Conversion tools for existing parameters

This enables accurate calibration of fisheye cameras using the well-established Ocam polynomial model within the Kalibr framework.
