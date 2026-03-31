# Ocam Camera Model Build Notes

This document provides notes for building Kalibr with the Ocam camera model support.

## Files Added

### Core C++ Implementation
1. `aslam_cv/aslam_cameras/include/aslam/cameras/OcamProjection.hpp`
2. `aslam_cv/aslam_cameras/include/aslam/cameras/implementation/OcamProjection.hpp`

### Test Files
3. `aslam_cv/aslam_cameras/test/testOcamProjection.cpp`

### Python Tools
4. `aslam_offline_calibration/kalibr/python/kalibr_common/OcamConverter.py`

### Configuration Examples
5. `aslam_offline_calibration/kalibr/config/ocam_camera_example.yaml`
6. `aslam_offline_calibration/kalibr/config/ocam_imu_example.yaml`

### Documentation
7. `aslam_offline_calibration/kalibr/OCAM_README.md`
8. `OCAM_BUILD_NOTES.md` (this file)

## Files Modified

### Core Camera Library
1. `aslam_cv/aslam_cameras/include/aslam/cameras.hpp`
   - Added OcamProjection include
   - Added OcamCameraGeometry typedefs

2. `aslam_cv/aslam_cameras/src/CameraGeometryBase.cpp`
   - Added Ocam to factory function

### Python Bindings
3. `aslam_cv/aslam_cv_python/src/CameraProjections.cpp`
   - Added OcamProjection include
   - Added exportOcamProjection function
   - Added export call for OcamProjection

4. `aslam_cv/aslam_cv_python/src/CameraGeometries.cpp`
   - Added export calls for OcamCameraGeometry variants

5. `aslam_cv/aslam_cv_python/src/Frame.cpp`
   - Added export call for OcamFrame

6. `aslam_cv/aslam_cv_backend_python/src/module.cpp`
   - Added OcamProjection include
   - Added export calls for Ocam reprojection errors

### Calibration Tools
7. `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py`
   - Added 'ocam' to cameraModels list
   - Added ocam case in checkIntrinsics()
   - Added ocam case in AslamCamera constructor

8. `aslam_offline_calibration/kalibr/python/kalibr_calibrate_cameras`
   - Added 'ocam-none' to cameraModels dict

9. `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py`
   - Added acvb.Ocam to cameraModels and distortionModels
   - Added ocam case in saveChainParametersYaml()

## Build Instructions

### Standard Build

```bash
cd /path/to/kalibr_ws
catkin build -j4
```

### Clean Build (if needed)

```bash
cd /path/to/kalibr_ws
rm -rf build devel
catkin build -j4
```

### Build Specific Packages

```bash
catkin build aslam_cv aslam_cv_backend aslam_cv_python aslam_cv_backend_python kalibr
```

## Testing

### Run Unit Tests

```bash
# If using catkin tools
catkin run_tests aslam_cameras

# Or manually
rosrun aslam_cameras aslam_cameras_test
```

### Test Ocam Projection

```bash
# Convert your Ocam JSON to Kalibr format
python kalibr_common/OcamConverter.py \
    --input your_camera.json \
    --output test_config.yaml

# Run camera calibration
kalibr_calibrate_cameras \
    --models ocam-none \
    --bag test.bag \
    --target target.yaml \
    --config test_config.yaml
```

## Troubleshooting

### Import Error

If you get `ImportError: No module named 'aslam_cv'`:

```bash
source /path/to/kalibr_ws/devel/setup.bash
```

### Compilation Error

If you get compilation errors related to OcamProjection:

1. Check that all headers are properly included
2. Ensure template instantiation is correct
3. Verify that boost::serialization is properly set up

### Runtime Error

If you get runtime errors during calibration:

1. Verify your intrinsics array has the correct length
2. Check that polynomial coefficients are in the correct order
3. Ensure the camera model string is exactly 'ocam-none'

## Verification

To verify the installation:

```python
import aslam_cv as cv

# Check if OcamProjection is available
try:
    proj = cv.OcamProjection()
    print("OcamProjection is available!")
except AttributeError:
    print("OcamProjection is NOT available")

# Check if OcamCameraGeometry is available
try:
    geom = cv.OcamCameraGeometry()
    print("OcamCameraGeometry is available!")
except AttributeError:
    print("OcamCameraGeometry is NOT available")
```

## Notes

1. The Ocam model uses the same polynomial convention as the reference implementation in `run_avm_new.cpp`.

2. The polynomial coefficients should be ordered from highest degree to lowest (for Horner's method evaluation).

3. The `world2cam_coeffs` are used for projection (3D→2D), and `cam2world_coeffs` are used for back-projection (2D→3D).

4. The affine transformation matrix is:
   ```
   [1  e]
   [d  c]
   ```
   where c, d, e are the affine parameters.
