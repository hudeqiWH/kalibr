# Kalibr Ocam 相机支持 - 修改总结

## 概述
本文档总结了在 Kalibr 中增加 Ocam (Omnidirectional Camera) 相机模型支持的所有修改。

## 新创建的文件

### 1. C++ 核心投影模型
- **aslam_cv/aslam_cameras/include/aslam/cameras/OcamProjection.hpp**
  - Ocam 投影模型头文件，定义了模板类 `OcamProjection<DISTORTION_T>`
  - 包含关键函数：投影、反投影、雅可比计算等
  - 支持多项式系数 world2cam 和 cam2world

- **aslam_cv/aslam_cameras/include/aslam/cameras/implementation/OcamProjection.hpp**
  - Ocam 投影模型的完整实现
  - 实现了 CPAC (Central Panoramic Affine Camera) 投影模型
  - 使用 Horner's method 进行多项式求值
  - 包含完整的雅可比矩阵计算

### 2. Python 工具
- **aslam_offline_calibration/kalibr/python/kalibr_common/OcamConverter.py**
  - JSON 到 YAML 配置转换工具
  - 支持将 Ocam JSON 参数转换为 Kalibr YAML 格式
  - 支持多相机批量转换

- **aslam_cv/aslam_cameras/test/testOcamProjection.cpp**
  - 单元测试文件
  - 验证投影/反投影一致性
  - 使用 park_front.json 中的真实参数进行测试

## 修改的文件

### 3. 相机类型定义
**aslam_cv/aslam_cameras/include/aslam/cameras.hpp**
- 添加 `#include <aslam/cameras/OcamProjection.hpp>`
- 添加 Ocam 相机几何类型定义：
  - `OcamCameraGeometry` (GlobalShutter + NoMask)
  - `OcamRsCameraGeometry` (RollingShutter + NoMask)
  - `MaskedOcamCameraGeometry` (GlobalShutter + ImageMask)
  - `MaskedOcamRsCameraGeometry` (RollingShutter + ImageMask)

### 4. 工厂函数
**aslam_cv/aslam_cameras/src/CameraGeometryBase.cpp**
- 在 `create()` 函数中添加 Ocam 相机类型的分支：
  - "Ocam" -> OcamCameraGeometry
  - "OcamRs" -> OcamRsCameraGeometry
  - "MaskedOcam" -> MaskedOcamCameraGeometry
  - "MaskedOcamRs" -> MaskedOcamRsCameraGeometry

### 5. Python 绑定 - 投影模型
**aslam_cv/aslam_cv_python/src/CameraProjections.cpp**
- 添加 `#include <aslam/cameras/OcamProjection.hpp>`
- 添加 `exportOcamProjection<NoDistortion>("OcamProjection")` 函数
- 导出 Ocam 特有的 getter 函数：cx(), cy(), c(), d(), e(), world2camCoeffs(), cam2worldCoeffs()

### 6. Python 绑定 - 相机几何
**aslam_cv/aslam_cv_python/src/CameraGeometries.cpp**
- 添加 Ocam 相机几何的导出：
  - OcamCameraGeometry
  - OcamRsCameraGeometry
  - MaskedOcamCameraGeometry
  - MaskedOcamRsCameraGeometry

### 7. Python 绑定 - 帧类型
**aslam_cv/aslam_cv_python/src/Frame.cpp**
- 添加 `aslam::python::exportFrame<OcamCameraGeometry>("OcamFrame")`

### 8. 后端 Python 绑定
**aslam_cv/aslam_cv_backend_python/src/module.cpp**
- 添加 `#include <aslam/cameras/OcamProjection.hpp>`
- 导出重投影误差：`exportReprojectionErrors<OcamCameraGeometry>("Ocam")`
- 导出设计变量：`exportCameraDesignVariables<OcamCameraGeometry>`
- 导出投影设计变量：`exportGenericProjectionDesignVariable<OcamProjection<NoDistortion>>`

### 9. 配置解析
**aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py**
- `AslamCamera.__init__`: 添加 ocam 模型支持
- `CameraParameters.checkIntrinsics`: 添加 ocam 内参验证
- `printDetails`: 添加 ocam 模型输出支持

### 10. 标定脚本
**aslam_offline_calibration/kalibr/python/kalibr_calibrate_cameras**
- 在 `cameraModels` 字典中添加：`'ocam-none': acvb.Ocam`

### 11. 相机工具
**aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py**
- `saveChainParametersYaml`: 添加 ocam 模型到 YAML 的导出
- 更新 `cameraModels` 和 `distortionModels` 字典

## Ocam 内参格式

### YAML 配置格式
```yaml
cam0:
  camera_model: ocam
  intrinsics: [cx, cy, c, d, e, w2c_len, c2w_len, w2c_coeffs..., c2w_coeffs...]
  distortion_model: none
  distortion_coeffs: []
  resolution: [width, height]
  rostopic: /camera/image_raw
```

### 参数说明
- `cx, cy`: 主点 (principal point)
- `c, d, e`: 仿射变换参数
- `w2c_len`: world2cam 多项式系数数量
- `c2w_len`: cam2world 多项式系数数量
- `w2c_coeffs`: world2cam 多项式系数 (3D→2D 投影)
- `c2w_coeffs`: cam2world 多项式系数 (2D→3D 反投影)

## 编译说明

### 编译命令
```bash
cd /path/to/kalibr_ws
catkin build aslam_cameras aslam_cv_python aslam_cv_backend_python
# 或
catkin_make
```

### 测试
```bash
# 运行单元测试
rosrun aslam_cameras testOcamProjection

# 转换配置文件
python aslam_offline_calibration/kalibr/python/kalibr_common/OcamConverter.py \
    --input park_front.json --output ocam.yaml

# 运行标定
rosrun kalibr kalibr_calibrate_cameras \
    --models ocam-none \
    --target target.yaml \
    --bag data.bag \
    --topics /camera/image_raw
```

## 参考

### 数学模型
Ocam 模型使用 CPAC (Central Panoramic Affine Camera) 公式：

**投影 (3D → 2D):**
```
norm = sqrt(x² + y²)
theta = atan(-z / norm)
rho = poly_w2c(theta)  # 多项式求值
uu = (x/norm) * rho
vv = (y/norm) * rho
u = uu + vv*e + cx
v = uu*d + vv*c + cy
```

**反投影 (2D → 3D):**
```
# 逆仿射变换
det = c - e*d
vv = (v - cy - (u - cx)*d) / det
uu = u - cx - vv*e

rho = sqrt(uu² + vv²)
theta = poly_c2w(rho)  # 逆多项式求值
```

### 参考文件
- 示例内参: `third_party/SensorsCalibration/surround-camera/manual_calib_front3/2_dog/param/park_front.json`
- 参考实现: `third_party/SensorsCalibration/surround-camera/manual_calib_new/src/run_avm_new.cpp`

## 注意事项

1. **多项式系数顺序**: 系数按照多项式的阶数从低到高排列
2. **坐标系**: Ocam 模型使用 CPAC 约定，相机看向 -z 方向
3. **畸变模型**: Ocam 本身包含畸变，通常与 'none' 畸变模型一起使用
4. **初始化**: 标定前需要提供合理的初始内参值
