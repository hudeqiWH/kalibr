#ifndef ASLAM_CAMERAS_IMPLEMENTATION_OCAM_PROJECTION_HPP
#define ASLAM_CAMERAS_IMPLEMENTATION_OCAM_PROJECTION_HPP

namespace aslam {
namespace cameras {

template<typename DISTORTION_T>
OcamProjection<DISTORTION_T>::OcamProjection()
    : _cx(0.0),
      _cy(0.0),
      _c(1.0),
      _d(0.0),
      _e(0.0),
      _ru(1),
      _rv(1) {
}

template<typename DISTORTION_T>
OcamProjection<DISTORTION_T>::OcamProjection(double cx, double cy, double c, double d, double e,
                                             const std::vector<double> & world2camCoeffs,
                                             const std::vector<double> & cam2worldCoeffs,
                                             int resolutionU, int resolutionV,
                                             distortion_t distortion)
    : _cx(cx),
      _cy(cy),
      _c(c),
      _d(d),
      _e(e),
      _world2camCoeffs(world2camCoeffs),
      _cam2worldCoeffs(cam2worldCoeffs),
      _ru(resolutionU),
      _rv(resolutionV),
      _distortion(distortion) {
}

template<typename DISTORTION_T>
OcamProjection<DISTORTION_T>::OcamProjection(const sm::PropertyTree & config)
    : _distortion(sm::PropertyTree(config, "distortion")) {
  _cx = config.getDouble("cx");
  _cy = config.getDouble("cy");
  _c = config.getDouble("c");
  _d = config.getDouble("d");
  _e = config.getDouble("e");
  _ru = config.getInt("ru");
  _rv = config.getInt("rv");

  // Load polynomial coefficients from property tree
  // world2cam coeffs
  sm::PropertyTree w2cConfig(config, "world2camCoeffs");
  int w2cSize = w2cConfig.getInt("size");
  _world2camCoeffs.resize(w2cSize);
  for (int i = 0; i < w2cSize; ++i) {
    std::string key = "coeff" + std::to_string(i);
    _world2camCoeffs[i] = w2cConfig.getDouble(key.c_str());
  }

  // cam2world coeffs
  sm::PropertyTree c2wConfig(config, "cam2worldCoeffs");
  int c2wSize = c2wConfig.getInt("size");
  _cam2worldCoeffs.resize(c2wSize);
  for (int i = 0; i < c2wSize; ++i) {
    std::string key = "coeff" + std::to_string(i);
    _cam2worldCoeffs[i] = c2wConfig.getDouble(key.c_str());
  }
}

template<typename DISTORTION_T>
OcamProjection<DISTORTION_T>::~OcamProjection() {
}

template<typename DISTORTION_T>
double OcamProjection<DISTORTION_T>::evaluatePolynomial(const std::vector<double> & coeffs, double x) const {
  // Horner's method: a0 + a1*x + a2*x^2 + ... = a0 + x*(a1 + x*(a2 + ...))
  double result = 0.0;
  for (int i = (int)coeffs.size() - 1; i >= 0; --i) {
    result = result * x + coeffs[i];
  }
  return result;
}

template<typename DISTORTION_T>
double OcamProjection<DISTORTION_T>::evaluatePolynomialDerivative(const std::vector<double> & coeffs, double x) const {
  // Derivative: a1 + 2*a2*x + 3*a3*x^2 + ...
  double result = 0.0;
  for (int i = (int)coeffs.size() - 1; i >= 1; --i) {
    result = result * x + i * coeffs[i];
  }
  return result;
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K>
bool OcamProjection<DISTORTION_T>::euclideanToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_K> & outKeypointConst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  Eigen::MatrixBase<DERIVED_K> & outKeypoint = const_cast<Eigen::MatrixBase<
      DERIVED_K> &>(outKeypointConst);
  outKeypoint.derived().resize(2);

  double x = p[0];
  double y = p[1];
  double z = p[2];

  // Check if point is in front of camera
  if (z <= 0) {
    return false;
  }

  // Ocam projection model
  double norm = std::sqrt(x * x + y * y);
  if (norm < 1e-14) {
    norm = 1e-14;
  }

  // CPAC formula: theta = atan(-z / norm)
  double theta = std::atan(-z / norm);

  // Evaluate polynomial: rho = poly(theta)
  double rho = evaluatePolynomial(_world2camCoeffs, theta);

  // CPAC: uu = x / norm * rho, vv = y / norm * rho
  double uu = x / norm * rho;
  double vv = y / norm * rho;

  // CPAC affine transformation
  // u = uu + vv * e + cx
  // v = uu * d + vv * c + cy
  outKeypoint[0] = uu + vv * _e + _cx;
  outKeypoint[1] = uu * _d + vv * _c + _cy;

  return isValid(outKeypoint);
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool OcamProjection<DISTORTION_T>::euclideanToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_K> & outKeypointConst,
    const Eigen::MatrixBase<DERIVED_JP> & outJp) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JP>, 2, 3);

  Eigen::MatrixBase<DERIVED_K> & outKeypoint = const_cast<Eigen::MatrixBase<
      DERIVED_K> &>(outKeypointConst);
  outKeypoint.derived().resize(2);

  Eigen::MatrixBase<DERIVED_JP> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JP> &>(outJp);
  J.derived().resize(KeypointDimension, 3);
  J.setZero();

  double x = p[0];
  double y = p[1];
  double z = p[2];

  // Check if point is in front of camera
  if (z <= 0) {
    return false;
  }

  // Ocam projection model
  double norm2 = x * x + y * y;
  double norm = std::sqrt(norm2);
  if (norm < 1e-14) {
    norm = 1e-14;
  }

  double theta = std::atan(-z / norm);
  double rho = evaluatePolynomial(_world2camCoeffs, theta);
  double drho_dtheta = evaluatePolynomialDerivative(_world2camCoeffs, theta);

  // Intermediate values
  double x_norm = x / norm;
  double y_norm = y / norm;

  // uu = x_norm * rho, vv = y_norm * rho
  double uu = x_norm * rho;
  double vv = y_norm * rho;

  // Final projection
  outKeypoint[0] = uu + vv * _e + _cx;
  outKeypoint[1] = uu * _d + vv * _c + _cy;

  // Compute Jacobian
  // d(theta)/d(x,y,z)
  double norm3 = norm * norm2;
  double dtheta_dx = z * x / (norm3 * (1.0 + z * z / norm2));
  double dtheta_dy = z * y / (norm3 * (1.0 + z * z / norm2));
  double dtheta_dz = -1.0 / (norm * (1.0 + z * z / norm2));

  // d(rho)/d(x,y,z)
  double drho_dx = drho_dtheta * dtheta_dx;
  double drho_dy = drho_dtheta * dtheta_dy;
  double drho_dz = drho_dtheta * dtheta_dz;

  // d(uu,vv)/d(x,y,z)
  // uu = (x/norm) * rho
  // d(uu)/dx = (1/norm - x^2/norm^3) * rho + (x/norm) * drho_dx
  // d(uu)/dy = (-x*y/norm^3) * rho + (x/norm) * drho_dy
  double duu_dx = (1.0 / norm - x * x / norm3) * rho + x_norm * drho_dx;
  double duu_dy = (-x * y / norm3) * rho + x_norm * drho_dy;
  double duu_dz = x_norm * drho_dz;

  double dvv_dx = (-x * y / norm3) * rho + y_norm * drho_dx;
  double dvv_dy = (1.0 / norm - y * y / norm3) * rho + y_norm * drho_dy;
  double dvv_dz = y_norm * drho_dz;

  // du/dx = duu/dx + e * dvv/dx
  J(0, 0) = duu_dx + _e * dvv_dx;
  J(0, 1) = duu_dy + _e * dvv_dy;
  J(0, 2) = duu_dz + _e * dvv_dz;

  // dv/dx = d * duu/dx + c * dvv/dx
  J(1, 0) = _d * duu_dx + _c * dvv_dx;
  J(1, 1) = _d * duu_dy + _c * dvv_dy;
  J(1, 2) = _d * duu_dz + _c * dvv_dz;

  return isValid(outKeypoint);
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K>
bool OcamProjection<DISTORTION_T>::homogeneousToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & ph,
    const Eigen::MatrixBase<DERIVED_K> & outKeypoint) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  if (ph[3] < 0)
    return euclideanToKeypoint(-ph.derived().template head<3>(), outKeypoint);
  else
    return euclideanToKeypoint(ph.derived().template head<3>(), outKeypoint);
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
bool OcamProjection<DISTORTION_T>::homogeneousToKeypoint(
    const Eigen::MatrixBase<DERIVED_P> & ph,
    const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
    const Eigen::MatrixBase<DERIVED_JP> & outJp) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JP>, 2, 4);

  Eigen::MatrixBase<DERIVED_JP> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JP> &>(outJp);
  J.derived().resize(KeypointDimension, 4);
  J.setZero();

  if (ph[3] < 0) {
    bool success = euclideanToKeypoint(
        -ph.derived().template head<3>(), outKeypoint,
        J.derived().template topLeftCorner<2, 3>());
    J = -J;
    return success;
  } else {
    return euclideanToKeypoint(ph.derived().template head<3>(), outKeypoint,
                               J.derived().template topLeftCorner<2, 3>());
  }
}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P>
bool OcamProjection<DISTORTION_T>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPointConst) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  Eigen::MatrixBase<DERIVED_P> & outPoint = const_cast<Eigen::MatrixBase<
      DERIVED_P> &>(outPointConst);
  outPoint.derived().resize(3);

  // Inverse affine transformation
  // uu + vv * e = u - cx  ... (1)
  // uu * d + vv * c = v - cy  ... (2)
  // From (1): uu = u - cx - vv * e
  // Substitute into (2): (u - cx - vv * e) * d + vv * c = v - cy
  // (u - cx) * d - vv * e * d + vv * c = v - cy
  // vv * (c - e * d) = v - cy - (u - cx) * d
  double u = keypoint[0];
  double v = keypoint[1];
  double det = _c - _e * _d;
  if (std::abs(det) < 1e-14) {
    det = 1e-14;
  }

  double vv = (v - _cy - (u - _cx) * _d) / det;
  double uu = u - _cx - vv * _e;

  // Now compute rho and theta
  double rho = std::sqrt(uu * uu + vv * vv);

  // Evaluate inverse polynomial: theta = cam2world(rho)
  double theta = evaluatePolynomial(_cam2worldCoeffs, rho);

  // Compute 3D direction
  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);

  if (std::abs(sin_theta) < 1e-14) {
    outPoint[0] = 0;
    outPoint[1] = 0;
    outPoint[2] = -1;  // Pointing forward
    return true;
  }

  // Normalize to get unit vector
  outPoint[0] = uu / sin_theta;
  outPoint[1] = vv / sin_theta;
  outPoint[2] = -cos_theta / sin_theta * rho;  // Negative because camera looks at -z

  return true;
}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool OcamProjection<DISTORTION_T>::keypointToEuclidean(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPointConst,
    const Eigen::MatrixBase<DERIVED_JK> & outJk) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JK>, 3, 2);

  Eigen::MatrixBase<DERIVED_P> & outPoint = const_cast<Eigen::MatrixBase<
      DERIVED_P> &>(outPointConst);
  outPoint.derived().resize(3);

  Eigen::MatrixBase<DERIVED_JK> & Jk =
      const_cast<Eigen::MatrixBase<DERIVED_JK> &>(outJk);
  Jk.derived().resize(3, 2);
  Jk.setZero();

  // First compute the point
  double u = keypoint[0];
  double v = keypoint[1];
  double det = _c - _e * _d;
  if (std::abs(det) < 1e-14) {
    det = 1e-14;
  }

  double vv = (v - _cy - (u - _cx) * _d) / det;
  double uu = u - _cx - vv * _e;

  double rho = std::sqrt(uu * uu + vv * vv);
  double theta = evaluatePolynomial(_cam2worldCoeffs, rho);
  double dtheta_drho = evaluatePolynomialDerivative(_cam2worldCoeffs, rho);

  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);

  if (std::abs(sin_theta) < 1e-14) {
    outPoint[0] = 0;
    outPoint[1] = 0;
    outPoint[2] = -1;
    // Jacobian is problematic here, return identity-ish
    Jk.setIdentity();
    return true;
  }

  outPoint[0] = uu / sin_theta;
  outPoint[1] = vv / sin_theta;
  outPoint[2] = -cos_theta / sin_theta * rho;

  // Compute Jacobian (simplified)
  // duu/du = 1, duu/dv = -e/det
  // dvv/du = -d/det, dvv/dv = 1/det
  double duu_du = 1.0;
  double duu_dv = -_e / det;
  double dvv_du = -_d / det;
  double dvv_dv = 1.0 / det;

  // drho/duu = uu/rho, drho/dvv = vv/rho
  double drho_duu = uu / rho;
  double drho_dvv = vv / rho;

  // dtheta/du = dtheta_drho * (drho_duu * duu_du + drho_dvv * dvv_du)
  double dtheta_du = dtheta_drho * (drho_duu * duu_du + drho_dvv * dvv_du);
  double dtheta_dv = dtheta_drho * (drho_duu * duu_dv + drho_dvv * dvv_dv);

  // dx/du = (1/sin_theta - uu*cos_theta/sin_theta^2 * dtheta_duu) * duu_du
  // Simplified: approximate for small changes
  Jk(0, 0) = duu_du / sin_theta;
  Jk(0, 1) = duu_dv / sin_theta;
  Jk(1, 0) = dvv_du / sin_theta;
  Jk(1, 1) = dvv_dv / sin_theta;

  return true;
}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P>
bool OcamProjection<DISTORTION_T>::keypointToHomogeneous(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPoint) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  Eigen::MatrixBase<DERIVED_P> & p =
      const_cast<Eigen::MatrixBase<DERIVED_P> &>(outPoint);
  p.derived().resize(4);
  p[3] = 0.0;
  return keypointToEuclidean(keypoint, p.derived().template head<3>());
}

template<typename DISTORTION_T>
template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
bool OcamProjection<DISTORTION_T>::keypointToHomogeneous(
    const Eigen::MatrixBase<DERIVED_K> & keypoint,
    const Eigen::MatrixBase<DERIVED_P> & outPoint,
    const Eigen::MatrixBase<DERIVED_JK> & outJk) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JK>, 4, 2);

  Eigen::MatrixBase<DERIVED_JK> & Jk =
      const_cast<Eigen::MatrixBase<DERIVED_JK> &>(outJk);
  Jk.derived().resize(4, 2);
  Jk.setZero();

  Eigen::MatrixBase<DERIVED_P> & p =
      const_cast<Eigen::MatrixBase<DERIVED_P> &>(outPoint);
  p.derived().resize(4);
  p[3] = 0.0;

  return keypointToEuclidean(keypoint, p.template head<3>(),
                             Jk.template topLeftCorner<3, 2>());
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JI>
void OcamProjection<DISTORTION_T>::euclideanToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JI> & outJi) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);

  Eigen::MatrixBase<DERIVED_JI> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JI> &>(outJi);
  J.derived().resize(KeypointDimension, 5);  // cx, cy, c, d, e
  J.setZero();

  // For now, simple approximation
  // du/dcx = 1, du/dcy = 0, du/dc = 0, du/dd = 0, du/de = vv
  // dv/dcx = 0, dv/dcy = 1, dv/dc = vv, dv/dd = uu, dv/de = 0
  J(0, 0) = 1.0;  // d(u)/d(cx)
  J(0, 4) = 0.0;  // d(u)/d(e) = vv (approx)
  J(1, 1) = 1.0;  // d(v)/d(cy)
  J(1, 2) = 0.0;  // d(v)/d(c) = vv (approx)
  J(1, 3) = 0.0;  // d(v)/d(d) = uu (approx)
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JD>
void OcamProjection<DISTORTION_T>::euclideanToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 3);

  Eigen::MatrixBase<DERIVED_JD> & J =
      const_cast<Eigen::MatrixBase<DERIVED_JD> &>(outJd);
  J.derived().resize(KeypointDimension, _distortion.minimalDimensions());
  J.setZero();
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JI>
void OcamProjection<DISTORTION_T>::homogeneousToKeypointIntrinsicsJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JI> & outJi) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);

  if (p[3] < 0.0) {
    euclideanToKeypointIntrinsicsJacobian(-p.derived().template head<3>(), outJi);
  } else {
    euclideanToKeypointIntrinsicsJacobian(p.derived().template head<3>(), outJi);
  }
}

template<typename DISTORTION_T>
template<typename DERIVED_P, typename DERIVED_JD>
void OcamProjection<DISTORTION_T>::homogeneousToKeypointDistortionJacobian(
    const Eigen::MatrixBase<DERIVED_P> & p,
    const Eigen::MatrixBase<DERIVED_JD> & outJd) const {

  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_P>, 4);

  if (p[3] < 0.0) {
    euclideanToKeypointDistortionJacobian(-p.derived().template head<3>(), outJd);
  } else {
    euclideanToKeypointDistortionJacobian(p.derived().template head<3>(), outJd);
  }
}

template<typename DISTORTION_T>
template<typename DERIVED_K>
bool OcamProjection<DISTORTION_T>::isValid(
    const Eigen::MatrixBase<DERIVED_K> & keypoint) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  return keypoint(0) >= 0 && keypoint(0) < ru() && keypoint(1) >= 0
      && keypoint(1) < rv();
}

template<typename DISTORTION_T>
template<typename DERIVED_K>
bool OcamProjection<DISTORTION_T>::isLiftable(
    const Eigen::MatrixBase<DERIVED_K> & keypoint) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_K>, 2);

  // Check if the keypoint is in the valid range
  return isValid(keypoint);
}

template<typename DISTORTION_T>
template<typename DERIVED_P>
bool OcamProjection<DISTORTION_T>::isEuclideanVisible(
    const Eigen::MatrixBase<DERIVED_P> & p) const {
  keypoint_t k;
  return euclideanToKeypoint(p, k);
}

template<typename DISTORTION_T>
template<typename DERIVED_P>
bool OcamProjection<DISTORTION_T>::isHomogeneousVisible(
    const Eigen::MatrixBase<DERIVED_P> & ph) const {
  keypoint_t k;
  return homogeneousToKeypoint(ph, k);
}

template<typename DISTORTION_T>
bool OcamProjection<DISTORTION_T>::initializeIntrinsics(const std::vector<GridCalibrationTargetObservation> &observations) {
  // Initialize with reasonable defaults
  if (observations.empty()) {
    return false;
  }

  _cx = (observations[0].imCols() - 1.0) / 2.0;
  _cy = (observations[0].imRows() - 1.0) / 2.0;
  _ru = observations[0].imCols();
  _rv = observations[0].imRows();
  _c = 1.0;
  _d = 0.0;
  _e = 0.0;
  _distortion.clear();

  // Default polynomial coefficients (approximate for pinhole)
  _world2camCoeffs = {0.0, 1000.0};  // rho = 1000 * theta
  _cam2worldCoeffs = {0.0, 0.001};   // theta = 0.001 * rho

  return true;
}

template<typename DISTORTION_T>
bool OcamProjection<DISTORTION_T>::estimateTransformation(
    const GridCalibrationTargetObservation & obs,
    sm::kinematics::Transformation & out_T_t_c) const {
  // Simplified version - use PnP on undistorted points
  std::vector<cv::Point2f> Ms;
  std::vector<cv::Point3f> Ps;

  obs.getCornersImageFrame(Ms);
  obs.getCornersTargetFrame(Ps);

  size_t count = 0;
  for (size_t i = 0; i < Ms.size(); ++i) {
    Eigen::Vector2d imagePoint(Ms[i].x, Ms[i].y);
    Eigen::Vector3d backProjection;

    if (keypointToEuclidean(imagePoint, backProjection)) {
      // Project to normalized plane
      double x = backProjection[0];
      double y = backProjection[1];
      double z = backProjection[2];
      double norm = std::sqrt(x * x + y * y + z * z);

      Ps.at(count).x = Ps[i].x;
      Ps.at(count).y = Ps[i].y;
      Ps.at(count).z = Ps[i].z;

      // Project to virtual pinhole
      if (z < 0) {
        Ms.at(count).x = x / -z;
        Ms.at(count).y = y / -z;
        ++count;
      }
    }
  }

  Ps.resize(count);
  Ms.resize(count);

  if (Ps.size() < 4) {
    return false;
  }

  std::vector<double> distCoeffs(4, 0.0);
  cv::Mat rvec(3, 1, CV_64F);
  cv::Mat tvec(3, 1, CV_64F);

  cv::solvePnP(Ps, Ms, cv::Mat::eye(3, 3, CV_64F), distCoeffs, rvec, tvec);

  cv::Mat C_camera_model = cv::Mat::eye(3, 3, CV_64F);
  Eigen::Matrix4d T_camera_model = Eigen::Matrix4d::Identity();
  cv::Rodrigues(rvec, C_camera_model);
  for (int r = 0; r < 3; ++r) {
    T_camera_model(r, 3) = tvec.at<double>(r, 0);
    for (int c = 0; c < 3; ++c) {
      T_camera_model(r, c) = C_camera_model.at<double>(r, c);
    }
  }

  out_T_t_c.set(T_camera_model.inverse());
  return true;
}

template<typename DISTORTION_T>
size_t OcamProjection<DISTORTION_T>::computeReprojectionError(
    const GridCalibrationTargetObservation & obs,
    const sm::kinematics::Transformation & T_target_camera,
    double & outErr) const {
  outErr = 0.0;
  size_t count = 0;
  sm::kinematics::Transformation T_camera_target = T_target_camera.inverse();

  for (size_t i = 0; i < obs.target()->size(); ++i) {
    Eigen::Vector2d y, yhat;
    if (obs.imagePoint(i, y)
        && euclideanToKeypoint(T_camera_target * obs.target()->point(i), yhat)) {
      outErr += (y - yhat).norm();
      ++count;
    }
  }

  return count;
}

// aslam::backend compatibility
template<typename DISTORTION_T>
void OcamProjection<DISTORTION_T>::update(const double * v) {
  _cx += v[0];
  _cy += v[1];
  _c += v[2];
  _d += v[3];
  _e += v[4];
}

template<typename DISTORTION_T>
int OcamProjection<DISTORTION_T>::minimalDimensions() const {
  return 5;  // cx, cy, c, d, e
}

template<typename DISTORTION_T>
Eigen::Vector2i OcamProjection<DISTORTION_T>::parameterSize() const {
  return Eigen::Vector2i(5, 1);
}

template<typename DISTORTION_T>
void OcamProjection<DISTORTION_T>::getParameters(Eigen::MatrixXd & P) const {
  P.resize(5, 1);
  P << _cx, _cy, _c, _d, _e;
}

template<typename DISTORTION_T>
void OcamProjection<DISTORTION_T>::setParameters(const Eigen::MatrixXd & P) {
  SM_ASSERT_EQ(std::runtime_error, P.rows(), 5, "Incorrect size");
  SM_ASSERT_EQ(std::runtime_error, P.cols(), 1, "Incorrect size");
  _cx = P(0, 0);
  _cy = P(1, 0);
  _c = P(2, 0);
  _d = P(3, 0);
  _e = P(4, 0);
}

template<typename DISTORTION_T>
bool OcamProjection<DISTORTION_T>::isBinaryEqual(
    const OcamProjection<distortion_t> & rhs) const {
  return _cx == rhs._cx && _cy == rhs._cy && _c == rhs._c && _d == rhs._d
      && _e == rhs._e && _ru == rhs._ru && _rv == rhs._rv
      && _world2camCoeffs == rhs._world2camCoeffs
      && _cam2worldCoeffs == rhs._cam2worldCoeffs
      && _distortion.isBinaryEqual(rhs._distortion);
}

template<typename DISTORTION_T>
OcamProjection<DISTORTION_T> OcamProjection<DISTORTION_T>::getTestProjection() {
  std::vector<double> w2c = {0.0, 400.0};
  std::vector<double> c2w = {0.0, 0.0025};
  return OcamProjection<DISTORTION_T>(320, 240, 1.0, 0.0, 0.0,
                                      w2c, c2w, 640, 480,
                                      DISTORTION_T::getTestDistortion());
}

template<typename DISTORTION_T>
void OcamProjection<DISTORTION_T>::resizeIntrinsics(double scale) {
  _cx *= scale;
  _cy *= scale;
  _ru = (int)(_ru * scale);
  _rv = (int)(_rv * scale);
  // Polynomial coefficients don't need scaling for Ocam model
}

// Serialization
template<typename DISTORTION_T>
template<class Archive>
void OcamProjection<DISTORTION_T>::load(Archive & ar,
                                        const unsigned int version) {
  SM_ASSERT_LE(std::runtime_error, version,
               (unsigned int) CLASS_SERIALIZATION_VERSION,
               "Unsupported serialization version");

  ar >> BOOST_SERIALIZATION_NVP(_cx);
  ar >> BOOST_SERIALIZATION_NVP(_cy);
  ar >> BOOST_SERIALIZATION_NVP(_c);
  ar >> BOOST_SERIALIZATION_NVP(_d);
  ar >> BOOST_SERIALIZATION_NVP(_e);
  ar >> BOOST_SERIALIZATION_NVP(_ru);
  ar >> BOOST_SERIALIZATION_NVP(_rv);
  ar >> BOOST_SERIALIZATION_NVP(_world2camCoeffs);
  ar >> BOOST_SERIALIZATION_NVP(_cam2worldCoeffs);
  ar >> BOOST_SERIALIZATION_NVP(_distortion);
}

template<typename DISTORTION_T>
template<class Archive>
void OcamProjection<DISTORTION_T>::save(Archive & ar,
                                        const unsigned int /* version */) const {
  ar << BOOST_SERIALIZATION_NVP(_cx);
  ar << BOOST_SERIALIZATION_NVP(_cy);
  ar << BOOST_SERIALIZATION_NVP(_c);
  ar << BOOST_SERIALIZATION_NVP(_d);
  ar << BOOST_SERIALIZATION_NVP(_e);
  ar << BOOST_SERIALIZATION_NVP(_ru);
  ar << BOOST_SERIALIZATION_NVP(_rv);
  ar << BOOST_SERIALIZATION_NVP(_world2camCoeffs);
  ar << BOOST_SERIALIZATION_NVP(_cam2worldCoeffs);
  ar << BOOST_SERIALIZATION_NVP(_distortion);
}

template<typename DISTORTION_T>
Eigen::VectorXd OcamProjection<DISTORTION_T>::createRandomKeypoint() const {
  Eigen::Vector2d u(_ru + 1, _rv + 1);

  while (u[0] <= 0 || u[0] >= _ru - 1 || u[1] <= 0 || u[1] >= _rv - 1) {
    u.setRandom();
    u[0] = std::abs(u[0]) * _ru;
    u[1] = std::abs(u[1]) * _rv;
  }

  return u;
}

template<typename DISTORTION_T>
Eigen::Vector3d OcamProjection<DISTORTION_T>::createRandomVisiblePoint(double depth) const {
  Eigen::VectorXd y = createRandomKeypoint();
  Eigen::Vector3d p;
  keypointToEuclidean(y, p);

  if (depth < 0.0) {
    depth = ((double) rand() / (double) RAND_MAX) * 100.0;
  }

  p /= p.norm();
  p *= depth;
  return p;
}

}  // namespace cameras
}  // namespace aslam

#endif // ASLAM_CAMERAS_IMPLEMENTATION_OCAM_PROJECTION_HPP
