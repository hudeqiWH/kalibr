#ifndef ASLAM_CAMERAS_OCAM_PROJECTION_HPP
#define ASLAM_CAMERAS_OCAM_PROJECTION_HPP

#include "StaticAssert.hpp"
#include "FiniteDifferences.hpp"
#include <sm/PropertyTree.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/boost/serialization.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sm/logging.hpp>
#include <vector>

namespace aslam {
namespace cameras {

template<typename DISTORTION_T>
class OcamProjection {
 public:

  enum {
    KeypointDimension = 2
  };
  // cx, cy, c, d, e + world2cam coeffs + cam2world coeffs
  // Note: IntrinsicsDimension is variable for Ocam, we use max size
  enum {
    IntrinsicsDimension = 3 + 3 + 20  // cx,cy + c,d,e + max 20 coeffs (10 each)
  };
  enum {
    DesignVariableDimension = IntrinsicsDimension
  };

  typedef DISTORTION_T distortion_t;
  typedef Eigen::Matrix<double, KeypointDimension, 1> keypoint_t;
  typedef Eigen::Matrix<double, KeypointDimension, IntrinsicsDimension> jacobian_intrinsics_t;

  /// \brief Default constructor
  OcamProjection();

  /// \brief Constructor with all parameters
  OcamProjection(double cx, double cy, double c, double d, double e,
                 const std::vector<double> & world2camCoeffs,
                 const std::vector<double> & cam2worldCoeffs,
                 int resolutionU, int resolutionV,
                 distortion_t distortion = distortion_t());

  /// \brief Constructor from property tree
  OcamProjection(const sm::PropertyTree & config);

  /// \brief destructor.
  virtual ~OcamProjection();

  /// \brief resize the intrinsics based on a scaling of the image.
  void resizeIntrinsics(double scale);

  template<typename DERIVED_P, typename DERIVED_K>
  bool euclideanToKeypoint(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypoint) const;

  template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool euclideanToKeypoint(const Eigen::MatrixBase<DERIVED_P> & p,
                           const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
                           const Eigen::MatrixBase<DERIVED_JP> & outJp) const;

  template<typename DERIVED_P, typename DERIVED_K>
  bool homogeneousToKeypoint(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_K> & outKeypoint) const;

  template<typename DERIVED_P, typename DERIVED_K, typename DERIVED_JP>
  bool homogeneousToKeypoint(const Eigen::MatrixBase<DERIVED_P> & p,
                             const Eigen::MatrixBase<DERIVED_K> & outKeypoint,
                             const Eigen::MatrixBase<DERIVED_JP> & outJp) const;

  template<typename DERIVED_K, typename DERIVED_P>
  bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K> & keypoint,
                           const Eigen::MatrixBase<DERIVED_P> & outPoint) const;

  template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
  bool keypointToEuclidean(const Eigen::MatrixBase<DERIVED_K> & keypoint,
                           const Eigen::MatrixBase<DERIVED_P> & outPoint,
                           const Eigen::MatrixBase<DERIVED_JK> & outJk) const;

  template<typename DERIVED_K, typename DERIVED_P>
  bool keypointToHomogeneous(
      const Eigen::MatrixBase<DERIVED_K> & keypoint,
      const Eigen::MatrixBase<DERIVED_P> & outPoint) const;

  template<typename DERIVED_K, typename DERIVED_P, typename DERIVED_JK>
  bool keypointToHomogeneous(const Eigen::MatrixBase<DERIVED_K> & keypoint,
                             const Eigen::MatrixBase<DERIVED_P> & outPoint,
                             const Eigen::MatrixBase<DERIVED_JK> & outJk) const;

  template<typename DERIVED_P, typename DERIVED_JI>
  void euclideanToKeypointIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JI> & outJi) const;

  template<typename DERIVED_P, typename DERIVED_JD>
  void euclideanToKeypointDistortionJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  template<typename DERIVED_P, typename DERIVED_JI>
  void homogeneousToKeypointIntrinsicsJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JI> & outJi) const;

  template<typename DERIVED_P, typename DERIVED_JD>
  void homogeneousToKeypointDistortionJacobian(
      const Eigen::MatrixBase<DERIVED_P> & p,
      const Eigen::MatrixBase<DERIVED_JD> & outJd) const;

  template<typename DERIVED_K>
  bool isValid(const Eigen::MatrixBase<DERIVED_K> & keypoint) const;

  template<typename DERIVED_K>
  bool isLiftable(const Eigen::MatrixBase<DERIVED_K> & keypoint) const;

  template<typename DERIVED_P>
  bool isEuclideanVisible(const Eigen::MatrixBase<DERIVED_P> & p) const;

  template<typename DERIVED_P>
  bool isHomogeneousVisible(const Eigen::MatrixBase<DERIVED_P> & ph) const;

  /// \brief initialize the intrinsics based on one view of a gridded calibration target
  /// \return true on success
  bool initializeIntrinsics(const std::vector<GridCalibrationTargetObservation> &observations);

  /// \brief estimate the transformation of the camera with respect to the calibration target
  ///        On success out_T_t_c is filled in with the transformation that takes points from
  ///        the camera frame to the target frame
  /// \return true on success
  bool estimateTransformation(const GridCalibrationTargetObservation & obs,
                              sm::kinematics::Transformation & out_T_t_c) const;

  /// \brief compute the reprojection error based on a checkerboard observation.
  /// \return the number of corners successfully observed and projected
  size_t computeReprojectionError(
      const GridCalibrationTargetObservation & obs,
      const sm::kinematics::Transformation & T_target_camera,
      double & outErr) const;

  // aslam::backend compatibility
  void update(const double * v);
  int minimalDimensions() const;
  void getParameters(Eigen::MatrixXd & P) const;
  void setParameters(const Eigen::MatrixXd & P);
  Eigen::Vector2i parameterSize() const;

  enum {
    CLASS_SERIALIZATION_VERSION = 0
  };
  BOOST_SERIALIZATION_SPLIT_MEMBER();
  template<class Archive>
  void load(Archive & ar, const unsigned int version);
  template<class Archive>
  void save(Archive & ar, const unsigned int version) const;

  /// \brief creates a random valid keypoint.
  virtual Eigen::VectorXd createRandomKeypoint() const;

  /// \brief creates a random visible point. Negative depth means random between 0 and 100 meters.
  virtual Eigen::Vector3d createRandomVisiblePoint(double depth = -1.0) const;

  /// \brief is the projection invertible?
  bool isProjectionInvertible() const {
    return true;
  }

  /// \brief set the distortion model.
  void setDistortion(const distortion_t & distortion) {
    _distortion = distortion;
  }
  /// \brief returns a reference to the distortion model
  distortion_t & distortion() {
    return _distortion;
  }
  const distortion_t & distortion() const {
    return _distortion;
  }

  /// \brief Getters for Ocam parameters
  double cx() const { return _cx; }
  double cy() const { return _cy; }
  double c() const { return _c; }
  double d() const { return _d; }
  double e() const { return _e; }
  const std::vector<double> & world2camCoeffs() const { return _world2camCoeffs; }
  const std::vector<double> & cam2worldCoeffs() const { return _cam2worldCoeffs; }

  /// \brief The horizontal resolution in pixels.
  int ru() const { return _ru; }
  /// \brief The vertical resolution in pixels.
  int rv() const { return _rv; }
  int width() const { return _ru; }
  int height() const { return _rv; }

  double focalLengthCol() const { return (_ru + _rv) / 2.0; }
  double focalLengthRow() const { return (_ru + _rv) / 2.0; }
  double opticalCenterCol() const { return _cx; }
  double opticalCenterRow() const { return _cy; }

  int keypointDimension() const { return KeypointDimension; }

  bool isBinaryEqual(const OcamProjection<distortion_t> & rhs) const;

  static OcamProjection<distortion_t> getTestProjection();

 private:
  // Ocam parameters
  double _cx;  // principal point x
  double _cy;  // principal point y
  double _c;   // affine parameter c
  double _d;   // affine parameter d
  double _e;   // affine parameter e
  std::vector<double> _world2camCoeffs;  // polynomial coefficients for world2cam
  std::vector<double> _cam2worldCoeffs;  // polynomial coefficients for cam2world
  int _ru;   // horizontal resolution
  int _rv;   // vertical resolution

  distortion_t _distortion;

  /// \brief Evaluate polynomial using Horner's method
  double evaluatePolynomial(const std::vector<double> & coeffs, double x) const;
  /// \brief Evaluate polynomial derivative using Horner's method
  double evaluatePolynomialDerivative(const std::vector<double> & coeffs, double x) const;

};

}  // namespace cameras
}  // namespace aslam

#include "implementation/OcamProjection.hpp"

SM_BOOST_CLASS_VERSION_T1 (aslam::cameras::OcamProjection);

#endif /* ASLAM_CAMERAS_OCAM_PROJECTION_HPP */
