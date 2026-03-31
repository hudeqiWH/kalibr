/*
 * Test file for OcamProjection
 * Validates projection and back-projection consistency
 */

#include <gtest/gtest.h>
#include <aslam/cameras/OcamProjection.hpp>
#include <aslam/cameras/NoDistortion.hpp>
#include <Eigen/Core>
#include <vector>

using namespace aslam::cameras;

class OcamProjectionTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Example parameters from park_front.json
    cx = 961.44189453125;
    cy = 764.6656494140625;
    c = 0.9988817572593689;
    d = 0.00048447417793795466;
    e = -0.000787204597145319;
    
    // world2cam polynomial coefficients (projection 3D->2D)
    world2cam_coeffs = {
      829.33251953125, 637.10107421875, 51.23870849609375, 58.38129425048828,
      80.20258331298828, -4.643816947937012, 10.809462547302246, 51.23932647705078,
      3.196411371231079, -35.826011657714844, -21.63343048095703, -3.894087314605713
    };
    
    // cam2world polynomial coefficients (back-projection 2D->3D)
    cam2world_coeffs = {
      -414.9294128417969, 0.0, 0.0003165224625263363, 
      2.5380396095897595e-07, 1.1088343437881676e-10
    };
    
    width = 1920;
    height = 1536;
    
    // Create projection
    projection.reset(new OcamProjection<NoDistortion>(
      cx, cy, c, d, e,
      world2cam_coeffs, cam2world_coeffs,
      width, height, NoDistortion()
    ));
  }
  
  double cx, cy, c, d, e;
  std::vector<double> world2cam_coeffs;
  std::vector<double> cam2world_coeffs;
  int width, height;
  boost::shared_ptr<OcamProjection<NoDistortion>> projection;
};

TEST_F(OcamProjectionTest, testProjectionAndBackProjection) {
  // Test that back-projection after projection gives approximately the same 3D direction
  
  // Create test 3D points (in front of camera, z < 0 for Ocam convention)
  std::vector<Eigen::Vector3d> test_points = {
    Eigen::Vector3d(0.0, 0.0, -1.0),    // Center
    Eigen::Vector3d(0.5, 0.0, -1.0),    // Right
    Eigen::Vector3d(-0.5, 0.0, -1.0),   // Left
    Eigen::Vector3d(0.0, 0.5, -1.0),    // Bottom
    Eigen::Vector3d(0.0, -0.5, -1.0),   // Top
    Eigen::Vector3d(1.0, 1.0, -1.0),    // Corner
  };
  
  for (const auto& p3d : test_points) {
    Eigen::Vector2d keypoint;
    bool proj_success = projection->euclideanToKeypoint(p3d, keypoint);
    
    EXPECT_TRUE(proj_success) << "Projection failed for point: " << p3d.transpose();
    
    if (proj_success) {
      // Check keypoint is within image bounds
      EXPECT_GE(keypoint[0], 0) << "Keypoint x < 0";
      EXPECT_LT(keypoint[0], width) << "Keypoint x >= width";
      EXPECT_GE(keypoint[1], 0) << "Keypoint y < 0";
      EXPECT_LT(keypoint[1], height) << "Keypoint y >= height";
      
      // Back-project
      Eigen::Vector3d p3d_back;
      bool backproj_success = projection->keypointToEuclidean(keypoint, p3d_back);
      
      EXPECT_TRUE(backproj_success) << "Back-projection failed for keypoint: " << keypoint.transpose();
      
      if (backproj_success) {
        // Normalize both vectors and compare directions
        Eigen::Vector3d p3d_norm = p3d.normalized();
        Eigen::Vector3d p3d_back_norm = p3d_back.normalized();
        
        double dot_product = p3d_norm.dot(p3d_back_norm);
        EXPECT_GT(dot_product, 0.99) << "Direction mismatch. Original: " << p3d_norm.transpose() 
                                     << ", Back-projected: " << p3d_back_norm.transpose();
      }
    }
  }
}

TEST_F(OcamProjectionTest, testPrincipalPointProjection) {
  // Point along optical axis should project near principal point
  Eigen::Vector3d center_point(0.0, 0.0, -1.0);
  Eigen::Vector2d keypoint;
  
  bool success = projection->euclideanToKeypoint(center_point, keypoint);
  EXPECT_TRUE(success);
  
  if (success) {
    // Should be close to principal point
    EXPECT_NEAR(keypoint[0], cx, 5.0) << "Center projection x mismatch";
    EXPECT_NEAR(keypoint[1], cy, 5.0) << "Center projection y mismatch";
  }
}

TEST_F(OcamProjectionTest, testGetters) {
  EXPECT_DOUBLE_EQ(projection->cx(), cx);
  EXPECT_DOUBLE_EQ(projection->cy(), cy);
  EXPECT_DOUBLE_EQ(projection->c(), c);
  EXPECT_DOUBLE_EQ(projection->d(), d);
  EXPECT_DOUBLE_EQ(projection->e(), e);
  EXPECT_EQ(projection->ru(), width);
  EXPECT_EQ(projection->rv(), height);
  EXPECT_EQ(projection->world2camCoeffs().size(), world2cam_coeffs.size());
  EXPECT_EQ(projection->cam2worldCoeffs().size(), cam2world_coeffs.size());
}

TEST_F(OcamProjectionTest, testPolynomialEvaluation) {
  // Test that polynomial evaluation is working correctly
  // A simple test: evaluate at theta=0 should give the first coefficient
  
  // This is implicitly tested through projection, but we can add explicit tests here
  // by accessing the polynomial evaluation if we make it public
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
