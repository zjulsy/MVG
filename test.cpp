#include "openMVG/multiview/essential.hpp"
#include "openMVG/multiview/motion_from_essential.hpp"
#include "openMVG/multiview/projection.hpp"
#include "openMVG/multiview/solver_essential_five_point.hpp"
#include "openMVG/multiview/test_data_sets.hpp"
#include "openMVG/numeric/numeric.h"

#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/multiview/triangulation.hpp"

#include <iostream>
#include <random>

using namespace openMVG;
using namespace openMVG::sfm;

struct TestData {
  //-- Dataset that encapsulate :
  // 3D points and their projection given P1 and P2
  // Link between the two camera [R|t]
  Mat3X X;
  Mat3 R;
  Vec3 t;
  Mat3 E;
  Mat34 P1, P2;
  Mat2X x1, x2;
};

TestData SomeTestData() {
  TestData d;

  // --
  // Modeling random 3D points,
  // Consider first camera as [R=I|t=0],
  // Second camera as [R=Rx*Ry*Rz|t=random],
  // Compute projection of the 3D points onto image plane.
  // --
  const int num_p = 5;
  const int z_p = 3;
  Mat3X ps(3, num_p);
  for(int i=0; i<num_p; ++i)
  {
      std::random_device r;
      std::default_random_engine e1(r());
      std::uniform_real_distribution<double> uniform_dist(-z_p, z_p);

      ps.col(i)[0] = uniform_dist(e1);
      ps.col(i)[1] = uniform_dist(e1);
      ps.col(i)[2] = z_p;
  }

  d.X = ps;


  d.R = RotationAroundZ(0.3) * RotationAroundX(0.1) * RotationAroundY(0.2);
  d.t.setRandom();

  EssentialFromRt(Mat3::Identity(), Vec3::Zero(), d.R, d.t, &d.E);

  P_From_KRt(Mat3::Identity(), Mat3::Identity(), Vec3::Zero(), &d.P1);
  P_From_KRt(Mat3::Identity(), d.R, d.t, &d.P2);

  Project(d.P1, d.X, &d.x1);
  Project(d.P2, d.X, &d.x2);

  return d;
}

void testPlane()
{
    using namespace std;
    const TestData d = SomeTestData();
    cout << "get data" << endl;
    std::vector<Mat3> Es;
    FivePointsRelativePose(d.x1.colwise().homogeneous(),
                           d.x2.colwise().homogeneous(),
                           &Es);
    cout << "get es" << endl;
    // Recover the relative pose from E.
    std::vector<geometry::Pose3> relative_poses;
    for (size_t s = 0; s < Es.size(); ++s) {
      std::vector<uint32_t> index(d.x1.cols());
      std::iota(index.begin(), index.end(), 0);
      geometry::Pose3 relative_pose;
      if(RelativePoseFromEssential(
        d.x1.colwise().homogeneous(),
        d.x2.colwise().homogeneous(),
        Es[s],
        index,
        &relative_pose))
      {
        relative_poses.push_back(relative_pose);
      }
    }

    cout << "get poses" << endl;

    bool bsolution_found = false;
    for (size_t i = 0; i < relative_poses.size(); ++i) {

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(d.R, relative_poses[i].rotation()) < 1e-3
              && (d.t / d.t.norm()
                       - relative_poses[i].translation()
                         / relative_poses[i].translation().norm()).norm() < 1e-3)
      {
          bsolution_found = true;
          std::cout << "yes" << " ";
      }
    }
    //-- Almost one solution must find the correct relative orientation
}
