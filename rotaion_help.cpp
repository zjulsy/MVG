#include "rotation_help.h"
#include "camera_lsy.h"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace std;

bool compute_relativePose(openMVG::rotation_averaging::RelativeRotations& relatives_R,
                          Hash_Map<Pair, RelativePose_Info>& relativePose_Infos_map,
                          SfM_Data& sfm_data,
                          std::shared_ptr<Features_Provider>& feats_provider,
                          std::shared_ptr<Matches_Provider> &matches_provider)
{
    Relative_Pose_Engine::Relative_Pair_Poses relative_poses;

    //有多少个位置对
    Pair_Set relative_pose_pairs;
    for (const auto & iterMatches : matches_provider->pairWise_matches_)
    {
        const Pair pair = iterMatches.first;
        const View * v1 = sfm_data.GetViews().at(pair.first).get();
        const View * v2 = sfm_data.GetViews().at(pair.second).get();
        if (v1->id_pose != v2->id_pose)
          relative_pose_pairs.insert({v1->id_pose, v2->id_pose});
    }

    //每个位置对有多少相机对
    using PoseWiseMatches = Hash_Map<Pair, Pair_Set>;
    PoseWiseMatches posewise_matches;
    for (const auto & iterMatches : matches_provider->pairWise_matches_)
    {
        const Pair pair = iterMatches.first;
        const View * v1 = sfm_data.GetViews().at(pair.first).get();
        const View * v2 = sfm_data.GetViews().at(pair.second).get();
        if (v1->id_pose != v2->id_pose
            && relative_pose_pairs.count({v1->id_pose, v2->id_pose}) == 1)
        {
            Pair p(v1->id_pose, v2->id_pose);
            posewise_matches[p].insert(pair);
        }
    }

    // Compute the relative pose from pairwise point matches:
    for (int i = 0; i < static_cast<int>(posewise_matches.size()); ++i)
    {
        PoseWiseMatches::const_iterator iter (posewise_matches.begin());
        std::advance(iter, i);
        const auto & relative_pose_iterator(*iter);
        const Pair relative_pose_pair = relative_pose_iterator.first;
        const Pair_Set & match_pairs = relative_pose_iterator.second;

        // If a pair has the same ID, discard it
        if (relative_pose_pair.first == relative_pose_pair.second)
        {
            continue;
        }

        // Select common bearing vectors
        if (match_pairs.size() > 1)
        {
            std::cerr << "Compute relative pose between more than two view is not supported" << std::endl;
            continue;
        }

        const Pair current_pair(*std::begin(match_pairs));

        const IndexT
            I = current_pair.first,
            J = current_pair.second;

        const View
            * view_I = sfm_data.views.at(I).get(),
            * view_J = sfm_data.views.at(J).get();

        // Check that valid cameras exist for the view pair
        if (sfm_data.GetIntrinsics().count(view_I->id_intrinsic) == 0 || sfm_data.GetIntrinsics().count(view_J->id_intrinsic) == 0)
            continue;


        const IntrinsicBase
        * cam_I = sfm_data.GetIntrinsics().at(view_I->id_intrinsic).get(),
        * cam_J = sfm_data.GetIntrinsics().at(view_J->id_intrinsic).get();

        // Compute for each feature the un-distorted camera coordinates
        const matching::IndMatches & matches = matches_provider->pairWise_matches_.at(current_pair);
        size_t number_matches = matches.size();
        Mat2X x1(2, number_matches), x2(2, number_matches);
        number_matches = 0;
        for (const auto & match : matches)
        {
            x1.col(number_matches) = feats_provider->feats_per_view.at(I)[match.i_].coords().cast<double>();
            x2.col(number_matches++) = feats_provider->feats_per_view.at(J)[match.j_].coords().cast<double>();
//            x1.col(number_matches) = cam_I->get_ud_pixel(
//                feats_provider->feats_per_view.at(I)[match.i_].coords().cast<double>());
//            x2.col(number_matches++) = cam_J->get_ud_pixel(
//                feats_provider->feats_per_view.at(J)[match.j_].coords().cast<double>());
        }

        RelativePose_Info relativePose_info;
        relativePose_info.initial_residual_tolerance = Square(2.5);
        if (!robustRelativePose(cam_I, cam_J,
                                x1, x2, relativePose_info,
                                {cam_I->w(), cam_I->h()},
                                {cam_J->w(), cam_J->h()},
                                256))
        {
            continue;
        }

        //记录相对位姿，之后选点会用到
        relativePose_Infos_map[current_pair] = relativePose_info;

        relative_poses[relative_pose_pair] = relativePose_info.relativePose;
    }

    // Export the rotation component from the computed relative poses
    for (const auto & relative_pose : relative_poses)
    {
      // Add the relative rotation to the relative 'rotation' pose graph
      relatives_R.emplace_back(
        relative_pose.first.first, relative_pose.first.second,
        relative_pose.second.rotation(),
        1.f);
    }

    return true;
}

bool Compute_Global_Rotations
(
  const rotation_averaging::RelativeRotations & relatives_R,
  Hash_Map<IndexT, Mat3> & global_rotations
)
{
    Bundle_Adjustment_Ceres::BA_Ceres_options options(false, false);  //这一行看着没有用，但是去掉有问题

    if (relatives_R.empty())
        return false;

    // Log statistics about the relative rotation graph
      {
        std::set<IndexT> set_pose_ids;
        for (const auto & relative_R : relatives_R)
        {
          set_pose_ids.insert(relative_R.i);
          set_pose_ids.insert(relative_R.j);
        }

        std::cout << "\n-------------------------------" << "\n"
          << " Global rotations computation: " << "\n"
          << "  #relative rotations: " << relatives_R.size() << "\n"
          << "  #global rotations: " << set_pose_ids.size() << std::endl;
      }

  // Global Rotation solver:
  const ERelativeRotationInferenceMethod eRelativeRotationInferenceMethod =
    TRIPLET_ROTATION_INFERENCE_COMPOSITION_ERROR;
    //TRIPLET_ROTATION_INFERENCE_NONE;

  system::Timer t;
  GlobalSfM_Rotation_AveragingSolver rotation_averaging_solver;
  // Set default motion Averaging methods
  ERotationAveragingMethod eRotation_averaging_method_ = ROTATION_AVERAGING_L2;

  const bool b_rotation_averaging = rotation_averaging_solver.Run(
    eRotation_averaging_method_, eRelativeRotationInferenceMethod,
    relatives_R, global_rotations);

  std::cout
    << "Found #global_rotations: " << global_rotations.size() << "\n"
    << "Timing: " << t.elapsed() << " seconds" << std::endl;

  return b_rotation_averaging;
}
