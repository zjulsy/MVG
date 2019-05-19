#ifndef ROTATION_HELP_H
#define ROTATION_HELP_H

#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"
#include "openMVG/sfm/pipelines/global/GlobalSfM_rotation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/GlobalSfM_translation_averaging.hpp"
#include "openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"

#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/system/timer.hpp"

#include "openMVG/sfm/pipelines/relative_pose_engine.hpp"

#include "openMVG/multiview/essential.hpp"
#include "openMVG/multiview/triangulation.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
#include "openMVG/sfm/sfm_data_triangulation.hpp"
#include "openMVG/robust_estimation/rand_sampling.hpp"
#include "third_party/htmlDoc/htmlDoc.hpp"



#include <cstdlib>
#include <memory>
#include <string>
#include <random>


using namespace openMVG;
using namespace openMVG::sfm;
using namespace std;

bool compute_relativePose(openMVG::rotation_averaging::RelativeRotations& relatives_R,
                          Hash_Map<Pair, RelativePose_Info>& relativePose_Infos_map,
                          SfM_Data& sfm_data,
                          std::shared_ptr<Features_Provider>& feats_provider,
                          std::shared_ptr<Matches_Provider> &matches_provider);

bool Compute_Global_Rotations
(
  const rotation_averaging::RelativeRotations & relatives_R,
  Hash_Map<IndexT, Mat3> & global_rotations
);
#endif // ROTATION_HELP_H
