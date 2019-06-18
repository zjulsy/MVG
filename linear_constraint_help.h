#ifndef LINEAR_CONSTRAINT_HELP_H
#define LINEAR_CONSTRAINT_HELP_H

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
#include "openMVG/sfm/sfm_data_triangulation.hpp"
#include "openMVG/robust_estimation/rand_sampling.hpp"

#include <cstdlib>
#include <memory>
#include <string>
#include <random>
#include "camera_lsy.h"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace std;

void test_constraints(openMVG::rotation_averaging::RelativeRotations& relatives_R,
                      Hash_Map<IndexT, Mat3>& global_rotations,
                      Hash_Map<Pair, RelativePose_Info>& relativePose_Infos_map,
                      SfM_Data& sfm_data,
                      std::shared_ptr<Features_Provider>& feats_provider,
                      std::shared_ptr<Matches_Provider> &matches_provider);

void get_linear_equation(vector<vector<double>>& A,
                         int number_select,
                         openMVG::rotation_averaging::RelativeRotations& relatives_R,
                         Hash_Map<IndexT, Mat3>& global_rotations,
                         Hash_Map<Pair, RelativePose_Info>& relativePose_Infos_map,
                         SfM_Data& sfm_data,
                         std::shared_ptr<Features_Provider>& feats_provider,
                         std::shared_ptr<Matches_Provider> &matches_provider);

void test_constraints2();

Mat directGetT(Hash_Map<Pair, RelativePose_Info>& relativePose_Infos_map, Hash_Map<IndexT, Mat3>& global_rotations, vector<double>& ts,  vector<Mat3>& vec_Rij, vector<Vec3>& vec_tij, vector< std::pair<int, int> >& vec_ij);
#endif // LINEAR_CONSTRAINT_HELP_H
