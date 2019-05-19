#ifndef SFM_DATA_LOAD_HELP_H
#define SFM_DATA_LOAD_HELP_H

#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/system/timer.hpp"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace std;

bool load_sfm_data(string sSfM_Data_Filename, SfM_Data& sfm_data);

bool load_features_matches_provider(string sMatchesDir, SfM_Data& sfm_data, std::shared_ptr<Features_Provider>& feats_provider, std::shared_ptr<Matches_Provider> &matches_provider);

bool load_gt_data(string & gt_dir, string &image_dir, vector<Vec3>& vec_camPosGT, vector<Mat3>& vec_camRotGT);

#endif // SFM_DATA_LOAD_HELP_H
