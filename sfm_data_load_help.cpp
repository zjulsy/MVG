#include "sfm_data_load_help.h"
#include "/home/lsy/openMVG/src/software/SfM/import/io_readGTStrecha.hpp"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace std;

bool load_sfm_data(string sSfM_Data_Filename, SfM_Data& sfm_data)
{
    if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
        std::cerr << std::endl
          << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
        return false;
        }
    return true;
}

bool load_features_matches_provider(string sMatchesDir, SfM_Data& sfm_data, std::shared_ptr<Features_Provider>& feats_provider, std::shared_ptr<Matches_Provider> &matches_provider)
{
    using namespace openMVG::features;
    const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
    std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
    if (!regions_type)
    {
        std::cerr << "Invalid: "
          << sImage_describer << " regions type file." << std::endl;
        return false;
    }

    // Features reading
    feats_provider = std::make_shared<Features_Provider>();
    if (!feats_provider->load(sfm_data, sMatchesDir, regions_type)) {
        std::cerr << std::endl
          << "Invalid features." << std::endl;
        return false;
    }

    // Matches reading
    matches_provider = std::make_shared<Matches_Provider>();
    if // Try to read the provided match filename or the default one (matches.e.txt/bin)
    (
    !(matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.e.txt")) ||
      matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.e.bin")) ||
      matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.f.bin")) ||
      matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.f.txt")))
    )
    {
        std::cerr << std::endl
          << "Invalid matches file." << std::endl;
        return false;
    }
    return true;
}

bool load_gt_data(std::string & gt_dir, std::string &image_dir, std::vector<Vec3>& vec_camPosGT, std::vector<Mat3>& vec_camRotGT)
{
    SfM_Data_GT_Loader_Strecha sfm_gt_loader;
    sfm_gt_loader.run(gt_dir, image_dir);

    for(int i=0; i<sfm_gt_loader.cameras_data_.size(); ++i)
    {
        vec_camPosGT.push_back(sfm_gt_loader.cameras_data_[i]._C);
        vec_camRotGT.push_back(sfm_gt_loader.cameras_data_[i]._R);
    }
    return true;
}



