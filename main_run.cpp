#include "sfm_data_load_help.h"
#include "rotation_help.h"
#include "camera_lsy.h"
#include "linear_constraint_help.h"
#include "/home/lsy/openMVG/src/software/SfM/tools_precisionEvaluationToGt.hpp"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace std;

int main()
{
    string sSfM_Data_Filename = "/media/lsy/doc_disk/benchmark_img/castle-P19/sfm_data.json";
    string sMatchesDir = "/media/lsy/doc_disk/benchmark_img/castle-P19/matches";
    string sSfM_Data_Final_Filename = "/media/lsy/doc_disk/benchmark_img/castle-P19/global_openMVG/sfm_data.bin";
    string gt_dir = "/media/lsy/doc_disk/benchmark_img/castle-P19";

    //获取数据
    SfM_Data sfm_data, sfm_data_final;
    if(!load_sfm_data(sSfM_Data_Filename, sfm_data))
    {
        cout << "获取sfm_data失败" << endl;
        return EXIT_FAILURE;
    }

    //更改内参
//    std::shared_ptr<camera_lsy> cam_lsy = std::make_shared<camera_lsy>(3072, 2048, (2759.48+2764.16)/2,
//                                                                       1520.69, 1006.81);
//    Mat3 K_lsy;
//    K_lsy << 2759.48, 0, 1520.69, 0, 2764.16, 1006.81, 0, 0, 1;
//    cam_lsy->setK(K_lsy);
//    sfm_data.intrinsics.at(0) = cam_lsy;

    if(!Load(sfm_data_final, sSfM_Data_Final_Filename, ESfM_Data(VIEWS|INTRINSICS|EXTRINSICS) ))
    {
        cout << "获取sfm_data_finanl失败" << endl;
        return EXIT_FAILURE;
    }

    std::shared_ptr<Features_Provider> feats_provider;
    std::shared_ptr<Matches_Provider> matches_provider;
    if(!load_features_matches_provider(sMatchesDir, sfm_data, feats_provider, matches_provider))
    {
        cout << "获取特征点及其匹配数据失败" << endl;
        return EXIT_FAILURE;
    }

    //获取真值数据
    std::vector<Vec3> vec_camPosGT;
    std::vector<Mat3> vec_camRotGT;
    string image_dir = gt_dir;
    load_gt_data(gt_dir, image_dir, vec_camPosGT, vec_camRotGT);

    //获取opemMVG数据
    std::vector<Vec3> vec_camPosOPENMVG;
    std::vector<Mat3> vec_camRotOPENMVG;
    for(int i=0; i<sfm_data_final.GetPoses().size(); ++i)
    {
        vec_camPosOPENMVG.push_back( sfm_data_final.GetPoses().at(i).center() );
        vec_camRotOPENMVG.push_back( sfm_data_final.GetPoses().at(i).rotation() );
    }


    //计算相对旋转
    openMVG::rotation_averaging::RelativeRotations relatives_R;
    Hash_Map<Pair, RelativePose_Info> relativePose_Infos_map;
    if( !compute_relativePose(relatives_R, relativePose_Infos_map, sfm_data, feats_provider, matches_provider) )
    {
        cout << "计算相对位姿失败" << endl;
        return EXIT_FAILURE;
    }

    //计算绝对旋转
    Hash_Map<IndexT, Mat3> global_rotations;
    if (!Compute_Global_Rotations(relatives_R, global_rotations))
    {
        std::cerr << "GlobalSfM:: Rotation Averaging failure!" << std::endl;
        return false;
    }


    //选择点构建线性约束方程
//    test_constraints(relatives_R, global_rotations, relativePose_Infos_map, sfm_data, feats_provider, matches_provider);
    vector<vector<double>> A;
    int number_select = 50;
    get_linear_equation(A, number_select, relatives_R, global_rotations, relativePose_Infos_map, sfm_data, feats_provider, matches_provider);

    //用真值测试 A
    const bool test = false;
    if(test)
    {
        Mat A_eigen(A.size(), A[0].size());
        for(int r=0; r<A.size(); ++r)
        for(int c=0; c<A[0].size(); ++c)
        {
            A_eigen(r, c) = A[r][c];
        }
        cout << "A_rows = " << A_eigen.rows() << " " << "rank(A) = " << A_eigen.fullPivHouseholderQr().rank() << endl;

        Vec t(A[0].size());
        for(int i=0; i<global_rotations.size(); ++i)
        {
            Vec3 ti = sfm_data_final.GetPoses().at(i).translation();
            t(3*i) = ti(0);
            t(3*i+1) = ti(1);
            t(3*i+2) = ti(2);
        }

        Vec res_error = A_eigen * t;
        cout << "归一化前误差的范数：" << res_error.norm() << ", t_norm = " << t.norm() << "; ";
        t.normalize();
        res_error = A_eigen * t;
        cout << "归一化后误差的范数：" << res_error.norm() << ", t_norm = " << t.norm() <<endl;;

    }

    //把第一个相机的平移定为(0, 0, 0)
    Vec t_all(3*global_rotations.size());
    {
        Mat A_eigen(A.size(), A[0].size()-3);
        for(int r=0; r<A.size(); ++r)
            for(int c=0; c<A[0].size()-3; ++c)
            {
                A_eigen(r, c) = A[r][c+3];
            }
        // Ax = 0
        Mat AAT = A_eigen.transpose() * A_eigen;
        cout << "AAT.rows()" << AAT.rows() << ", rank(AAT)" << AAT.fullPivHouseholderQr().rank() << endl;
        Eigen::SelfAdjointEigenSolver<Mat> eigensolver(AAT);
        Vec t = eigensolver.eigenvectors().col(0);
        double min_eigen_vals = eigensolver.eigenvalues()[0];
        Vec t_test = AAT * t - min_eigen_vals * t;
//        cout << eigensolver.eigenvalues();
        cout << min_eigen_vals << " " << t_test.norm() << endl;

        Vec3 t0(0, 0, 0);
        t_all << t0, t;
    }

    Mat A_eigen(A.size(), A[0].size());
    for(int r=0; r<A.size(); ++r)
        for(int c=0; c<A[0].size(); ++c)
        {
            A_eigen(r, c) = A[r][c];
        }
    Vec t_error = A_eigen * t_all;
    cout << "t_error.norm() = " << t_error.norm() << endl;

    Vec t_openmvg(3*global_rotations.size());
    for(int i=0; i<global_rotations.size(); ++i)
    {
        Vec3 t3 = sfm_data_final.GetPoses().at(i).translation();
        t_openmvg(3*i) = t3(0);
        t_openmvg(3*i+1) = t3(1);
        t_openmvg(3*i+2) = t3(2);
    }
    t_error = A_eigen * t_openmvg;
    cout << "t_error_openmvg.norm() = " << t_error.norm() << endl;

    //t -> c
    std::vector<Vec3> vec_camPosComputed;
    std::vector<Mat3> vec_camRotComputed;
    for(int i=0; i<global_rotations.size(); ++i)
    {
        vec_camRotComputed.push_back(global_rotations[i]);
        Vec3 c3( t_all(3*i), t_all(3*i+1), t_all(3*i+2) );
        c3 = -global_rotations[i].transpose() * c3;
//        vec_camPosComputed.push_back(c3);
        vec_camPosComputed.push_back(sfm_data_final.GetPoses().at(i).center());
    }

    //计算结果精度
    std::string sOutPath = "/home/lsy/openMVG_Build/software/SfM/tutorial_out/benchmark/test";
    htmlDocument::htmlDocumentStream htmlDocStream("lsy sfm");
    openMVG::EvaluteToGT(vec_camPosGT, vec_camPosOPENMVG, vec_camRotGT, vec_camRotOPENMVG, sOutPath, &htmlDocStream);
//    openMVG::EvaluteToGT(vec_camPosGT, vec_camPosComputed, vec_camRotGT, vec_camRotComputed, sOutPath, &htmlDocStream);
}
