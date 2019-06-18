#include "sfm_data_load_help.h"
#include "rotation_help.h"
#include "camera_lsy.h"
#include "linear_constraint_help.h"
#include "/home/lsy/openMVG/src/software/SfM/tools_precisionEvaluationToGt.hpp"
#include "test.h"

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
    std::vector<Vec3> vec_camTsOPENMVG;
    std::vector<Mat3> vec_camRotOPENMVG;
    for(int i=0; i<sfm_data_final.GetPoses().size(); ++i)
    {
        vec_camPosOPENMVG.push_back( sfm_data_final.GetPoses().at(i).center() );
        vec_camRotOPENMVG.push_back( sfm_data_final.GetPoses().at(i).rotation() );
        vec_camTsOPENMVG.push_back( sfm_data_final.GetPoses().at(i).translation() );
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
        return EXIT_FAILURE;
    }



//    testPlane();
//    return  0;

    //test tij error
//    {
//        for(auto& it : relativePose_Infos_map)
//        {
//            Pair p = it.first;
//            int i = p.first,
//                j = p.second;

//            Vec3 tij = it.second.relativePose.translation();
//            Vec3 tij_2 = vec_camTsOPENMVG[j] - vec_camRotOPENMVG[j] * vec_camRotOPENMVG[i].transpose() * vec_camTsOPENMVG[i];

//            double diff_angle = acos( tij.dot(tij_2) / tij.norm() / tij_2.norm() ) * 180 / 3.14;
//            double distance = (vec_camPosOPENMVG[i] - vec_camPosOPENMVG[j]).norm();

////            Mat3 Ri = global_rotations[i];
//            Mat3 Ri = vec_camRotOPENMVG[i];
////            Mat3 Rj = global_rotations[j];
//            Mat3 Rj = vec_camRotOPENMVG[j];
//            Mat3 Rot_To_Identity = it.second.relativePose.rotation() * Ri * Rj.transpose(); // motion composition

//            float angularErrorDegree = static_cast<float>(R2D(getRotationMagnitude(Rot_To_Identity)));

//            Mat3 Eij = it.second.essential_matrix;
//            Eij = Eij/Eij.norm();

//            Mat3 Rij_2 = Rj * Ri.transpose();
////            Mat3 Rij_2 = it.second.relativePose.rotation();
//            Mat3 Eij2;
//            Eij2 << CrossProductMatrix(tij_2) * Rij_2;
//            Eij2 = Eij2/Eij2.norm();


//            const matching::IndMatches & matches = matches_provider->pairWise_matches_.at({i, j});
//            std::vector<uint32_t> vec_inliers = it.second.vec_inliers;
//            size_t number_valid_matches = vec_inliers.size();
//            size_t number_valid_matches2 = matches.size();
//            Mat2X xi(2, number_valid_matches), xj(2, number_valid_matches);

//            const View
//                * view_I = sfm_data.views.at(i).get(),
//                * view_J = sfm_data.views.at(j).get();

//            const IntrinsicBase
//                * cam_I = sfm_data.GetIntrinsics().at(view_I->id_intrinsic).get(),
//                * cam_J = sfm_data.GetIntrinsics().at(view_J->id_intrinsic).get();
//            const Mat3& K = dynamic_cast<const cameras::Pinhole_Intrinsic*>(cam_I)->K();

//            double err1 = 0, err2 = 0;
//            for (int i_num=0; i_num<number_valid_matches; ++i_num)
//            {
//                xi.col(i_num) = feats_provider->feats_per_view.at(i)[matches[vec_inliers[i_num]].i_].coords().cast<double>();
//                xj.col(i_num) = feats_provider->feats_per_view.at(j)[matches[vec_inliers[i_num]].j_].coords().cast<double>();

//                Vec3 pj, pi;
//                pi = xi.col(i_num).homogeneous();
//                pj = xj.col(i_num).homogeneous();
////                pi = K.inverse() * xi.col(i_num).homogeneous();
////                pj = K.inverse() * xj.col(i_num).homogeneous();

//                Mat3 Fij = K.inverse().transpose() * Eij * K.inverse();
//                Mat3 Fij2 = K.inverse().transpose() * Eij2 * K.inverse();
//                Vec3 li = (Fij * pi).normalized();
//                Vec3 lj = (Fij2 * pi).normalized();
//                err1 += abs( pj.transpose() * li );
//                err2 += abs( pj.transpose() * lj );
//            }

//            cout << i << " " << j << " " << diff_angle << " " << distance << " " << angularErrorDegree
//                 << " " << err1 << " " << err2 << " " << number_valid_matches << " " << number_valid_matches2 << endl;
//        }
//    }
//    return 0;

    // directly use tij to get ti , tj
    vector<double> ts(global_rotations.size()*3, 0);
    vector<Mat3> vec_Rij;
    vector<Vec3> vec_tij;
    vector< std::pair<int, int> > vec_ij;
    Mat A2 = directGetT(relativePose_Infos_map, global_rotations, ts, vec_Rij, vec_tij, vec_ij);
    Mat A22 = A2;
    int flag = 0;
    while(1)
    {
        flag++;
        cout << flag << "********" << "\n";
        Eigen::JacobiSVD<Mat> svd( A2, Eigen::ComputeFullV );
        Vec nullspace = svd.matrixV().col( A2.cols() - 1 );

        for(int i=0; i<ts.size()-3; ++i)
            ts[i+3] = nullspace(i);

        //t -> c
        std::vector<Vec3> vec_camPosComputed;
        std::vector<Mat3> vec_camRotComputed;

        //rot ambiguity
        Mat3 R_global, R1, R2;
        Vec3 t0, t1, t2, t01, t02, t12;
        int i = 0, j = 1, k = 3;
        t0 << ts[3*i], ts[3*i+1], ts[3*i+2];
        t1 << ts[3*j], ts[3*j+1], ts[3*j+2];
        t2 << ts[3*k], ts[3*k+1], ts[3*k+2];

        t01 = global_rotations[j].transpose() * relativePose_Infos_map.at({i, j}).relativePose.translation();
        t02 = global_rotations[k].transpose() * relativePose_Infos_map.at({i, k}).relativePose.translation();
        t12 = global_rotations[k].transpose() * relativePose_Infos_map.at({j, k}).relativePose.translation();

//        R1 << (global_rotations[j].transpose() * t1 - global_rotations[i].transpose() * t0).normalized(),
//              (global_rotations[k].transpose() * t2 - global_rotations[i].transpose() * t0).normalized(),
//              (global_rotations[k].transpose() * t2 - global_rotations[j].transpose() * t1).normalized();
//        R2 << t01.normalized(), t02.normalized(), t12.normalized();
//        cout << R1 << "\n" << R2 << endl;
//        R_global = R2 * R1.transpose();
//        cout << R_global << endl;
//        float angularErrorDegree = static_cast<float>(R2D(getRotationMagnitude(R_global)));
//        cout << angularErrorDegree << endl;

        if( (t1-relativePose_Infos_map.at({0, 1}).relativePose.rotation()*t0).transpose() * relativePose_Infos_map.at({0, 1}).relativePose.translation() < 0)
            for(auto& t : ts)
                t = -t;

//        cout << (t1-relativePose_Infos_map.at({0, 1}).relativePose.rotation()*t0).normalized() << "\n" << relativePose_Infos_map.at({0, 1}).relativePose.translation() << endl;

        // t -> c
//        vector<double> cs;
//        for(int i=0; i<global_rotations.size(); ++i)
//        {
//            Vec3 t3( ts[3*i], ts[3*i+1], ts[3*i+2] );
//            Vec3 c3 = -global_rotations[i].transpose() * t3;
//            cs.push_back(c3(0));
//            cs.push_back(c3(1));
//            cs.push_back(c3(2));
//        }

        //rot ambiguity : Ri.T  * cij = cj - ci



        for(int i=0; i<global_rotations.size(); ++i)
        {
            vec_camRotComputed.push_back(global_rotations[i]);
//            Vec3 c3( cs[3*i], cs[3*i+1], cs[3*i+2] );
            Vec3 c3( ts[3*i], ts[3*i+1], ts[3*i+2] );
//            c3 = R_global.transpose() * c3;
            c3 = -global_rotations[i].transpose() * c3;
            vec_camPosComputed.push_back(c3);
    //        vec_camPosComputed.push_back(sfm_data_final.GetPoses().at(i).center());
        }
        //计算结果精度
        std::string sOutPath = "/home/lsy/openMVG_Build/software/SfM/tutorial_out/benchmark/test";
        htmlDocument::htmlDocumentStream htmlDocStream("lsy sfm");
    //    openMVG::EvaluteToGT(vepush_backc_camPosGT, vec_camPosOPENMVG, vec_camRotGT, vec_camRotOPENMVG, sOutPath, &htmlDocStream);
        openMVG::EvaluteToGT(vec_camPosOPENMVG, vec_camPosComputed, vec_camRotOPENMVG, vec_camRotComputed, sOutPath, &htmlDocStream);

//        string s = "****** ******";
//        cout << s << endl;
        Vec err(vec_ij.size() * 3);
        for(int r=0; r<err.size()/3; ++r)
        {
            int i = vec_ij[r].first, j = vec_ij[r].second;
            Vec3 tj, ti;
            ti << ts[3*i], ts[3*i+1], ts[3*i+2];
            tj << ts[3*j], ts[3*j+1], ts[3*j+2];
            Mat3 Rij = vec_Rij[r];
            Vec3 tij = vec_tij[r];

            Vec3 tij2 = tj - Rij * ti;

//            cout << r << " " << tij.norm() << " " << tij2.norm() << " ";
            double a = tij.dot(tij2) / tij.norm() / tij2.norm();
//            double a = tij.dot(tij2) / tij.norm() / tij2.norm();
            if(a>0.9999)
                a = 1;
            if(a<-0.9999)
                a = -1;

            err(3*r) = acos( a ) * 180 / 3.14;
            err(3*r + 1) = err(3*r);
            err(3*r + 2) = err(3*r);
        }

        cout << "*******" << err.norm() << endl;

//        for(int r=0; r<err.size()/3; ++r)
//        {
//            int i = vec_ij[r].first, j = vec_ij[r].second;
//            Vec3 tj, ti;
//            ti << ts[3*i], ts[3*i+1], ts[3*i+2];
//            tj << ts[3*j], ts[3*j+1], ts[3*j+2];
//            Mat3 Rij = vec_Rij[r];
//            Vec3 tij = vec_tij[r];

//            Vec3 tij2 = tj - Rij * ti;

//            err(3*r) = tij2.norm();
//            err(3*r + 1) = err(3*r);
//            err(3*r + 2) = err(3*r);
//        }

//        cout << "*******" << err.norm() << endl;

//        Vec ts2(ts.size()-3);
//        for(int i=0; i<ts.size()-3; ++i)
//            ts2[i] = ts[i+3];

//        err = A2 * ts2;
//        cout << "*******" << err.norm() << endl;

        Vec w(err.size());
        for(int i=0; i<err.size(); ++i)
        {
//            w(i) = 1 / max(0.0001, err(i));
            w(i) = 1.0 / sqrt( max( 1.0, abs(err(i)) ) );
//             w(i) = 1.0 / sqrt( max( 0.01, abs(err(i)) ) );
        }

        for(int i=0; i<A2.rows(); ++i)
        {
            A2.row(i) = A22.row(i) * w(i);
        }

//        Vec ts2(ts.size()-3);
//        for(int i=0; i<ts.size()-3; ++i)
//            ts2[i] = ts[i+3];

//        Vec err2 = A2 * ts2;
//        cout << "*******" << err2.norm() << endl;

        if(flag==10)
            break;

//        for(auto& it : relativePose_Infos_map)
//        {
//            Pair current_pair = it.first;
//            int i = current_pair.first;
//            int j= current_pair.second;

//            Vec3 tij = it.second.relativePose.translation();

//            Vec3 ti( ts[3*i], ts[3*i+1], ts[3*i+2] );
//            Vec3 tj( ts[3*j], ts[3*j+1], ts[3*j+2] );
//            Mat3 Ri = global_rotations[i];
//            Mat3 Rj = global_rotations[j];

//            Vec3 t = ( tj - Rj * Ri.transpose() * ti ).normalized();


//            Vec3 ti2 = -vec_camRotOPENMVG[i] * vec_camPosOPENMVG[i];
//            Vec3 tj2 = -vec_camRotOPENMVG[j] * vec_camPosOPENMVG[j];

//            Vec3 t2 = ( tj2 - Rj * Ri.transpose() * ti2 ).normalized();
//            Vec3 t3 = vec_camTsOPENMVG[j] - vec_camRotOPENMVG[j] * vec_camRotOPENMVG[i].transpose() * vec_camTsOPENMVG[i];
//            double err1 = tij.cross(t).norm();
//            double err2 = tij.cross(t2).norm();
//            double err3 = tij.cross(t3).norm();
//            cout << err1 << "\t" << err2 << "\t" << err3 << "\n";
////            cout <<  ti2 - vec_camTsOPENMVG[i] << "\n";
//        }
    }
    return  0;



    //选择点构建线性约束方程
//    test_constraints(relatives_R, global_rotations, relativePose_Infos_map, sfm_data, feats_provider, matches_provider);
    vector<vector<double>> A;
    int number_select = 20;
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
    Vec3 t1, t2, t3, t4;
    Vec t_all1(3*global_rotations.size());
    Vec t_all2(3*global_rotations.size());
    Vec t_all3(3*global_rotations.size());
    Vec t_all4(3*global_rotations.size());
    {
        Mat A_eigen(A.size(), A[0].size()-3);
        for(int r=0; r<A.size(); ++r)
            for(int c=0; c<A[0].size()-3; ++c)
            {
                A_eigen(r, c) = A[r][c+3];
            }

        // normalized = 1
//        for(int r=0; r<A_eigen.rows(); ++r)
//        {
//            A_eigen.row(r).normalize();
//        }

        // Ax = 0
        Mat AAT = A_eigen.transpose() * A_eigen;
        cout << "AAT.rows()" << AAT.rows() << ", rank(AAT)" << AAT.fullPivHouseholderQr().rank() << endl;
        Eigen::SelfAdjointEigenSolver<Mat> eigensolver(AAT);
        Vec t = eigensolver.eigenvectors().col(0);

        Vec t1 = eigensolver.eigenvectors().col(0);
        Vec t2 = eigensolver.eigenvectors().col(1);
        Vec t3 = eigensolver.eigenvectors().col(2);
        Vec t4 = eigensolver.eigenvectors().col(3);

        double min_eigen_vals = eigensolver.eigenvalues()[0];
        Vec t_test = AAT * t - min_eigen_vals * t;
        cout << "\n" << eigensolver.eigenvalues() << "\n";
        cout << min_eigen_vals << " " << t_test.norm() << endl;

        Vec3 t0(0, 0, 0);
        t_all1 << t0, t1;
        t_all2 << t0, t2;
        t_all3 << t0, t3;
        t_all4 << t0, t4;
    }

    for(int i=0; i<4; ++i)
    {
        Vec t_all;
        if( i== 0)
            t_all = t_all1;
        if( i== 1)
            t_all = t_all2;
        if( i== 2)
            t_all = t_all3;
        if( i== 3)
            t_all = t_all4;

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
            vec_camPosComputed.push_back(c3);
    //        vec_camPosComputed.push_back(sfm_data_final.GetPoses().at(i).center());
        }
        //计算结果精度
        std::string sOutPath = "/home/lsy/openMVG_Build/software/SfM/tutorial_out/benchmark/test";
        htmlDocument::htmlDocumentStream htmlDocStream("lsy sfm");
//        openMVG::EvaluteToGT(vec_camPosGT, vec_camPosOPENMVG, vec_camRotGT, vec_camRotOPENMVG, sOutPath, &htmlDocStream);
        openMVG::EvaluteToGT(vec_camPosGT, vec_camPosComputed, vec_camRotGT, vec_camRotComputed, sOutPath, &htmlDocStream);
    }


}
