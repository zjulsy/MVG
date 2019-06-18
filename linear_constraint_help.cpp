#include "linear_constraint_help.h"
#include <Eigen/Geometry>


void test_constraints(openMVG::rotation_averaging::RelativeRotations& relatives_R,
                      Hash_Map<IndexT, Mat3>& global_rotations,
                      Hash_Map<Pair, RelativePose_Info>& relativePose_Infos_map,
                      SfM_Data& sfm_data,
                      std::shared_ptr<Features_Provider>& feats_provider,
                      std::shared_ptr<Matches_Provider> &matches_provider)
{
    for(int i=0; i<relatives_R.size(); ++i)
    {
        int index_1 = relatives_R[i].i;
        int index_2 = relatives_R[i].j;
        Mat3 Rij = relatives_R[i].Rij;

        if( global_rotations.count(index_1) == 1 || global_rotations.count(index_2) == 1 )
        {
            Mat3 Ri = global_rotations[index_1];
            Mat3 Rj = global_rotations[index_2];
            Mat3 Rot_To_Identity = Rij * Ri * Rj.transpose(); // motion composition

            float angularErrorDegree = static_cast<float>(R2D(getRotationMagnitude(Rot_To_Identity)));
            if(angularErrorDegree < 3)
            {
                const IndexT I = index_1, J = index_2;

                const View
                    * view_I = sfm_data.views.at(I).get(),
                    * view_J = sfm_data.views.at(J).get();

                const IntrinsicBase
                    * cam_I = sfm_data.GetIntrinsics().at(view_I->id_intrinsic).get(),
                    * cam_J = sfm_data.GetIntrinsics().at(view_J->id_intrinsic).get();

                const Pair current_pair(I, J);

                if( matches_provider->pairWise_matches_.count(current_pair) == 0 )
                {
                    cout << "图像图顺序不对" << endl;
                    continue;
                }

                if(relativePose_Infos_map.count(current_pair) == 0)
                    continue;

                const Pose3 & pose_I = {Mat3::Identity(), Vec3::Zero()};
                const Pose3 & pose_J = relativePose_Infos_map.at(current_pair).relativePose;

                const Mat3& K = dynamic_cast<const cameras::Pinhole_Intrinsic*>(cam_I)->K();

                const matching::IndMatches & matches = matches_provider->pairWise_matches_.at(current_pair);
                size_t number_matches = matches.size();

                //随机选择number_select个对应点
                int number_select = 2;
                vector<int> vec_idx(number_matches), vec_idx_sample(number_select);
                std::iota(vec_idx.begin(), vec_idx.end(), 0);

                std::mt19937 random_generator(std::mt19937::default_seed);
                openMVG::robust::UniformSample(number_select, random_generator, &vec_idx, &vec_idx_sample);

                for(int s = 0; s<number_select; ++s)
                {
                    auto & match = matches[vec_idx_sample[s]];

                    Vec2 x1 = feats_provider->feats_per_view.at(I)[match.i_].coords().cast<double>();
                    Vec2 x2 = feats_provider->feats_per_view.at(J)[match.j_].coords().cast<double>();

                    Vec3 pj, pi;
                    pi = K.inverse() * x1.homogeneous();
                    pj = K.inverse() * x2.homogeneous();
                    pi.normalize();
                    pj.normalize();

                    Vec3 tij = pose_J.translation();
                    cout << pj.transpose() * tij.cross( pose_J.rotation() * pi ) / tij.norm() << " ";

                    Rij = pose_J.rotation();
                    pi = Rij * pi;

                    Vec v1(6), v2(6);
                    v1(0)   = pi(0)*( -pj(1)*Rij(2,0) + pj(2)*Rij(1,0) ) + pi(1)*( pj(0)*Rij(2,0) - pj(2)*Rij(0,0) ) + pi(2)*( -pj(0)*Rij(1,0) + pj(1)*Rij(0,0) );
                    v1(1) = pi(0)*( -pj(1)*Rij(2,1) + pj(2)*Rij(1,1) ) + pi(1)*( pj(0)*Rij(2,1) - pj(2)*Rij(0,1) ) + pi(2)*( -pj(0)*Rij(1,1) + pj(1)*Rij(0,1) );
                    v1(2) = pi(0)*( -pj(1)*Rij(2,2) + pj(2)*Rij(1,2) ) + pi(1)*( pj(0)*Rij(2,2) - pj(2)*Rij(0,2) ) + pi(2)*( -pj(0)*Rij(1,2) + pj(1)*Rij(0,2) );
                    v1(3)   = pi(1)*pj(2) - pi(2)*pj(1);
                    v1(4) = -pi(0)*pj(2) + pi(2)*pj(0);
                    v1(5) = pi(0)*pj(1) - pi(1)*pj(0);

                    Vec3 ti(0, 0, 0), tj = pose_J.translation();
                    v2 << ti, tj;

                    cout << v1.transpose() * v2 << endl;
                }

            }
        }
    }
}


void test_constraints2()
{
    std::random_device rd;  // 将用于获得随机数引擎的种子
    std::mt19937 gen(rd()); // 以 rd() 播种的标准 mersenne_twister_engine
    std::uniform_real_distribution<> dis(0, 1);

    using namespace Eigen;
    Vec3 pi = Eigen::VectorXd::Random(3);
    Vec3 pj = Eigen::VectorXd::Random(3);

    Vec3 ti = Eigen::VectorXd::Random(3);
    Vec3 tj = Eigen::VectorXd::Random(3);

    Matrix3d Rij;
    Rij = AngleAxisd(dis(gen)*M_PI, Vector3d::UnitX())
        * AngleAxisd(dis(gen)*M_PI,  Vector3d::UnitY())
        * AngleAxisd(dis(gen)*M_PI, Vector3d::UnitZ());

    Vec3 ti_r = Rij * ti;
    Vec3 tx = tj - ti_r;

    cout << pj.transpose() * tx.cross( pi ) << " ";

    Vec v1(6), v2(6);
    v1(0)   = pi(0)*( -pj(1)*Rij(2,0) + pj(2)*Rij(1,0) ) + pi(1)*( pj(0)*Rij(2,0) - pj(2)*Rij(0,0) ) + pi(2)*( -pj(0)*Rij(1,0) + pj(1)*Rij(0,0) );
    v1(1) = pi(0)*( -pj(1)*Rij(2,1) + pj(2)*Rij(1,1) ) + pi(1)*( pj(0)*Rij(2,1) - pj(2)*Rij(0,1) ) + pi(2)*( -pj(0)*Rij(1,1) + pj(1)*Rij(0,1) );
    v1(2) = pi(0)*( -pj(1)*Rij(2,2) + pj(2)*Rij(1,2) ) + pi(1)*( pj(0)*Rij(2,2) - pj(2)*Rij(0,2) ) + pi(2)*( -pj(0)*Rij(1,2) + pj(1)*Rij(0,2) );
    v1(3)   = pi(1)*pj(2) - pi(2)*pj(1);
    v1(4) = -pi(0)*pj(2) + pi(2)*pj(0);
    v1(5) = pi(0)*pj(1) - pi(1)*pj(0);

    v2 << ti, tj;

    cout << v1.transpose() * v2 << endl;

}


void get_linear_equation(vector<vector<double>>& A,
                         int number_select,
                         openMVG::rotation_averaging::RelativeRotations& relatives_R,
                         Hash_Map<IndexT, Mat3>& global_rotations,
                         Hash_Map<Pair, RelativePose_Info>& relativePose_Infos_map,
                         SfM_Data& sfm_data,
                         std::shared_ptr<Features_Provider>& feats_provider,
                         std::shared_ptr<Matches_Provider> &matches_provider)
{
    vector<int> num_points_per_img(global_rotations.size(), 0);
    int valid_pair_number = 0;
    for(int i=0; i<relatives_R.size(); ++i)
    {
        int index_1 = relatives_R[i].i;
        int index_2 = relatives_R[i].j;
        Mat3 Rij = relatives_R[i].Rij;

        if( global_rotations.count(index_1) == 1 || global_rotations.count(index_2) == 1 )
        {
            Mat3 Ri = global_rotations[index_1];
            Mat3 Rj = global_rotations[index_2];
            Mat3 Rot_To_Identity = Rij * Ri * Rj.transpose(); // motion composition

            float angularErrorDegree = static_cast<float>(R2D(getRotationMagnitude(Rot_To_Identity)));
            if(angularErrorDegree < 3)
            {
                ++valid_pair_number;

                const IndexT
                I = index_1,
                J = index_2;

                const View
                * view_I = sfm_data.views.at(I).get(),
                * view_J = sfm_data.views.at(J).get();

                const IntrinsicBase
                * cam_I = sfm_data.GetIntrinsics().at(view_I->id_intrinsic).get(),
                * cam_J = sfm_data.GetIntrinsics().at(view_J->id_intrinsic).get();

                const Pair current_pair(I, J);

                if( matches_provider->pairWise_matches_.count(current_pair) == 0 )
                {
                    cout << "图像图顺序不对" << endl;
                    continue;
                }
                if(relativePose_Infos_map.count(current_pair) == 0)
                    continue;

                const Pose3 & pose_I = {Mat3::Identity(), Vec3::Zero()};
                const Pose3 & pose_J = relativePose_Infos_map.at(current_pair).relativePose;

                const matching::IndMatches & matches = matches_provider->pairWise_matches_.at(current_pair);
                size_t number_matches = matches.size();

                vector<int> vec_idx(number_matches), vec_idx_sample(number_select);
                std::iota(vec_idx.begin(), vec_idx.end(), 0);
                std::mt19937 random_generator(std::mt19937::default_seed);
                openMVG::robust::UniformSample(number_select, random_generator, &vec_idx, &vec_idx_sample);

                bool flag = false;
                for(int s = 0; s<number_select; ++s)
                {
                    auto & match = matches[vec_idx_sample[s]];

                    Vec2 x1 = feats_provider->feats_per_view.at(I)[match.i_].coords().cast<double>();
                    Vec2 x2 = feats_provider->feats_per_view.at(J)[match.j_].coords().cast<double>();

                    Vec3 X;
                    TriangulateDLT(pose_I.asMatrix(), (*cam_I)(x1),
                             pose_J.asMatrix(), (*cam_J)(x2), &X);

                    const Mat3& K = dynamic_cast<const cameras::Pinhole_Intrinsic*>(cam_I)->K();
                    double px = K(0, 2), py = K(1, 2);
                    double fx = K(0, 0), fy = K(1, 1);

                    Vec2 x1_project = X.hnormalized();
                    x1_project(0) = px + fx * x1_project(0);
                    x1_project(1) = py + fy * x1_project(1);

                    Vec3 X_project;
                    X_project = pose_J.rotation() * X;
                    // Apply the camera translation
                    X_project += pose_J.translation();
                    Vec2 x2_project = X_project.hnormalized();
                    x2_project(0) = px + fx * x2_project(0);
                    x2_project(1) = py + fy * x2_project(1);

                    double d_max = 0.05;
                    if( x1_project(0)-x1(0) > d_max || x1_project(1)-x1(1) > d_max || x2_project(0)-x2(0) > d_max || x2_project(1)-x2(1) > d_max )
                        continue;
                    flag = true;
                    ++num_points_per_img[I];
                    ++num_points_per_img[J];

                    Vec3 pj, pi;
                    pi = dynamic_cast<const cameras::Pinhole_Intrinsic*>(cam_I)->Kinv() * x1.homogeneous();
                    pj = dynamic_cast<const cameras::Pinhole_Intrinsic*>(cam_I)->Kinv() * x2.homogeneous();
                    Rij = Rj * Ri.transpose();
                    pi = Rij * pi;

//                    pi.normalize();
//                    pj.normalize();

                    vector<double> row_A(3*global_rotations.size(), 0);
                    row_A[3*I]   = pi(0)*( -pj(1)*Rij(2,0) + pj(2)*Rij(1,0) ) + pi(1)*( pj(0)*Rij(2,0) - pj(2)*Rij(0,0) ) + pi(2)*( -pj(0)*Rij(1,0) + pj(1)*Rij(0,0) );
                    row_A[3*I+1] = pi(0)*( -pj(1)*Rij(2,1) + pj(2)*Rij(1,1) ) + pi(1)*( pj(0)*Rij(2,1) - pj(2)*Rij(0,1) ) + pi(2)*( -pj(0)*Rij(1,1) + pj(1)*Rij(0,1) );
                    row_A[3*I+2] = pi(0)*( -pj(1)*Rij(2,2) + pj(2)*Rij(1,2) ) + pi(1)*( pj(0)*Rij(2,2) - pj(2)*Rij(0,2) ) + pi(2)*( -pj(0)*Rij(1,2) + pj(1)*Rij(0,2) );
                    row_A[3*J]   = pi(1)*pj(2) - pi(2)*pj(1);
                    row_A[3*J+1] = -pi(0)*pj(2) + pi(2)*pj(0);
                    row_A[3*J+2] = pi(0)*pj(1) - pi(1)*pj(0);

                    // normalize
                    double sum = row_A[3*I] * row_A[3*I] + row_A[3*I+1] * row_A[3*I+1] + row_A[3*I+2] * row_A[3*I+2] +
                              row_A[3*J] * row_A[3*J] + row_A[3*J+1] * row_A[3*J+1] + row_A[3*J+2] * row_A[3*J+2];
                    row_A[3*I] /= sum;
                    row_A[3*I+1] /= sum;
                    row_A[3*I+2] /= sum;
                    row_A[3*J] /= sum;
                    row_A[3*J+1] /= sum;
                    row_A[3*J+2] /= sum;



                    A.push_back(row_A);
                }
                if( !flag )
                cout << "当前图像对没有点约束" << endl;
            }

        }
    }

    cout << "num_points_per_img : ";
    for(int i : num_points_per_img)
        cout << i << " ";
    cout << endl;
}

Mat directGetT(Hash_Map<Pair, RelativePose_Info>& relativePose_Infos_map, Hash_Map<IndexT, Mat3>& global_rotations, vector<double>& ts,
               vector<Mat3>& vec_Rij, vector<Vec3>& vec_tij, vector< std::pair<int, int> >& vec_ij)
{
    vector<vector<double>> A;

    for(auto& it : relativePose_Infos_map)
    {
        Pair current_pair = it.first;
        int i = current_pair.first;
        int j= current_pair.second;

        Vec3 tij = it.second.relativePose.translation();
        Mat3 Rij = it.second.relativePose.rotation();

        Mat3 Ri = global_rotations[i];
        Mat3 Rj = global_rotations[j];
        Mat3 Rot_To_Identity = Rij * Ri * Rj.transpose(); // motion composition

        float angularErrorDegree = static_cast<float>(R2D(getRotationMagnitude(Rot_To_Identity)));
        if(angularErrorDegree > 3)
            continue;

        Rij = Rj * Ri.transpose();
        vec_Rij.push_back(Rij);
        vec_tij.push_back(tij);
        vec_ij.push_back( {i, j} );

        vector<double> a1(ts.size(), 0);
        a1[3*i] = -tij(1)*Rij(2, 0) + tij(2)*Rij(1, 0);
        a1[3*i+1] = -tij(1)*Rij(2, 1) + tij(2)*Rij(1, 1);
        a1[3*i+2] = -tij(1)*Rij(2, 2) + tij(2)*Rij(1, 2);

        a1[3*j+1] = -tij(2);
        a1[3*j+2] = tij[1];

        vector<double> a2(ts.size(), 0);
        a2[3*i] = tij(0)*Rij(2, 0) - tij(2)*Rij(0, 0);
        a2[3*i+1] = tij(0)*Rij(2, 1) - tij(2)*Rij(0, 1);
        a2[3*i+2] = tij(0)*Rij(2, 2) - tij(2)*Rij(0, 2);

        a2[3*j] = tij(2);
        a2[3*j+2] = -tij[0];

        vector<double> a3(ts.size(), 0);
        a3[3*i] = -tij(0)*Rij(1, 0) + tij(1)*Rij(0, 0);
        a3[3*i+1] = -tij(0)*Rij(1, 1) + tij(1)*Rij(0, 1);
        a3[3*i+2] = -tij(0)*Rij(1, 2) + tij(1)*Rij(0, 2);

        a3[3*j] = -tij(1);
        a3[3*j+1] = tij[0];


        A.push_back(a1);
        A.push_back(a2);
        A.push_back(a3);
    }

    // Ax = 0, t0 = (0,0,0)
    Mat A_eigen(A.size(), A[0].size()-3);
    for(int r=0; r<A.size(); ++r)
        for(int c=0; c<A[0].size()-3; ++c)
        {
            A_eigen(r, c) = A[r][c+3];
        }
    for(int i=0; i<A_eigen.rows(); ++i)
    {
        A_eigen.row(i).normalize();
    }

    Eigen::JacobiSVD<Mat> svd( A_eigen, Eigen::ComputeFullV );
    Vec nullspace = svd.matrixV().col( A_eigen.cols() - 1 );

    for(int i=0; i<ts.size()-3; ++i)
        ts[i+3] = nullspace(i);

    return A_eigen;
}
