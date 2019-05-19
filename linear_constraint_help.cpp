#include "linear_constraint_help.h"

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

                    pi.normalize();
                    pj.normalize();

                    vector<double> row_A(3*global_rotations.size(), 0);
                    row_A[3*I]   = pi(0)*( -pj(1)*Rij(2,0) + pj(2)*Rij(1,0) ) + pi(1)*( pj(0)*Rij(2,0) - pj(2)*Rij(0,0) ) + pi(2)*( -pj(0)*Rij(1,0) + pj(1)*Rij(0,0) );
                    row_A[3*I+1] = pi(0)*( -pj(1)*Rij(2,1) + pj(2)*Rij(1,1) ) + pi(1)*( pj(0)*Rij(2,1) - pj(2)*Rij(0,1) ) + pi(2)*( -pj(0)*Rij(1,1) + pj(1)*Rij(0,1) );
                    row_A[3*I+2] = pi(0)*( -pj(1)*Rij(2,2) + pj(2)*Rij(1,2) ) + pi(1)*( pj(0)*Rij(2,2) - pj(2)*Rij(0,2) ) + pi(2)*( -pj(0)*Rij(1,2) + pj(1)*Rij(0,2) );
                    row_A[3*J]   = pi(1)*pj(2) - pi(2)*pj(1);
                    row_A[3*J+1] = -pi(0)*pj(2) + pi(2)*pj(0);
                    row_A[3*J+2] = pi(0)*pj(1) - pi(1)*pj(0);

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
