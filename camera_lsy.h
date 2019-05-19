#ifndef CAMERA_LSY_H
#define CAMERA_LSY_H

#include <openMVG/cameras/Camera_Pinhole_Radial.hpp>
#include <openMVG/numeric/eigen_alias_definition.hpp>

using namespace openMVG::cameras;
using namespace openMVG;
class camera_lsy : public openMVG::cameras::Pinhole_Intrinsic_Radial_K3
{
    using class_type = camera_lsy;
public:
    camera_lsy(int w = 0, int h = 0,
    double focal = 0.0, double ppx = 0, double ppy = 0,
    double k1 = 0.0, double k2 = 0.0, double k3 = 0.0 )
    : Pinhole_Intrinsic_Radial_K3( w, h, focal, ppx, ppy, k1, k2, k3)
    {}

    void setK(Mat3& K)
    {
        K_ = K;
        Kinv_ = K_.inverse();
    }

    Vec2 cam2ima( const Vec2& p ) const override
    {
//        return p;
        Vec2 p2;
        p2(0) = p(0) * K_(0, 1);
        p2(1) = p(1) * K_(1, 1);
        return p2 + principal_point();
    }

    Vec2 ima2cam( const Vec2& p ) const override
    {
//        return p;
        Vec2 p2 = p -  principal_point();
        p2(0) = p2(0) / K_(0, 1);
        p2(1) = p2(1) / K_(1, 1);
        return p2;
    }
};

#endif // CAMERA_LSY_H
